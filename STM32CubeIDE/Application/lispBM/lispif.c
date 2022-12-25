/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se
	Copyright 2022 Joel Svensson    svenssonjoel@yahoo.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "lispif.h"
#include "commands.h"
#include "terminal.h"
//#include "flash_helper.h"
#include "buffer.h"
#include "timeout.h"
#include "lispbm.h"
#include "conf_general.h"
#include "task_init.h"
#include "stdarg.h"

#define HEAP_SIZE				1024
#define LISP_MEM_SIZE			LBM_MEMORY_SIZE_8K
#define LISP_MEM_BITMAP_SIZE	LBM_MEMORY_BITMAP_SIZE_8K
#define GC_STACK_SIZE			160
#define PRINT_STACK_SIZE		128
#define EXTENSION_STORAGE_SIZE	200
#define VARIABLE_STORAGE_SIZE	64

static lbm_cons_t heap[HEAP_SIZE] __attribute__ ((aligned (8)));
static uint32_t memory_array[LISP_MEM_SIZE];
static uint32_t bitmap_array[LISP_MEM_BITMAP_SIZE];
static uint32_t gc_stack_storage[GC_STACK_SIZE];
static uint32_t print_stack_storage[PRINT_STACK_SIZE];
static extension_fptr extension_storage[EXTENSION_STORAGE_SIZE];
 static lbm_value variable_storage[VARIABLE_STORAGE_SIZE];

static lbm_string_channel_state_t string_tok_state;
static lbm_char_channel_t string_tok;
static lbm_buffered_channel_state_t buffered_tok_state;
static lbm_char_channel_t buffered_string_tok;

void eval_thread(void * arg);
static bool lisp_thd_running = false;
static bool lisp_is_init = false;

static int repl_cid = -1;

// Private functions
static uint32_t timestamp_callback(void);
static void sleep_callback(uint32_t us);

static SemaphoreHandle_t xSemaphore;

void lispif_init(void) {
	// Do not attempt to start lisp after a watchdog reset, in case lisp
	// was the cause of it.
	// TODO: Anything else to check?
	lbm_eval_init();

	lbm_init(heap, HEAP_SIZE,
						gc_stack_storage, GC_STACK_SIZE,
						memory_array, LISP_MEM_SIZE,
						bitmap_array, LISP_MEM_BITMAP_SIZE,
						print_stack_storage, PRINT_STACK_SIZE,
						extension_storage, EXTENSION_STORAGE_SIZE);

	if (!timeout_had_IWDG_reset() && terminal_get_first_fault() != FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET) {
		lispif_restart(false, true);
	}
	lisp_is_init=true;

	lbm_set_eval_step_quota(50);

	xSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(xSemaphore);
}

void lispif_lock_lbm(void) {
	xSemaphoreTake(xSemaphore,5);
}

void lispif_unlock_lbm(void) {
	xSemaphoreGive(xSemaphore);
}


int lispif_printf_wrapper(const char* format, ...) {
	if(main_uart.phandle==NULL) return 0;
	va_list arg;
	va_start (arg, format);
	int len = commands_printf_lisp(main_uart.phandle, format, arg);
	va_end (arg);
	return len;
}

static void ctx_cb(eval_context_t *ctx, void *arg1, void *arg2) {
	if (arg2 != NULL) {
		lbm_print_value((char*)arg2, 40, ctx->r);
	}

	if (arg1 != NULL) {
		float *res = (float*)arg1;
		*res = 100.0 * (float)ctx->K.max_sp / 256.0;
	}
}

static void print_ctx_info(eval_context_t *ctx, void *arg1, void *arg2) {
	(void) arg1;
	(void) arg2;

	char output[128];

	int print_ret = lbm_print_value(output, sizeof(output), ctx->r);

	commands_printf_lisp(main_uart.phandle, "--------------------------------");
	commands_printf_lisp(main_uart.phandle, "ContextID: %u", ctx->id);
	commands_printf_lisp(main_uart.phandle, "Stack SP: %u",  ctx->K.sp);
	commands_printf_lisp(main_uart.phandle, "Stack SP max: %u", ctx->K.max_sp);
	if (print_ret) {
		commands_printf_lisp(main_uart.phandle, "Value: %s", output);
	} else {
		commands_printf_lisp(main_uart.phandle, "Error: %s", output);
	}
}

static void sym_it(const char *str) {
	commands_printf_lisp(main_uart.phandle, "%s", str);
}

void lispif_process_cmd(unsigned char *data, unsigned int len, PACKET_STATE_t * phandle) {
	if(!lisp_is_init) return;
	COMM_PACKET_ID packet_id;

	packet_id = data[0];
	data++;
	len--;

	switch (packet_id) {
	case COMM_LISP_SET_RUNNING: {
		bool ok = false;
		bool running = data[0];
		lispif_disable_all_events();

		if (!running) {
			int timeout_cnt = 2000;
			lbm_pause_eval();
			while (lbm_get_eval_state() != EVAL_CPS_STATE_PAUSED && timeout_cnt > 0) {
				chThdSleepMilliseconds(1);
				timeout_cnt--;
			}
			ok = timeout_cnt > 0;
		} else {
			ok = lispif_restart(true, true);
		}

		int32_t ind = 0;
		uint8_t send_buffer[PACKET_SIZE(50)];
		uint8_t * buffer = send_buffer + PACKET_HEADER;
		buffer[ind++] = packet_id;
		buffer[ind++] = ok;
		packet_send_packet(send_buffer, ind, phandle);
	} break;


	case COMM_LISP_GET_STATS: {

		float cpu_use = 0.0;
		float heap_use = 0.0;
		float mem_use = 0.0;
		float stack_use = 0.0;

//		static systime_t time_last = 0;
//		if (eval_tp) {
//			cpu_use = 100.0 * (float)eval_tp->p_time / (float)(chVTGetSystemTimeX() - time_last);
//			time_last = chVTGetSystemTimeX();
//			eval_tp->p_time = 0;
//		} else {
//			break;
//		}

		if (lbm_heap_state.gc_num > 0) {
			heap_use = 100.0 * (float)(HEAP_SIZE - lbm_heap_state.gc_last_free) / (float)HEAP_SIZE;
		}

		mem_use = 100.0 * (float)(lbm_memory_num_words() - lbm_memory_num_free()) / (float)lbm_memory_num_words();
		lbm_running_iterator(ctx_cb, &stack_use, NULL);

		int32_t ind = 0;
		uint8_t * send_buffer = pvPortMalloc(512 + PACKET_HEADER);
		uint8_t * buffer = send_buffer + PACKET_HEADER;
		buffer[ind++] = packet_id;
		buffer_append_float16(buffer, cpu_use, 1e2, &ind);
		buffer_append_float16(buffer, heap_use, 1e2, &ind);
		buffer_append_float16(buffer, mem_use, 1e2, &ind);
		buffer_append_float16(buffer, stack_use, 1e2, &ind);

		char r_buf[40];
		r_buf[0] = '\0'; lbm_done_iterator(ctx_cb, NULL, r_buf); r_buf[39] = '\0';
		strcpy((char*)(buffer + ind), r_buf); ind += strlen(r_buf) + 1;

		lbm_value curr = *lbm_get_env_ptr();
		while (lbm_type_of(curr) == LBM_TYPE_CONS) {
			lbm_value key_val = lbm_car(curr);
			if (lbm_type_of(lbm_car(key_val)) == LBM_TYPE_SYMBOL && lbm_is_number(lbm_cdr(key_val))) {
				const char *name = lbm_get_name_by_symbol(lbm_dec_sym(lbm_car(key_val)));
				strcpy((char*)(buffer + ind), name);
				ind += strlen(name) + 1;
				buffer_append_float32_auto(buffer, lbm_dec_as_float(lbm_cdr(key_val)), &ind);
			}

			if (ind > 300) {
				break;
			}

			curr = lbm_cdr(curr);
		}

		for (int i = 0; i < lbm_get_num_variables(); i ++) {
			const char *name = lbm_get_variable_name_by_index(i);
			const lbm_value var = lbm_get_variable_by_index(i);
			if (lbm_is_number(var) && name) {
				strcpy((char*)(buffer + ind), name);
				ind += strlen(name) + 1;
				buffer_append_float32_auto(buffer, lbm_dec_as_float(var), &ind);

				if (ind > 300) {
					break;
				}
			}
		}

		packet_send_packet(send_buffer, ind, phandle);
		vPortFree(send_buffer);
	} break;

	case COMM_LISP_REPL_CMD: {
		if (!lisp_thd_running) {
			lispif_restart(true, false);
		}

		if (lisp_thd_running) {
			lispif_lock_lbm();	 
			char *str = (char*)data;

			if (len <= 1) {
				commands_printf_lisp(phandle, ">");
			} else if (len >= 5 && strncmp(str, ":help", 5) == 0) {
				commands_printf_lisp(phandle, "== Special Commands ==");
				commands_printf_lisp(phandle,
						":help\n"
						"  Print this help text");
				commands_printf_lisp(phandle,
						":info\n"
						"  Print info about memory usage, allocated arrays and garbage collection");
				commands_printf_lisp(phandle,
						":env\n"
						"  Print current environment and variables");
				commands_printf_lisp(phandle,
						":ctxs\n"
						"  Print context (threads) info");
				commands_printf_lisp(phandle,
						":symbols\n"
						"  Print symbol names");
				commands_printf_lisp(phandle,
						":reset\n"
						"  Reset LBM");
				commands_printf_lisp(phandle,
						":pause\n"
						"  Pause LBM");
				commands_printf_lisp(phandle,
						":continue\n"
						"  Continue running LBM");
				commands_printf_lisp(phandle,
						":undef <symbol_name>\n"
						"  Undefine symbol");
				commands_printf_lisp(phandle,
						":verb\n"
						"  Toggle verbose error messages");
				commands_printf_lisp(phandle, " ");
				commands_printf_lisp(phandle, "Anything else will be evaluated as an expression in LBM.");
				commands_printf_lisp(phandle, " ");
			} else if (len >= 5 && strncmp(str, ":info", 5) == 0) {
				commands_printf_lisp(phandle, "--(LISP HEAP)-----------------------------------------------\n");
				commands_printf_lisp(phandle, "Heap size: %u Bytes\n", HEAP_SIZE * 8);
				commands_printf_lisp(phandle, "Used cons cells: %d\n", HEAP_SIZE - lbm_heap_num_free());
				commands_printf_lisp(phandle, "Free cons cells: %d\n", lbm_heap_num_free());
				commands_printf_lisp(phandle, "GC counter: %d\n", lbm_heap_state.gc_num);
				commands_printf_lisp(phandle, "Recovered: %d\n", lbm_heap_state.gc_recovered);
				commands_printf_lisp(phandle, "Recovered arrays: %u\n", lbm_heap_state.gc_recovered_arrays);
				commands_printf_lisp(phandle, "Marked: %d\n", lbm_heap_state.gc_marked);
				commands_printf_lisp(phandle, "--(Symbol and Array memory)---------------------------------\n");
				commands_printf_lisp(phandle, "Memory size: %u Words\n", lbm_memory_num_words());
				commands_printf_lisp(phandle, "Memory free: %u Words\n", lbm_memory_num_free());
				commands_printf_lisp(phandle, "Allocated arrays: %u\n", lbm_heap_state.num_alloc_arrays);
				commands_printf_lisp(phandle, "Symbol table size: %u Bytes\n", lbm_get_symbol_table_size());
			} else if (strncmp(str, ":env", 4) == 0) {
				lbm_value curr = *lbm_get_env_ptr();
				char output[128];

				commands_printf_lisp(phandle, "Environment:\n");
				while (lbm_type_of(curr) == LBM_TYPE_CONS) {
					lbm_print_value(output, sizeof(output), lbm_car(curr));
					curr = lbm_cdr(curr);
					commands_printf_lisp(phandle, "  %s",output);
				}
				commands_printf_lisp(phandle, "Variables:");
				for (int i = 0; i < lbm_get_num_variables(); i ++) {
					const char *name = lbm_get_variable_name_by_index(i);
					lbm_print_value(output,1024, lbm_get_variable_by_index(i));
					commands_printf_lisp(phandle, "  %s = %s", name ? name : "error", output);
				}
			} else if (strncmp(str, ":ctxs", 5) == 0) {
				commands_printf_lisp(phandle, "****** Running contexts ******");
				lbm_running_iterator(print_ctx_info, NULL, NULL);
				commands_printf_lisp(phandle, "****** Blocked contexts ******");
				lbm_blocked_iterator(print_ctx_info, NULL, NULL);
				commands_printf_lisp(phandle, "****** Done contexts ******");
				lbm_done_iterator(print_ctx_info, NULL, NULL);
			} else if (strncmp(str, ":symbols", 8) == 0) {
				lbm_symrepr_name_iterator(sym_it);
				commands_printf_lisp(phandle, " ");
			} else if (strncmp(str, ":reset", 6) == 0) {
				commands_printf_lisp(phandle, lispif_restart(false, conf_general_code_size(CODE_IND_LISP) > 0) ?
						"Reset OK\n\n" : "Reset Failed\n\n");
			} else if (strncmp(str, ":pause", 6) == 0) {
				lbm_pause_eval_with_gc(30);
				while(lbm_get_eval_state() != EVAL_CPS_STATE_PAUSED) {
					lbm_pause_eval();
					sleep_callback(1);
				}
				commands_printf_lisp(phandle, "Evaluator paused\n");
			} else if (strncmp(str, ":continue", 9) == 0) {
				lbm_continue_eval();
			} else if (strncmp(str, ":undef", 6) == 0) {
				lbm_pause_eval();
				while(lbm_get_eval_state() != EVAL_CPS_STATE_PAUSED) {
					lbm_pause_eval();
					sleep_callback(1);
				}
				char *sym = str + 7;
				commands_printf_lisp(phandle, "undefining: %s", sym);
				commands_printf_lisp(phandle, "%s", lbm_undefine(sym) ? "Cleared bindings" : "No definition found");
				lbm_continue_eval();
			} else if (strncmp(str, ":verb", 5) == 0) {
				static bool verbose_now = false;
				verbose_now = !verbose_now;
				lbm_set_verbose(verbose_now);
				commands_printf_lisp(phandle, "Verbose errors %s", verbose_now ? "Enabled" : "Disabled");
			} else {
				bool ok = true;
				int timeout_cnt = 1000;
				lbm_pause_eval_with_gc(30);
				while (lbm_get_eval_state() != EVAL_CPS_STATE_PAUSED && timeout_cnt > 0) {
					vTaskDelay(1);
					timeout_cnt--;
				}
				ok = timeout_cnt > 0;

				if (ok) {
					lbm_create_string_char_channel(&string_tok_state, &string_tok, (char*)data);
					//if (reply_func != NULL) {
						repl_cid = lbm_load_and_eval_expression(&string_tok);
						lbm_continue_eval();
						lbm_wait_ctx(repl_cid, 500);
					//}
					repl_cid = -1;
				} else {
					commands_printf_lisp(phandle, "Could not pause");
				}
			}
		} else {
			commands_printf_lisp(phandle, "LispBM is not running");
		}
	} break;

	case COMM_LISP_STREAM_CODE: {
		int32_t ind = 0;
		int32_t offset = buffer_get_int32(data, &ind);
		int32_t tot_len = buffer_get_int32(data, &ind);
		int8_t restart = data[ind++];

		static bool buffered_channel_created = false;
		static int32_t offset_last = -1;
		static int16_t result_last = -1;

		if (offset == 0) {
			if (!lisp_thd_running) {
				lispif_restart(true, restart == 2 ? true : false);
				buffered_channel_created = false;
			} else if (restart == 1) {
				lispif_restart(true, false);
				buffered_channel_created = false;
			} else if (restart == 2) {
				lispif_restart(true, true);
				buffered_channel_created = false;
			}
		}

		int32_t send_ind = 0;
		uint8_t buffer[PACKET_SIZE(50)];
		uint8_t * send_buffer = buffer + PACKET_HEADER;
		send_buffer[send_ind++] = packet_id;
		buffer_append_int32(send_buffer, offset, &send_ind);

		if (offset_last == offset) {
			buffer_append_int16(send_buffer, result_last, &send_ind);
			packet_send_packet(buffer, ind, phandle);
			break;
		}

		offset_last = offset;

		if (!lisp_thd_running) {
			result_last = -1;
			offset_last = -1;
			buffer_append_int16(send_buffer, result_last, &send_ind);
			packet_send_packet(buffer, ind, phandle);
			break;
		}

		if (offset == 0) {
			if (buffered_channel_created) {
				int timeout = 1500;
				while (!buffered_tok_state.reader_closed) {
					lbm_channel_writer_close(&buffered_string_tok);
					chThdSleepMilliseconds(1);
					timeout--;
					if (timeout == 0) {
						break;
					}
				}

				if (timeout == 0) {
					result_last = -2;
					offset_last = -1;
					buffer_append_int16(send_buffer, result_last, &send_ind);
					commands_printf_lisp(phandle, "Reader not closing");
					packet_send_packet(buffer, ind, phandle);
					break;
				}
			}

			int timeout_cnt = 1000;
			lispif_lock_lbm();
			lbm_pause_eval_with_gc(30);
			while (lbm_get_eval_state() != EVAL_CPS_STATE_PAUSED && timeout_cnt > 0) {
				vTaskDelay(1);
				timeout_cnt--;
			}

			if (timeout_cnt == 0) {
				lispif_unlock_lbm();
				result_last = -3;
				offset_last = -1;
				buffer_append_int16(send_buffer, result_last, &send_ind);
				commands_printf_lisp(phandle, "Could not pause");
				packet_send_packet(buffer, ind, phandle);
				break;
			}

			lbm_create_buffered_char_channel(&buffered_tok_state, &buffered_string_tok);

			if (lbm_load_and_eval_program(&buffered_string_tok) <= 0) {
				lispif_unlock_lbm();
				result_last = -4;
				offset_last = -1;
				buffer_append_int16(send_buffer, result_last, &send_ind);
				commands_printf_lisp(phandle, "Could not start eval");
				packet_send_packet(buffer, ind, phandle);
				break;
			}

			lbm_continue_eval();
			buffered_channel_created = true;
			lispif_unlock_lbm();
		}

		int32_t written = 0;
		int timeout = 1500;
		while (ind < (int32_t)len) {
			int ch_res = lbm_channel_write(&buffered_string_tok, (char)data[ind]);

			if (ch_res == CHANNEL_SUCCESS) {
				ind++;
				written++;
				timeout = 1500;
			} else if (ch_res == CHANNEL_READER_CLOSED) {
				break;
			} else {
				chThdSleepMilliseconds(1);
				if (timeout == 0) {
					break;
				}
				timeout--;
			}
		}

		if (ind == (int32_t)len) {
			if ((offset + written) == tot_len) {
				lbm_channel_writer_close(&buffered_string_tok);
				offset_last = -1;
				commands_printf_lisp(phandle, "Stream done, starting...");
			}

			result_last = 0;
			buffer_append_int16(send_buffer, result_last, &send_ind);
		} else {
			if (timeout == 0) {
				result_last = -5;
				offset_last = -1;
				buffer_append_int16(send_buffer, result_last, &send_ind);
				commands_printf_lisp(phandle, "Stream timed out");
			} else {
				result_last = -6;
				offset_last = -1;
				buffer_append_int16(send_buffer, result_last, &send_ind);
				commands_printf_lisp(phandle, "Stream closed");
			}
		}

		packet_send_packet(buffer, ind, phandle);
	} break;

	default:
		break;
	}
}

static void done_callback(eval_context_t *ctx) {
	lbm_cid cid = ctx->id;
	lbm_value t = ctx->r;

	if (cid == repl_cid) {
		char output[128];
		lbm_print_value(output, sizeof(output), t);
		commands_printf_lisp(main_uart.phandle, "> %s", output);
	}
}

bool lispif_restart(bool print, bool load_code) {
	bool res = false;

	//lispif_stop_lib();

	char *code_data = (char*)conf_general_code_data(CODE_IND_LISP);
	int32_t code_len = conf_general_code_size(CODE_IND_LISP);
	if(code_len==-1) code_len=0;

	if (!load_code || (code_data != 0 && code_len > 0)) {
		lispif_disable_all_events();					  
		if (!lisp_thd_running) {
			lbm_init(heap, HEAP_SIZE,
					gc_stack_storage, GC_STACK_SIZE,
					memory_array, LISP_MEM_SIZE,
					bitmap_array, LISP_MEM_BITMAP_SIZE,
					print_stack_storage, PRINT_STACK_SIZE,
					extension_storage, EXTENSION_STORAGE_SIZE);
			lbm_variables_init(variable_storage, VARIABLE_STORAGE_SIZE);

			lbm_set_timestamp_us_callback(timestamp_callback);
			lbm_set_usleep_callback(sleep_callback);
			lbm_set_printf_callback(lispif_printf_wrapper);
			lbm_set_ctx_done_callback(done_callback);
			xTaskCreate(eval_thread, "tskEval", 2048, NULL, PRIO_NORMAL, NULL);
			lisp_thd_running = true;
		} else {
			lbm_pause_eval();
			while (lbm_get_eval_state() != EVAL_CPS_STATE_PAUSED) {
				lbm_pause_eval();
				chThdSleepMilliseconds(1);
			}

			lbm_init(heap, HEAP_SIZE,
					gc_stack_storage, GC_STACK_SIZE,
					memory_array, LISP_MEM_SIZE,
					bitmap_array, LISP_MEM_BITMAP_SIZE,
					print_stack_storage, PRINT_STACK_SIZE,
					extension_storage, EXTENSION_STORAGE_SIZE);
			lbm_variables_init(variable_storage, VARIABLE_STORAGE_SIZE);
		}

		lbm_pause_eval();
		while (lbm_get_eval_state() != EVAL_CPS_STATE_PAUSED) {
								
			lbm_pause_eval();
			chThdSleepMilliseconds(1);
		}

		lispif_load_vesc_extensions();
		lbm_set_dynamic_load_callback(lispif_vesc_dynamic_loader);

				  
		int code_chars = strnlen(code_data, code_len);

		// Load imports
		if (code_len > code_chars + 3) {
			int32_t ind = code_chars + 1;
			uint16_t num_imports = buffer_get_uint16((uint8_t*)code_data, &ind);

			if (num_imports > 0 && num_imports < 500) {
				for (int i = 0;i < num_imports;i++) {
					char *name = code_data + ind;
					ind += strlen(name) + 1;
					int32_t offset = buffer_get_int32((uint8_t*)code_data, &ind);
					int32_t len = buffer_get_int32((uint8_t*)code_data, &ind);

					lbm_value val;
					if (lbm_share_array(&val, code_data + offset, LBM_TYPE_BYTE, len)) {
						lbm_define(name, val);
					}
				}
			}
		}

		if (load_code) {
			if (print) {
				commands_printf_lisp(main_uart.phandle, "Parsing %d characters", code_chars);
			}

			lbm_create_string_char_channel(&string_tok_state, &string_tok, code_data);
			lbm_load_and_eval_program(&string_tok);
		}

		lbm_continue_eval();

		res = true;
	}

	return res;
}

static uint32_t timestamp_callback(void) {
	systime_t t = xTaskGetTickCount();
	return (uint32_t) ((1000000 / configTICK_RATE_HZ) * t);
}

static void sleep_callback(uint32_t us) {
	//chThdSleepMicroseconds(us);
	float ms = (float)us / 1000.0;
	vTaskDelay(MS_TO_TICKS(ms));
}

void eval_thread(void * arg){
	(void)arg;
	//eval_tp = chThdGetSelfX();
	//chRegSetThreadName("Lisp Eval");
	lbm_run_eval();
	vTaskDelete(NULL);
	vTaskDelay(portMAX_DELAY);
}
