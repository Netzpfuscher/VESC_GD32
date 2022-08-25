#ifndef TERMINAL_H_
#define TERMINAL_H_

#include "datatypes.h"
#include "packet.h"

// Functions
void terminal_process_string(char *str, PACKET_STATE_t * phandle);
void terminal_add_fault_data(fault_data *data);
mc_fault_code terminal_get_first_fault(void);
void terminal_register_command_callback(
		const char* command,
		const char *help,
		const char *arg_names,
		void(*cbf)(PACKET_STATE_t * phandle, int argc, const char **argv));
void terminal_unregister_callback(void(*cbf)(PACKET_STATE_t * phandle, int argc, const char **argv));

#endif /* TERMINAL_H_ */
