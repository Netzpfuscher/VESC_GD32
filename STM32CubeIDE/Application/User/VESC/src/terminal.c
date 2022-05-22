#include "terminal.h"
#include <string.h>
#include "datatypes.h"
#include "VescCommand.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "product.h"
#include "defines.h"
#include "stdlib.h"
#include "ninebot.h"
#include "conf_general.h"
#include "utils.h"
#include "mcpwm_foc.h"

void terminal_top(PACKET_STATE_t * phandle){
    TaskStatus_t * taskStats;
    uint32_t taskCount = uxTaskGetNumberOfTasks();
    uint32_t sysTime;

    taskStats = pvPortMalloc( taskCount * sizeof( TaskStatus_t ) );
    if(taskStats){
        taskCount = uxTaskGetSystemState(taskStats, taskCount, &sysTime);

        commands_printf(phandle, "Task info:");


        commands_printf(phandle, "FOC ms: %f ", mcpwm_foc_get_last_adc_isr_duration());

        commands_printf(phandle, "Tasks: %d",  taskCount);

        uint32_t heapRemaining = xPortGetFreeHeapSize();
        commands_printf(phandle, "Mem: %db Free: %db Used: %db (%d%%)", configTOTAL_HEAP_SIZE, heapRemaining, configTOTAL_HEAP_SIZE - heapRemaining, ((configTOTAL_HEAP_SIZE - heapRemaining) * 100) / configTOTAL_HEAP_SIZE);

        uint32_t currTask = 0;
        for(;currTask < taskCount; currTask++){
			char name[configMAX_TASK_NAME_LEN+1];
			strncpy(name, taskStats[currTask].pcTaskName, configMAX_TASK_NAME_LEN);
			commands_printf(phandle, "%d Name: %s State: %s Runtime: %d Stack free: %d", taskStats[currTask].xTaskNumber, name, SYS_getTaskStateString(taskStats[currTask].eCurrentState), taskStats[currTask].ulRunTimeCounter, taskStats[currTask].usStackHighWaterMark);
        }
        commands_printf(phandle, "EOL");
        vPortFree(taskStats);
    }
}

#if MUSIC_ENABLE
extern MUSIC_PARAM bldc_music;
#endif
void terminal_process_string(char *str, PACKET_STATE_t * phandle) {
	enum { kMaxArgs = 16 };
	int argc = 0;
	char *argv[kMaxArgs];

	char *p2 = strtok(str, " ");
	while (p2 && argc < kMaxArgs) {
		argv[argc++] = p2;
		p2 = strtok(0, " ");
	}

	if (argc == 0) {
		commands_printf(phandle, "No command received\n");
		return;
	}

	if (strcmp(argv[0], "ping") == 0) {
		commands_printf(phandle, "pong\n");
	}else if (strcmp(argv[0], "top") == 0){
		terminal_top(phandle);
	}

}
