/*
 * hw.h
 *
 *  Created on: 26.05.2022
 *      Author: jensk
 */

#ifndef APPLICATION_USER_VESC_INC_HW_H_
#define APPLICATION_USER_VESC_INC_HW_H_

#include "main.h"
#include "stdbool.h"

//Set hw here:
#define HW_MP2

#ifdef HW_M365
#include "hw_m365_core.h"
#include "hw_m365.h"
#endif

#ifdef HW_M365_20S
#include "hw_m365_core.h"
#include "hw_m365_20s.h"
#endif

#ifdef HW_MP2
#include "hw_mp2_core.h"
#include "hw_mp2.h"
#endif

// Firmware version
#define FW_VERSION_MAJOR			6
#define FW_VERSION_MINOR			00

// Set to 0 for building a release and iterate during beta test builds
#define FW_TEST_VERSION_NUMBER		0

#define STM32_UUID					((uint32_t*)0x1FFF7A10)
#define STM32_UUID_8				((uint8_t*) 0x1FFF7A10)


//RTOS
#define HEAP_SIZE_KB 35
#define CPU_PLL_MUL     RCC_PLL_MUL5
#define CPU_MHZ  		(120*1000000)
#define MS_TO_TICKS( xTimeInMs ) ( ( TickType_t ) ( ( ( TickType_t ) ( xTimeInMs ) * ( TickType_t ) configTICK_RATE_HZ ) / ( TickType_t ) 1000 ) )
#define ST2MS(n) (((n) * 1000UL + configTICK_RATE_HZ - 1UL) / configTICK_RATE_HZ)

#define PRIO_BELOW_NORMAL 4
#define PRIO_NORMAL  5
#define PRIO_HIGHER  6


//Debug
#define DEMCR_TRCENA    0x01000000
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT

//#define DEBUG_ISR


#endif /* APPLICATION_USER_VESC_INC_HW_H_ */
