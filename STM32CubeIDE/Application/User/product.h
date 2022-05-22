#include "main.h"
#include "defines.h"

#ifndef APP_PRODUCT_H_
#define APP_PRODUCT_H_
 /*  Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#ifdef M365_gd32

#define VESC_USART_DMA													     huart3
#define VESC_USART_TX_DMA													 hdma_usart3_tx
#define VESC_USART_RX_DMA													 hdma_usart3_rx

#define APP_USART_DMA														 huart1
#define APP_USART_TX_DMA												     hdma_usart1_tx
#define APP_USART_RX_DMA													 hdma_usart1_rx

//Current Measurement
#define RSHUNT                        										 0.00200
#define AMPLIFICATION_GAIN            										 8.00


// Setting limits
#define HW_LIM_CURRENT			-70.0, 70.0
#define HW_LIM_CURRENT_IN		-70.0, 70.0
#define HW_LIM_CURRENT_ABS		0.0, 100.0
#define HW_LIM_VIN				6.0, 56.0
#define HW_LIM_ERPM				-100e3, 100e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 75.0
#define HW_LIM_F_SW			    4000.0, 50000.0

#define MOT_TMR_MHZ 60
#define HEAP_SIZE_KB 50
#define CPU_MHZ  (120*1000000)
#define APP_PAGE				255
#define CONF_PAGE				254
#define PAGE_SIZE				0x800
#define HW_DEFAULT_ID 122
#define FOC_CONTROL_LOOP_FREQ_DIVIDER 1
#define CURRENT_FILTER_ON()
#define CURRENT_FILTER_OFF()
#define PHASE_FILTER_ON()
#define PHASE_FILTER_OFF()
#define SHUTDOWN_RESET()
#define IS_DRV_FAULT() 0
#define GET_INPUT_VOLTAGE() ((float)ADC3->JDR2*0.013927)

//Hall
#define READ_HALL1() HAL_GPIO_ReadPin(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin)
#define READ_HALL2() HAL_GPIO_ReadPin(M1_HALL_H2_GPIO_Port, M1_HALL_H2_Pin)
#define READ_HALL3() HAL_GPIO_ReadPin(M1_HALL_H3_GPIO_Port, M1_HALL_H3_Pin)
#define MOTOR_TEMP_LPF 0.1
#define ADC_IND_TEMP_MOS 2
#define HW_ADC_CHANNELS 10
#define HW_ADC_CHANNELS_EXTRA 2

#define GET_CURRENT1() (ADC3->JDR1)
#define GET_CURRENT2() (ADC1->JDR1)
#define GET_CURRENT3() (ADC2->JDR1)
#define V_REG 3.3
#define GET_VOLT1() ((float)ADC1->JDR2 / 4096.0 * V_REG)
#define GET_VOLT2() ((float)ADC2->JDR2 / 4096.0 * V_REG)
#define GET_VOLT3() ((float)ADC1->JDR3 / 4096.0 * V_REG)
#define ADC_V_L1				ADC1->JDR2
#define ADC_V_L2				ADC2->JDR2
#define ADC_V_L3				ADC1->JDR3
#define HW_HAS_3_SHUNTS


//Phase Voltage
#define VIN_R1 22000.0
#define VIN_R2 2000.0
#define ADC_VOLTS_PH_FACTOR 1.0
#define ADC_VOLTS_INPUT_FACTOR 1.0

#endif
/****************************************************************************/


#define VESC_TOOL_ENABLE													 1

#define MODE_SLOW_CURR														 0.5
#define MODE_DRIVE_CURR														 0.8
#define MODE_SPORT_CURR														 1.0
#define MODE_SLOW_SPEED														 10
#define MODE_DRIVE_SPEED													 25
#define MODE_SPORT_SPEED													 KMH_NO_LIMIT

#define BATTERY_VOLTAGE_GAIN     											 ((VOLTAGE_DIVIDER_GAIN * ADC_GAIN) * 512.0)
#define CURRENT_FACTOR_A 													 ((2048.0*RSHUNT*AMPLIFICATION_GAIN)/(3.3/2))
#define CURRENT_FACTOR_mA 													 (CURRENT_FACTOR_A/1000.0)
#define FAC_CURRENT 													 	 (-1.0/((2048.0*RSHUNT*AMPLIFICATION_GAIN)/(3.3/2)))
#define MIN_DUTY_PWM												         (32768 * MIN_DUTY_FOR_PWM_FREEWHEEL / 100)

#define DEMCR_TRCENA    0x01000000
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT
#define CPU_CLOCK		64000000


#define PRIO_BELOW_NORMAL 4
#define PRIO_NORMAL  5
#define PRIO_HIGHER  6

#endif /* APP_PRODUCT_H_ */
