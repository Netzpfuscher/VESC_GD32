/*
 * hw_m365.h
 *
 *  Created on: 25.07.2022
 *      Author: jensk
 */

#ifndef APPLICATION_USER_VESC_INC_HW_M365_H_
#define APPLICATION_USER_VESC_INC_HW_M365_H_


#define HW_NAME					"GD32_m365"

//Injected configuration

//ADC1
#define ADC1_INJ_1_ENABLED		1
#define ADC1_INJ_1_CH			ADC_CHANNEL_4
#define ADC1_INJ_1_TIM			ADC_SAMPLETIME_1CYCLE_5

#define ADC1_INJ_2_ENABLED		1
#define ADC1_INJ_2_CH			ADC_CHANNEL_6
#define ADC1_INJ_2_TIM			ADC_SAMPLETIME_13CYCLES_5

#define ADC1_INJ_3_ENABLED		1
#define ADC1_INJ_3_CH			ADC_CHANNEL_9
#define ADC1_INJ_3_TIM			ADC_SAMPLETIME_13CYCLES_5

#define ADC1_INJ_4_ENABLED		0
#define ADC1_INJ_4_CH			ADC_CHANNEL_0
#define ADC1_INJ_4_TIM			ADC_SAMPLETIME_1CYCLE_5

//ADC2
#define ADC2_INJ_1_ENABLED		1
#define ADC2_INJ_1_CH			ADC_CHANNEL_5
#define ADC2_INJ_1_TIM			ADC_SAMPLETIME_1CYCLE_5

#define ADC2_INJ_2_ENABLED		1
#define ADC2_INJ_2_CH			ADC_CHANNEL_7
#define ADC2_INJ_2_TIM			ADC_SAMPLETIME_13CYCLES_5

#define ADC2_INJ_3_ENABLED		0
#define ADC2_INJ_3_CH			ADC_CHANNEL_0
#define ADC2_INJ_3_TIM			ADC_SAMPLETIME_1CYCLE_5

#define ADC2_INJ_4_ENABLED		0
#define ADC2_INJ_4_CH			ADC_CHANNEL_0
#define ADC2_INJ_4_TIM			ADC_SAMPLETIME_1CYCLE_5

//ADC3
#define ADC3_INJ_1_ENABLED		1
#define ADC3_INJ_1_CH			ADC_CHANNEL_3
#define ADC3_INJ_1_TIM			ADC_SAMPLETIME_1CYCLE_5

#define ADC3_INJ_2_ENABLED		0
#define ADC3_INJ_2_CH			ADC_CHANNEL_0
#define ADC3_INJ_2_TIM			ADC_SAMPLETIME_1CYCLE_5

#define ADC3_INJ_3_ENABLED		0
#define ADC3_INJ_3_CH			ADC_CHANNEL_0
#define ADC3_INJ_3_TIM			ADC_SAMPLETIME_1CYCLE_5

#define ADC3_INJ_4_ENABLED		0
#define ADC3_INJ_4_CH			ADC_CHANNEL_0
#define ADC3_INJ_4_TIM			ADC_SAMPLETIME_1CYCLE_5

//COMM
#define VESC_USART_DMA			huart3
#define VESC_USART_TX_DMA		hdma_usart3_tx
#define VESC_USART_RX_DMA		hdma_usart3_rx

#define APP_USART_DMA			huart1
#define APP_USART_TX_DMA		hdma_usart1_tx
#define APP_USART_RX_DMA		hdma_usart1_rx


//Global
#define HW_HAS_3_SHUNTS

//Shutdown
#define HW_SHUTDOWN_HOLD_ON()
#define HW_SHUTDOWN_HOLD_OFF()
#define HW_SAMPLE_SHUTDOWN() 0
#define ENABLE_GATE()
#define DISABLE_GATE()

// NTC Termistors
#define ADC_TEMP_MOS			ADC3
#define ADC_CH_TEMP_MOS			0
#define ADC_IND_TEMP_MOS 		RCM[0].result
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / (4095 - adc_val) - 10000.0)
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(reg_adc[0]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define MOTOR_TEMP_LPF 0.1

//Hall
#define READ_HALL1() HAL_GPIO_ReadPin(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin)
#define READ_HALL2() HAL_GPIO_ReadPin(M1_HALL_H2_GPIO_Port, M1_HALL_H2_Pin)
#define READ_HALL3() HAL_GPIO_ReadPin(M1_HALL_H3_GPIO_Port, M1_HALL_H3_Pin)


//Current
#define RSHUNT                  0.00200
#define AMPLIFICATION_GAIN      8.00
#define FAC_CURRENT 			(-1.0/((2048.0*RSHUNT*AMPLIFICATION_GAIN)/(3.3/2)))
#define ADC_INJ_CH_A			ADC_CHANNEL_4
#define ADC_INJ_CH_B			ADC_CHANNEL_5
#define ADC_INJ_CH_C			ADC_CHANNEL_3

#define GET_CURRENT1() 			(ADC3->JDR1)
#define GET_CURRENT2() 			(ADC1->JDR1)
#define GET_CURRENT3() 			(ADC2->JDR1)

//Voltage
#define V_REG 3.3
#define GET_VOLT1() 			((float)ADC1->JDR2 / 4096.0 * V_REG)
#define GET_VOLT2() 			((float)ADC2->JDR2 / 4096.0 * V_REG)
#define GET_VOLT3() 			((float)ADC1->JDR3 / 4096.0 * V_REG)
#define ADC_V_L1				ADC1->JDR2
#define ADC_V_L2				ADC2->JDR2
#define ADC_V_L3				ADC1->JDR3

//Voltage Dividers
#define VBUS_R1 10000000.0
#define VBUS_R2 620000.0
#define VPHASE_R1 22000.0
#define VPHASE_R2 2000.0

#define ADC_VOLTS_PH_FACTOR 1.0
#define ADC_VOLTS_INPUT_FACTOR 1.0

#define ADC_INPUT_VOLTAGE		ADC3
#define ADC_CH_INPUT_VOLTAGE	2
#define GET_INPUT_VOLTAGE() 	(((float)reg_adc[1] / 4096.0 * V_REG) * ((VBUS_R1 + VBUS_R2) / VBUS_R2) * ADC_VOLTS_INPUT_FACTOR)


//Limits
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

#ifndef HW_MAX_CURRENT_OFFSET
#define HW_MAX_CURRENT_OFFSET 				620
#endif
#ifndef MCCONF_MAX_CURRENT_UNBALANCE
#define MCCONF_MAX_CURRENT_UNBALANCE		(FAC_CURRENT * 512)
#endif
#ifndef MCCONF_MAX_CURRENT_UNBALANCE_RATE
#define MCCONF_MAX_CURRENT_UNBALANCE_RATE	0.3
#endif

//Extra defines
#define HW_DEFAULT_ID 122
#define FOC_CONTROL_LOOP_FREQ_DIVIDER 1
#define CURRENT_FILTER_ON()
#define CURRENT_FILTER_OFF()
#define PHASE_FILTER_ON()
#define PHASE_FILTER_OFF()
#define IS_DRV_FAULT() 0
#define IS_DRV_FAULT_2() 0
#define HW_RESET_DRV_FAULTS()

#endif /* APPLICATION_USER_VESC_INC_HW_M365_H_ */
