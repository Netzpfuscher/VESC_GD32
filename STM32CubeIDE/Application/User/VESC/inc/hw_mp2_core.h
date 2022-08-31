/*
 * hw_m365_core.h
 *
 *  Created on: 29.07.2022
 *      Author: jensk
 */

#ifndef HW_MP2_CORE_H_
#define HW_MP2_CORE_H_

#ifdef HW_MP2

//Injected configuration

//ADC1
#define ADC1_INJ_1_ENABLED		1
#define ADC1_INJ_1_PORT			GPIOA
#define ADC1_INJ_1_PIN			GPIO_PIN_4
#define ADC1_INJ_1_CH			ADC_CHANNEL_4
#define ADC1_INJ_1_TIM			ADC_SAMPLETIME_1CYCLE_5

#define ADC1_INJ_2_ENABLED		1
#define ADC1_INJ_2_PORT			GPIOA
#define ADC1_INJ_2_PIN			GPIO_PIN_6
#define ADC1_INJ_2_CH			ADC_CHANNEL_6
#define ADC1_INJ_2_TIM			ADC_SAMPLETIME_13CYCLES_5

#define ADC1_INJ_3_ENABLED		1
#define ADC1_INJ_3_PORT			GPIOB
#define ADC1_INJ_3_PIN			GPIO_PIN_1
#define ADC1_INJ_3_CH			ADC_CHANNEL_9
#define ADC1_INJ_3_TIM			ADC_SAMPLETIME_13CYCLES_5

#define ADC1_INJ_4_ENABLED		0
#define ADC1_INJ_4_PORT			NULL
#define ADC1_INJ_4_PIN			NULL
#define ADC1_INJ_4_CH			ADC_CHANNEL_0
#define ADC1_INJ_4_TIM			ADC_SAMPLETIME_1CYCLE_5

//ADC2
#define ADC2_INJ_1_ENABLED		1
#define ADC2_INJ_1_PORT			GPIOA
#define ADC2_INJ_1_PIN			GPIO_PIN_5
#define ADC2_INJ_1_CH			ADC_CHANNEL_5
#define ADC2_INJ_1_TIM			ADC_SAMPLETIME_1CYCLE_5

#define ADC2_INJ_2_ENABLED		1
#define ADC2_INJ_2_PORT			GPIOA
#define ADC2_INJ_2_PIN			GPIO_PIN_7
#define ADC2_INJ_2_CH			ADC_CHANNEL_7
#define ADC2_INJ_2_TIM			ADC_SAMPLETIME_13CYCLES_5

#define ADC2_INJ_3_ENABLED		0
#define ADC2_INJ_3_PORT			NULL
#define ADC2_INJ_3_PIN			NULL
#define ADC2_INJ_3_CH			ADC_CHANNEL_0
#define ADC2_INJ_3_TIM			ADC_SAMPLETIME_1CYCLE_5

#define ADC2_INJ_4_ENABLED		0
#define ADC2_INJ_4_PORT			NULL
#define ADC2_INJ_4_PIN			NULL
#define ADC2_INJ_4_CH			ADC_CHANNEL_0
#define ADC2_INJ_4_TIM			ADC_SAMPLETIME_1CYCLE_5

//ADC3
#define ADC3_INJ_1_ENABLED		1
#define ADC3_INJ_1_PORT			GPIOA
#define ADC3_INJ_1_PIN			GPIO_PIN_3
#define ADC3_INJ_1_CH			ADC_CHANNEL_3
#define ADC3_INJ_1_TIM			ADC_SAMPLETIME_1CYCLE_5

#define ADC3_INJ_2_ENABLED		1
#define ADC3_INJ_2_PORT			GPIOA
#define ADC3_INJ_2_PIN			GPIO_PIN_1
#define ADC3_INJ_2_CH			ADC_CHANNEL_1
#define ADC3_INJ_2_TIM			ADC_SAMPLETIME_13CYCLES_5

#define ADC3_INJ_3_ENABLED		0
#define ADC3_INJ_3_PORT			NULL
#define ADC3_INJ_3_PIN			NULL
#define ADC3_INJ_3_CH			ADC_CHANNEL_0
#define ADC3_INJ_3_TIM			ADC_SAMPLETIME_71CYCLES_5

#define ADC3_INJ_4_ENABLED		0
#define ADC3_INJ_4_PORT			NULL
#define ADC3_INJ_4_PIN			NULL
#define ADC3_INJ_4_CH			ADC_CHANNEL_0
#define ADC3_INJ_4_TIM			ADC_SAMPLETIME_1CYCLE_5


//Regular
#define ADC3_REG_0_ENABLED		1
#define ADC3_REG_0_PORT			GPIOA
#define ADC3_REG_0_PIN			GPIO_PIN_0
#define ADC3_REG_0_CH			ADC_CHANNEL_0

#define ADC3_REG_1_ENABLED		1
#define ADC3_REG_1_PORT			GPIOA
#define ADC3_REG_1_PIN			GPIO_PIN_2
#define ADC3_REG_1_CH			ADC_CHANNEL_2

//COMM
#define VESC_USART_DMA			huart1
#define VESC_USART_TX_DMA		hdma_usart3_tx
#define VESC_USART_RX_DMA		hdma_usart3_rx

#define APP_USART_DMA			huart3
#define APP_USART_TX_DMA		hdma_usart1_tx
#define APP_USART_RX_DMA		hdma_usart1_rx

//Global
#define HW_HAS_3_SHUNTS
#define USE_LISPBM

//Shutdown
#define HW_SHUTDOWN_HOLD_ON()
#define HW_SHUTDOWN_HOLD_OFF()
#define HW_SAMPLE_SHUTDOWN() 0
#define ENABLE_GATE()
#define DISABLE_GATE()

//UART
#define HW_UART_RX_PORT			GPIOC			//Button on m365 not UART
#define HW_UART_TX_PORT			GPIOB

#define HW_UART_RX_PIN			GPIO_PIN_14		//Button on m365 not UART
#define HW_UART_TX_PIN			GPIO_PIN_6


//GPIO
#define HW_ICU_GPIO				GPIOC
#define HW_ICU_PIN				GPIO_PIN_14

//LED
#define LED1_ENA				0
#define LED1_PORT				GPIOD
#define LED1_PIN				GPIO_PIN_1

//Power enable
#define TPS_ENA					0
#define TPS_PORT				GPIOC
#define TPS_PIN					GPIO_PIN_15

//Conversion
#define V_REG 3.3
// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ch / 4096.0 * V_REG)


// NTC Termistors
#define ADC_TEMP_MOS			ADC3
#define ADC_CH_TEMP_MOS			0
#define ADC_IND_TEMP_MOS 		RCM[0].result
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / (4095 - adc_val) - 10000.0)
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(reg_adc[0]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define MOTOR_TEMP_LPF 0.1

//Hall
#define HW_HALL_ENC_GPIO1 		GPIOC
#define HW_HALL_ENC_GPIO2 		GPIOC
#define HW_HALL_ENC_GPIO3 		GPIOC

#define HW_HALL_ENC_PIN1		GPIO_PIN_13
#define HW_HALL_ENC_PIN2		GPIO_PIN_14
#define HW_HALL_ENC_PIN3		GPIO_PIN_15

#define READ_HALL1() 			HAL_GPIO_ReadPin(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2() 			HAL_GPIO_ReadPin(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3() 			HAL_GPIO_ReadPin(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

//Ext ADC
#define ADC_IND_EXT				0 //ADC3->JDR2
#define ADC_IND_EXT2			0
#define ADC_IND_EXT3			0

//Aux
#define AUX_PORT				GPIOA
#define AUX_PIN					GPIO_PIN_15
#define AUX2_PORT
#define AUX2_PIN

#define AUX_ON()				HAL_GPIO_WritePin(AUX_PORT, AUX_PIN, GPIO_PIN_SET);
#define AUX_OFF()				HAL_GPIO_WritePin(AUX_PORT, AUX_PIN, GPIO_PIN_RESET);
#define AUX2_ON()
#define AUX2_OFF()

//Buttons
#define READ_BUTTON_UART_CC		1		//HAL_GPIO_ReadPin(HW_UART_TX_PORT, HW_UART_TX_PIN)
#define READ_BUTTON_UART_REV	1		//HAL_GPIO_ReadPin(HW_UART_RX_PORT, HW_UART_RX_PIN)
#define READ_BUTTON_ICU_CC		1		//HAL_GPIO_ReadPin(HW_ICU_GPIO, HW_ICU_PIN)

//Current

#define FAC_CURRENT 			(-1.0/((2048.0*RSHUNT*AMPLIFICATION_GAIN)/(3.3/2)))

#define GET_CURRENT1() 			(ADC3->JDR1)
#define GET_CURRENT2() 			(ADC1->JDR1)
#define GET_CURRENT3() 			(ADC2->JDR1)

//Voltage

#define GET_VOLT1() 			((float)ADC1->JDR2 / 4096.0 * V_REG)
#define GET_VOLT2() 			((float)ADC2->JDR2 / 4096.0 * V_REG)
#define GET_VOLT3() 			((float)ADC1->JDR3 / 4096.0 * V_REG)
#define ADC_V_L1				ADC1->JDR2
#define ADC_V_L2				ADC2->JDR2
#define ADC_V_L3				ADC1->JDR3

#define ADC_VOLTS_PH_FACTOR 1.0
#define ADC_VOLTS_INPUT_FACTOR 1.0

#define ADC_INPUT_VOLTAGE		ADC3
#define ADC_CH_INPUT_VOLTAGE	2
#define GET_INPUT_VOLTAGE() 	(((float)reg_adc[1] / 4096.0 * V_REG) * ((VBUS_R1 + VBUS_R2) / VBUS_R2) * ADC_VOLTS_INPUT_FACTOR)


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

#endif

#endif /* HW_MP2_CORE_H_ */
