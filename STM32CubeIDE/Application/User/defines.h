#ifndef DEFINES_H_
#define DEFINES_H_

#define HW_NAME					"40"

// Firmware version
#define FW_VERSION_MAJOR			6
#define FW_VERSION_MINOR			00

// Set to 0 for building a release and iterate during beta test builds
#define FW_TEST_VERSION_NUMBER		0

#define STM32_UUID					((uint32_t*)0x1FFF7A10)
#define STM32_UUID_8				((uint8_t*)0x1FFF7A10)

#define MS_TO_TICKS( xTimeInMs ) ( ( TickType_t ) ( ( ( TickType_t ) ( xTimeInMs ) * ( TickType_t ) configTICK_RATE_HZ ) / ( TickType_t ) 1000 ) )

#define ANG_TO_DEG(x) (x/(65536.0/360.0))
#define DEG_TO_ANG(x) (x*(65536.0/360.0))

#define ADC_GAIN (float)(3.3 / 4095.0)

#define float_to_s16q16(x) ( x * 65536.0)
#define s16q16_to_float(x) ((float)x / 65536.0)
#define PI_s16q16 ((int32_t)(M_PI*65536.0))

#endif
