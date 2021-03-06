/*
 * global.h
 *
 *  Created on: 03.02.2013
 *      Author: thomas
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#ifdef CONFIG_CONTROLCAN
#define CONFIG_HOMECAN_CAN
#define CONFIG_INPUT
#define CONFIG_OUTPUT
#define CONFIG_RAFFSTORE
#define CONFIG_ELTAKO
#define CONFIG_LED
#define CONFIG_SSR
#define CONFIG_TEMP
#define CONFIG_I2C

#elif CONFIG_MOTIONCAN
#define CONFIG_HOMECAN_CAN
#define CONFIG_MOTION

#elif CONFIG_SENSORCAN
#define CONFIG_HOMECAN_CAN
#define CONFIG_INPUT
#define CONFIG_LED
#define CONFIG_TEMP
#define CONFIG_ONEWIRE
#define CONFIG_IR
#define CONFIG_BUZZER
#define CONFIG_ANALOG

#elif CONFIG_KEYPADCAN
#define CONFIG_HOMECAN_CAN
#define CONFIG_INPUT
#define CONFIG_KEYPAD
#define CONFIG_BUZZER
#define CONFIG_ANALOG

#elif CONFIG_NETWORKCAN
#define CONFIG_HOMECAN_GATEWAY
#define CONFIG_HOMECAN_UDP
#define CONFIG_HOMECAN_CAN

#elif CONFIG_KWBLAN
#define CONFIG_HOMECAN_UDP
#define CONFIG_KWB
#define CONFIG_INPUT
#define CONFIG_OUTPUT
#define CONFIG_TEMP
#define CONFIG_I2C
#define CONFIG_POTIO
#endif

#endif /* GLOBAL_H_ */
