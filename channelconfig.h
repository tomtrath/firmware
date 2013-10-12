/*
 * channelconfig.h
 *
 *  Created on: 12.12.2012
 *      Author: thomas
 */

#ifndef CHANNELCONFIG_H_
#define CHANNELCONFIG_H_

#include <stdbool.h>

typedef enum function_t {
	FUNCTION_NONE = 0,
#ifdef CONFIG_INPUT
	FUNCTION_INPUT = 1,
#endif
#ifdef CONFIG_OUTPUT
	FUNCTION_OUTPUT = 2,
#endif
#ifdef CONFIG_RAFFSTORE
	FUNCTION_RAFFSTORE = 3,
#endif
#ifdef CONFIG_SSR
	FUNCTION_SSR = 4,
#endif
#ifdef CONFIG_ELTAKO
	FUNCTION_DIMMER = 5,
	FUNCTION_FTK = 6,
#endif
#ifdef CONFIG_TEMP
	FUNCTION_TEMPSENS = 7,
#endif
#ifdef CONFIG_LED
	FUNCTION_LED = 8,
#endif
#ifdef CONFIG_BUZZER
	FUNCTION_BUZZER = 9,
#endif
#ifdef CONFIG_IR
	FUNCTION_IRTX = 10,
	FUNCTION_IRRX = 11,
#endif
#ifdef CONFIG_MOTION
	FUNCTION_MOTION = 12,
#endif
#ifdef CONFIG_ANALOG
	FUNCTION_HUMIDITY = 14,
	FUNCTION_LUMINOSITY = 15,
#endif
#ifdef CONFIG_KEYPAD
	FUNCTION_KEYPAD = 16,
#endif
#ifdef CONFIG_KWB
	FUNCTION_KWB_INPUT = 17,
	FUNCTION_KWB_TEMP = 18,
	FUNCTION_KWB_HK = 19,
#endif
#ifdef CONFIG_HOMECAN_GATEWAY
	FUNCTION_BUSLOAD = 20,
#endif
	FUNCTION_RESERVED = 255
} function_t;

typedef enum circuit_t {
	CIRCUIT_NONE = 0,
	CIRCUIT_INPUT = 1,
	CIRCUIT_OCIN = 2,
	CIRCUIT_OUT = 3,
	CIRCUIT_LEDOUT = 4,
	CIRCUIT_RS485TX = 5,
	CIRCUIT_RS485RX = 6,
	CIRCUIT_I2C = 7,
	CIRCUIT_ANALOG = 8,
	CIRCUIT_1WIRE = 9,
	CIRCUIT_PWM = 10,
	CIRCUIT_KEYPAD = 11,
	CIRCUIT_IRRX = 12,
	CIRCUIT_RS485KWB = 13,
	CIRCUIT_RESERVED = 255
} circuit_t;

#ifdef CONFIG_LED
typedef enum ledstate_t {
	LED_MODE_OFF = 0,
	LED_MODE_ON = 1,
	LED_MODE_SLOW = 2,
	LED_MODE_FAST = 3
} ledmode_t;

typedef struct
{
	ledmode_t mode;
	uint8_t time;
} ledstate_t;
#endif

#ifdef CONFIG_RAFFSTORE
typedef enum raffmode_t {
	RAFFSTORE_IDLE = 0,
	RAFFSTORE_UP = 1,
	RAFFSTORE_DOWN = 2,
	RAFFSTORE_MOVE = 3
} raffmode_t;

#define CHANNELCONFIG_RAFFSTORE_ANGLE_MAX		12750	//0.5s*100*255
#define CHANNELCONFIG_RAFFSTORE_POSITION_MAX	510000	//20s*100*255

typedef struct
{
	raffmode_t mode;
	uint8_t wait;

	uint16_t angle;			//0(offen)-CHANNELCONFIG_RAFFSTORE_ANGLE_MAX(zu)
	uint16_t angleTarget;	//0(offen)-CHANNELCONFIG_RAFFSTORE_ANGLE_MAX(zu)
	uint8_t angleClose;		//delta in 10ms
	uint8_t angleOpen;		//delta in 10ms

	uint32_t position;			//0(offen)-CHANNELCONFIG_RAFFSTORE_POSITION_MAX(zu)
	uint32_t positionTarget;	//0(offen)-CHANNELCONFIG_RAFFSTORE_POSITION_MAX(zu)
	uint8_t positionUp;			//delta position in 10ms
	uint8_t positionDown;		//delta position in 10ms
} raffstate_t;
#endif

#ifdef CONFIG_BUZZER
typedef struct
{
	uint16_t freq;
} buzzerstate_t;
#endif

#ifdef CONFIG_TEMP
typedef struct
{
	float value;
	uint8_t intervall;
	uint8_t counter;
} temperature_t;
#endif

#ifdef CONFIG_ELTAKO
typedef struct
{
	uint8_t value;
} dimmerstate_t;

typedef struct
{
	union {
		uint32_t id;
		uint8_t id_buffer[4];
	};
	uint8_t sniffing;
	uint8_t state;
} ftk_t;
#endif

#ifdef CONFIG_ANALOG
typedef struct
{
	float value;
	uint32_t avg;
	uint8_t intervall;
	uint8_t counter;
} analog_t;

typedef struct
{
	uint16_t accumulator;
	uint8_t value;
	uint8_t intervall;
	uint8_t counter;
	uint8_t scale;	//0.8bit fix comma
	int8_t offset;
} humidity_t;
#endif

#ifdef CONFIG_KWB
typedef struct
{
	uint8_t value;
	uint8_t channel;
} kwbstate_t;

typedef struct
{
	float value;
	uint8_t channel;
	uint8_t intervall;
	uint8_t counter;
} kwbtemp_t;

#ifdef CONFIG_POTIO
typedef struct
{
	uint8_t hk;
	uint8_t mode;
	uint16_t position;
} kwbhk_t;
#endif
#endif

#ifdef CONFIG_HOMECAN_GATEWAY
typedef struct
{
	uint32_t byteCount;
	uint8_t intervall;
	uint8_t counter;
} busload_t;
#endif

typedef struct
{
	function_t function;
	uint8_t port[2];
	union  {
		uint8_t state;
#ifdef CONFIG_LED
		ledstate_t ledstate;
#endif
#ifdef CONFIG_RAFFSTORE
		raffstate_t raffstate;
#endif
#ifdef CONFIG_BUZZER
		buzzerstate_t buzzerstate;
#endif
#ifdef CONFIG_ELTAKO
		dimmerstate_t dimmerstate;
		ftk_t ftkstate;
#endif
#ifdef CONFIG_TEMP
		temperature_t tempstate;
#endif
#ifdef CONFIG_ANALOG
		analog_t analogstate;
		humidity_t humiditystate;
#endif
#ifdef CONFIG_KWB
		kwbstate_t kwbstate;
		kwbtemp_t kwbtemp;
#ifdef CONFIG_POTIO
		kwbhk_t kwbhk;
#endif
#endif
#ifdef CONFIG_HOMECAN_GATEWAY
		busload_t busloadstate;
#endif
	};
	uint8_t changed;
} channelconfig_t;

void channelconfig_init(void);
//iterate all channel, do for each according to configuration
void channelconfig_task(void);
bool channelconfig_configure(uint8_t channel, const channelconfig_t *config);
uint8_t channelconfig_getChannel(uint8_t port);

//to be implemented by app
extern void channelconfig_init_device(void);
extern circuit_t channelconfig_getPortType(uint8_t port);
extern uint8_t channelconfig_getMaxPort(void);
extern void channelconfig_setPort(uint8_t port, uint8_t state);
extern uint8_t channelconfig_getPort(uint8_t port);
extern void channelconfig_setStatusLED(uint8_t led, uint8_t state);
extern void channelconfig_10msUserISR(void);
extern void channelconfig_100msUserTask(void);
extern void channelconfig_1sUserTask(void);

#ifdef CONFIG_KEYPAD
extern void channelconfig_keyTask(void);
#endif
#ifdef CONFIG_BUZZER
extern void channelconfig_setBuzzer(uint16_t freq);
#endif

#endif /* CHANNELCONFIG_H_ */
