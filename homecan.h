#ifndef HOMECAN_H
#define HOMECAN_H

#include <stdbool.h>

#define EEPROM_DEVICE_ID		0x00

#define HOMECAN_UDP_PORT							15000
#define HOMECAN_UDP_PORT_BOOTLOADER					15001

#define HOMECAN_ADDRESS_FROM_EEPROM	0

typedef enum homecan_msgtype_t {
	HOMECAN_MSGTYPE_ONOFF				= 0x00,
	HOMECAN_MSGTYPE_OPENCLOSED			= 0x01,
#ifdef CONFIG_MOTION
	HOMECAN_MSGTYPE_MOTION				= 0x02,
#endif
#ifdef CONFIG_ELTAKO
	HOMECAN_MSGTYPE_DIMMER				= 0x03,
#endif
#ifdef CONFIG_KWB
	HOMECAN_MSGTYPE_KWB_HK				= 0x04,
#endif
#ifdef CONFIG_RAFFSTORE
	HOMECAN_MSGTYPE_POSITION			= 0x05,
	HOMECAN_MSGTYPE_SHADE				= 0x06,
	HOMECAN_MSGTYPE_STOPMOVE			= 0x07,
	HOMECAN_MSGTYPE_UPDOWN				= 0x08,
#endif
	HOMECAN_MSGTYPE_TEMPERATURE			= 0x09,
	HOMECAN_MSGTYPE_HUMIDITY			= 0x0A,
	HOMECAN_MSGTYPE_LUMINOSITY			= 0x0B,
	HOMECAN_MSGTYPE_UV_INDEX			= 0x0C,
	HOMECAN_MSGTYPE_AIR_PRESSURE		= 0x0D,
	HOMECAN_MSGTYPE_WIND_SPEED			= 0x0E,
	HOMECAN_MSGTYPE_WIND_DIRECTION		= 0x0F,
	HOMECAN_MSGTYPE_RAIN				= 0x10,
#ifdef CONFIG_ELTAKO
	HOMECAN_MSGTYPE_INCDEC				= 0x11,
	HOMECAN_MSGTYPE_FTKID				= 0x12,
#endif

	HOMECAN_MSGTYPE_FLOAT				= 0x20,
	HOMECAN_MSGTYPE_UINT32				= 0x21,

#ifdef CONFIG_KEYPAD
	HOMECAN_MSGTYPE_KEY_SEQUENCE		= 0x80,
#endif
	HOMECAN_MSGTYPE_STRING				= 0x81,
#ifdef CONFIG_IR
	HOMECAN_MSGTYPE_IR					= 0x83,
#endif
#ifdef CONFIG_BUZZER
	HOMECAN_MSGTYPE_BUZZER				= 0x85,
#endif

	HOMECAN_MSGTYPE_CHANNEL_CONFIG		= 0xE0,
	HOMECAN_MSGTYPE_GET_CONFIG			= 0xE1,
	HOMECAN_MSGTYPE_CLEAR_CONFIG		= 0xE2,
	HOMECAN_MSGTYPE_STORE_CONFIG 		= 0xE3,
#ifdef CONFIG_ELTAKO
	HOMECAN_MSGTYPE_DIMMER_LEARN		= 0xE4,
#endif
	HOMECAN_MSGTYPE_REQUEST_STATE		= 0xE5,

	HOMECAN_MSGTYPE_BOOTLOADER			= 0xF0,
	HOMECAN_MSGTYPE_CALL_BOOTLOADER		= 0xF1,

	HOMECAN_MSGTYPE_HEARTBEAT			= 0xFF
} homecan_msgtype_t;


#define HOMECAN_HEADER_MODE_SRC		0
#define HOMECAN_HEADER_MODE_DST		1
#define HOMECAN_HEADER_PRIO_DEFAULT	0x7

typedef struct
{
	struct {
		int priority : 4;
		int mode : 1;
	} header;
	homecan_msgtype_t msgtype;
	uint8_t address;
	uint8_t channel;

	uint8_t length;
	uint8_t data[8];
} homecan_t;


//functions
void homecan_init(uint8_t address);
bool homecan_transmit(const homecan_t *msg);
bool homecan_receive(homecan_t *msg);
void homecan_transmitHeartbeat(void);
uint8_t homecan_getDeviceID(void);

#ifdef CONFIG_HOMECAN_GATEWAY
uint32_t homecan_getTxByteCount(void);
#endif

#endif
