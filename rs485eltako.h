/*
 * rs485eltako.h
 *
 * Created: 10.11.2012 17:09:04
 *  Author: thomas
 */ 


#ifndef RS485ELTAKO_H_
#define RS485ELTAKO_H_

typedef struct 
{
	uint8_t org;
	union {
		uint32_t data;
		uint8_t databuf[4];
	};	
	union {
		uint32_t id;
		uint8_t idbuf[4];
	};	
} rs485eltako_t ;

#define RS485ELTAKO_BAUDRATE		9600

#define RS485ELTAKO_ORG_RPS			0x05
#define RS485ELTAKO_ORG_1BS			0x06
#define RS485ELTAKO_ORG_4BS			0x07
#define RS485ELTAKO_ORG_HRC			0x08
#define RS485ELTAKO_ORG_6DT			0x0A
#define RS485ELTAKO_ORG_MDA			0x0B

#define RS485ELTAKO_SYNCBYTE1		0xA5
#define RS485ELTAKO_SYNCBYTE2		0x5A
#define RS485ELTAKO_LENGTH			0x0B
#define RS485ELTAKO_STATUS			0x00

#define RS485ELTAKO_DIMMER_OFF		0x0200010E
#define RS485ELTAKO_DIMMER_0		0x0200010F
#define RS485ELTAKO_DIMMER_10		0x0200010F
#define RS485ELTAKO_DIMMER_100		0x0200010F
#define RS485ELTAKO_DIMMER_LEARN	0x02000000

#define RS485ELTAKO_FTK_OPENED		0x08000000
#define RS485ELTAKO_FTK_CLOSED		0x09000000

//functions
extern void rs485eltako_init(void);

//return false if old transmission not yet completed
extern uint8_t rs485eltako_transmitMessage(const rs485eltako_t *msg);

//called if msg received through previously called rs485eltakoReceiveTask
void rs485eltako_setRxHandler(void (*rx_func)(const rs485eltako_t *msg));

//call from app main task
void rs485eltakoReceiveTask(void);

uint32_t rs485eltako_createDimmerValue(uint8_t value);

#endif /* RS485ELTAKO_H_ */
