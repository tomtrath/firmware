/*
 * rs485kwb.h
 *
 * Created: 10.11.2012 17:09:04
 *  Author: thomas
 */ 


#ifndef RS485KWB_H_
#define RS485KWB_H_

typedef struct 
{
	uint8_t raw[64];
	float temp[18];
} rs485kwbsense_t ;

typedef struct
{
	uint8_t raw[13];
	struct {
		int boiler0pump : 1;
		int hk2pump : 1;
		int hk1pump : 1;
		int ascheaustragung : 1;
		int reinigung : 1;
		int hk2mischer : 2;
		int hk1mischer : 2;
		int mainrelais: 1;
		int raumaustragung: 1;
	} data;
} rs485kwbctrl_t ;

typedef struct
{
	uint8_t msgtype;
	union {
		rs485kwbsense_t sense;
		rs485kwbctrl_t ctrl;
	};	
} rs485kwb_t ;

#define RS485KWB_CTRLMSG		0x01
#define RS485KWB_SENSEMSG		0x02

#define RS485KWB_BAUDRATE		19200

//CtrlMsg 2,16,17,counter, data[11],checksum
//SenseMsg 2,2,51,16,counter, data, checksum
#define RS485KWB_SYNCBYTE		0x02
#define RS485KWB_PRECTRL1		0x10
#define RS485KWB_PRECTRL2		0x11
#define RS485KWB_PRESENSE1		0x02
#define RS485KWB_PRESENSE2		0x33
#define RS485KWB_PRESENSE3		0x10
#define RS485KWB_FILLBYTE		0x00


//functions
extern void rs485kwb_init(void);

//return false if old transmission not yet completed
//extern uint8_t rs485kwb_transmitMessage(const rs485kwb_t *msg);

//called if msg received through previously called rs485kwbReceiveTask
void rs485kwb_setRxHandler(void (*rx_func)(const rs485kwb_t *msg));

//call from app main task
void rs485kwbReceiveTask(void);

#endif /* RS485KWB_H_ */
