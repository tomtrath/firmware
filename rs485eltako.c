/*
 * rs485Eltako.c
 *
 * Created: 10.11.2012 17:04:51
 *  Author: thomas
 */ 

#include <avr/io.h>
#include "uart2.h"
#include "rs485eltako.h"

rs485eltako_t msgrx;

typedef void (*voidFuncPtru08)(const rs485eltako_t *msg);
volatile static voidFuncPtru08 appRxFuncHandler;

#define MODE_WAITING_FOR_PREAMBLE 0
#define MODE_RECEIVED_SYNC_BYTE1 1
#define MODE_IN_FRAME 2

uint8_t mode;
uint8_t byteIdx;
uint8_t frameLen;
uint8_t checksum;

void rs485eltako_init(void) {
	uart1Init();
	uartSetBaudRate(1,RS485ELTAKO_BAUDRATE);
	mode = MODE_WAITING_FOR_PREAMBLE;
}

uint8_t rs485eltako_transmitMessage(const rs485eltako_t *msg) {
	uint8_t i;
	uint8_t checksum;
	char *msgbuf;

	if (uartTransmitPending(1)) return 0;
	
	uartAddToTxBuffer(1,RS485ELTAKO_SYNCBYTE1);
	uartAddToTxBuffer(1,RS485ELTAKO_SYNCBYTE2);
	checksum = 0;
	uartAddToTxBuffer(1,RS485ELTAKO_LENGTH);
	checksum+=RS485ELTAKO_LENGTH;	
	uartAddToTxBuffer(1,msg->org);
	uartAddToTxBuffer(1,msg->databuf[3]);
	uartAddToTxBuffer(1,msg->databuf[2]);
	uartAddToTxBuffer(1,msg->databuf[1]);
	uartAddToTxBuffer(1,msg->databuf[0]);
	uartAddToTxBuffer(1,msg->idbuf[3]);
	uartAddToTxBuffer(1,msg->idbuf[2]);
	uartAddToTxBuffer(1,msg->idbuf[1]);
	uartAddToTxBuffer(1,msg->idbuf[0]);	
	msgbuf =(char*)msg;
	for (i=0;i<sizeof(rs485eltako_t);i++) {		
		checksum+=msgbuf[i];
	}
	uartAddToTxBuffer(1,RS485ELTAKO_STATUS);
	checksum+=RS485ELTAKO_STATUS;
	uartAddToTxBuffer(1,checksum);
	uartSendTxBuffer(1);
	return 1;
}

void rs485eltako_setRxHandler(void (*rx_func)(const rs485eltako_t *msg)) {
	appRxFuncHandler = rx_func;
}

void rs485eltakoReceiveTask(void) {
	uint8_t data;	
	//Check if new data available and get it
	if (uartReceiveByte(1,&data)) {		
		switch (mode) {
			case MODE_WAITING_FOR_PREAMBLE:
				if (data==RS485ELTAKO_SYNCBYTE1) {
					mode = MODE_RECEIVED_SYNC_BYTE1;
				} 			
				break;	
			case MODE_RECEIVED_SYNC_BYTE1:
				if (data==RS485ELTAKO_SYNCBYTE2) {
					mode = MODE_IN_FRAME;
					byteIdx = 0;						
				} else {
					//preamble wrong change back to searching for full preamble
					mode = MODE_WAITING_FOR_PREAMBLE;
				}						
				break;
			case MODE_IN_FRAME:				
				if (byteIdx==0) { 
					frameLen = data&0x1F; //mask out H_SEQ, just keep LENGTH
					checksum = data;					
				} else {				
					if (byteIdx==1) msgrx.org = data;
					if (byteIdx>=2 && byteIdx<=5) msgrx.databuf[5-byteIdx] = data;
					if (byteIdx>=6 && byteIdx<=9) msgrx.idbuf[9-byteIdx] = data;
					
					//calculate checksum	
					if (byteIdx<frameLen) {
						checksum+=data;
					} else {
						mode = MODE_WAITING_FOR_PREAMBLE;							
						if (checksum==data) appRxFuncHandler(&msgrx);
					}
					
				}	
				byteIdx++;				
				break;			
		}								
	}
}

uint32_t rs485eltako_createDimmerValue(uint8_t value) {
	uint8_t val = ((uint16_t)value)*100/255;
	if (val==0) return RS485ELTAKO_DIMMER_OFF;
	return ((uint32_t) RS485ELTAKO_DIMMER_0 | ((uint32_t)val)<<16);
}	
