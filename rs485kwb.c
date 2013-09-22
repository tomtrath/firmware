/*
 * rs485Eltako.c
 *
 * Created: 10.11.2012 17:04:51
 *  Author: thomas
 */ 

#include <avr/io.h>
#include "uart2.h"
#include "rs485kwb.h"

rs485kwb_t msgrx;

typedef void (*voidFuncPtru08)(const rs485kwb_t *msg);
volatile static voidFuncPtru08 appRxFuncHandler;

#define MODE_WAITING 0
#define MODE_IN_CTRLFRAME 1
#define MODE_IN_SENSEFRAME 2

uint8_t mode;
uint8_t byteIdx;
uint8_t frameLen;
uint8_t checksum;
uint8_t preamblebuf[5];
uint8_t preambleidx;
uint8_t tempIdx;

uint8_t pos;
#define SCALE		1
#define SCALE_EXT	2
#define VALUE		3
#define VALUE_EXT	4


void rs485kwb_init(void) {
	uart1Init();
	uartSetBaudRate(1,RS485KWB_BAUDRATE);
	mode = MODE_WAITING;
}

void rs485kwb_setRxHandler(void (*rx_func)(const rs485kwb_t *msg)) {
	appRxFuncHandler = rx_func;
}

void rs485kwbReceiveTask(void) {
	uint8_t data;	
	//Check if new data available and get it
	if (uartReceiveByte(1,&data)) {
		switch (mode) {
		case MODE_IN_CTRLFRAME:
			msgrx.ctrl.raw[byteIdx] = data;
			switch (byteIdx) {
			case 2:
				msgrx.ctrl.data.boiler0pump = (data>>5)&0x01;
				msgrx.ctrl.data.hk1pump = (data>>7)&0x01;
				msgrx.ctrl.data.hk2pump = (data>>6)&0x01;
				break;
			case 3:
				msgrx.ctrl.data.ascheaustragung = data&0x01;
				msgrx.ctrl.data.reinigung = (data>>1)&0x01;
				msgrx.ctrl.data.hk2mischer = (data>>4)&0x03;
				msgrx.ctrl.data.hk1mischer = (data>>6)&0x03;
				break;
			case 4:
				msgrx.ctrl.data.mainrelais = (data>>4)&0x01;
				msgrx.ctrl.data.raumaustragung = (data>>6)&0x01;
				break;
			default:
				break;
			}
			//calculate checksum
			if (byteIdx<frameLen) {
				checksum+=data;
			} else {
				mode = MODE_WAITING;
				//if (checksum==data) appRxFuncHandler(&msgrx);
				appRxFuncHandler(&msgrx);
			}
			byteIdx++;
			break;
		case MODE_IN_SENSEFRAME:
			msgrx.sense.raw[byteIdx] = data;
			if (byteIdx>5 && tempIdx<18) {
				if (pos==SCALE) {
					if (data==2) {
						pos = SCALE_EXT;
					} else {
						pos = VALUE;
					}
					msgrx.sense.temp[tempIdx] = ((int8_t)data)*25.5;
				} else if (pos==VALUE) {
					if (data==2) {
						pos = VALUE_EXT;
					} else  {
						pos = SCALE;
					}
					//can value be 2?? if yes need something like VALUE_EXT because of additional 0
					msgrx.sense.temp[tempIdx] += data/10.0;
					tempIdx++;
				} else if (pos==SCALE_EXT) {
					//skip this 0
					frameLen++;
					pos = VALUE;
				} else if (pos==VALUE_EXT) {
					//skip this 0
					frameLen++;
					pos = SCALE;
				}
			}
			//calculate checksum
			if (byteIdx<frameLen) {
				checksum+=data;
			} else {
				mode = MODE_WAITING;
				//if (checksum==data) appRxFuncHandler(&msgrx);
				appRxFuncHandler(&msgrx);
			}
			byteIdx++;
			break;
		}
		if (preambleidx==255) {
			if (data==RS485KWB_SYNCBYTE) {
				preambleidx = 0;
				preamblebuf[preambleidx] = data;
			}
		} else if (preambleidx<4){
			preambleidx++;
			preamblebuf[preambleidx] = data;
			if (preambleidx==1 && preamblebuf[1]==RS485KWB_FILLBYTE) {
				//normal data
				preambleidx = 255;
			} else if (preambleidx==2 && preamblebuf[1]==RS485KWB_PRECTRL1 && preamblebuf[2]==RS485KWB_PRECTRL2) {
				mode = MODE_IN_CTRLFRAME;
				preambleidx = 255;
				byteIdx = 0;
				checksum = 0;
				frameLen = 11;
				msgrx.msgtype = RS485KWB_CTRLMSG;
			} else 	if (preambleidx==3 && preamblebuf[1]==RS485KWB_PRESENSE1 && preamblebuf[2]==RS485KWB_PRESENSE2 && preamblebuf[3]==RS485KWB_PRESENSE3) {
				mode = MODE_IN_SENSEFRAME;
				preambleidx = 255;
				byteIdx = 0;
				checksum = 0;
				frameLen = 5+18*2+4;
				tempIdx = 0;
				pos = SCALE;
				msgrx.msgtype = RS485KWB_SENSEMSG;
			}
		} else {
			preambleidx = 255;
		}
	}
}
