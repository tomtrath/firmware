/*
 * networkcan.c
 *
 * Created: 10.11.2012 17:29:29
 *  Author: thomas
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <string.h>

#include "global.h"
#include "channelconfig.h"
#include "homecan.h"

typedef struct
{
	uint8_t port;
	circuit_t circuit;
} portconfig_t;

#define CHANNELCONFIG_MAX_PORT	4

static portconfig_t portconfig[CHANNELCONFIG_MAX_PORT+1] = {
		{ 0, CIRCUIT_OUT},
		{ 1, CIRCUIT_INPUT},
		{ 2, CIRCUIT_RS485TX},
		{ 3, CIRCUIT_RS485RX},
		{ 4, CIRCUIT_I2C}
};

circuit_t channelconfig_getPortType(uint8_t port) {
	if (port>CHANNELCONFIG_MAX_PORT) return CIRCUIT_NONE;
	return portconfig[port].circuit;
}

uint8_t channelconfig_getMaxPort() {
	return CHANNELCONFIG_MAX_PORT;
}

void channelconfig_setPort(uint8_t port, uint8_t state) {
	if (state == 0) {
		switch (port) {
			case 0: PORTF &= ~(1<<PF3); break;
			case 1: PORTA &= ~(1<<PA3); break;
		}
	} else {
		switch (port) {
			case 0: PORTF |= (1<<PF3); break;
			case 1: PORTA |= (1<<PA3); break;
		}
	}
}


uint8_t channelconfig_getPort(uint8_t port) {
	switch (port) {
		case 0: return (PINF>>PINF3)&0x01; break;
		case 1: return (~(PINA>>PINA3))&0x01; break;
	}
	return 0;
}

void channelconfig_init_device(void) {
    //init CAN port LED
	PORTE |= 1<<PE2;
	DDRE |= 1<<PE2;
	//init RS485 to RX
	DDRD |= (1<<PD4);
	PORTD &= ~(1<<PD4);
}

void channelconfig_setStatusLED(uint8_t led,uint8_t state) {
	if (led==0) {
		if (!state) {
			PORTE |= (1<<PE2);
		} else {
			PORTE &= ~(1<<PE2);
		}
	}
}

void channelconfig_10msUserISR() {}
void channelconfig_100msUserTask() {}
void channelconfig_1sUserTask() {}

int main(void)
{
	channelconfig_init();

	sei();

    while(1)
    {
    	channelconfig_task();
    }
}

