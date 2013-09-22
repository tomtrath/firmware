/*
 * MotionCAN.c
 *
 * Created: 29.09.2012 16:27:41
 *  Author: thomas
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

#include "global.h"
#include "homecan.h"
#include "channelconfig.h"


void channelconfig_init_device(void) {
	DDRF = 0x00;
}

circuit_t channelconfig_getPortType(uint8_t port) {
	if (port==0) return CIRCUIT_INPUT;
	return CIRCUIT_NONE;
}

uint8_t channelconfig_getMaxPort() {
	return 0;
}

void channelconfig_setPort(uint8_t port, uint8_t state) {}

uint8_t channelconfig_getPort(uint8_t port) {
	return !((PINF>>PINF0)&0x01);
}

void channelconfig_setStatusLED(uint8_t led, uint8_t state) {}
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
