/*
 * ControlCAN.c
 *
 * Created: 10.11.2012 17:29:29
 *  Author: thomas
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "global.h"
#include "channelconfig.h"
#include "homecan.h"

typedef struct
{
	uint8_t port;
	circuit_t circuit;
} portconfig_t;

#define CHANNELCONFIG_MAX_PORT	46
static portconfig_t portconfig[CHANNELCONFIG_MAX_PORT+1] = {
		{ 0, CIRCUIT_INPUT},
		{ 1, CIRCUIT_INPUT},
		{ 2, CIRCUIT_INPUT},
		{ 3, CIRCUIT_INPUT},
		{ 4, CIRCUIT_INPUT},
		{ 5, CIRCUIT_INPUT},
		{ 6, CIRCUIT_INPUT},
		{ 7, CIRCUIT_INPUT},
		{ 8, CIRCUIT_INPUT},
		{ 9, CIRCUIT_INPUT},
		{10, CIRCUIT_INPUT},
		{11, CIRCUIT_INPUT},
		{12, CIRCUIT_INPUT},
		{13, CIRCUIT_INPUT},
		{14, CIRCUIT_INPUT},
		{15, CIRCUIT_INPUT},
		{16, CIRCUIT_INPUT},
		{17, CIRCUIT_INPUT},
		{18, CIRCUIT_INPUT},
		{19, CIRCUIT_INPUT},
		{20, CIRCUIT_INPUT},
		{21, CIRCUIT_OCIN},
		{22, CIRCUIT_OUT},
		{23, CIRCUIT_OUT},
		{24, CIRCUIT_OUT},
		{25, CIRCUIT_OUT},
		{26, CIRCUIT_OUT},
		{27, CIRCUIT_OUT},
		{28, CIRCUIT_OUT},
		{29, CIRCUIT_OUT},
		{30, CIRCUIT_OUT},
		{31, CIRCUIT_OUT},
		{32, CIRCUIT_OUT},
		{33, CIRCUIT_OUT},
		{34, CIRCUIT_LEDOUT},
		{35, CIRCUIT_LEDOUT},
		{36, CIRCUIT_LEDOUT},
		{37, CIRCUIT_LEDOUT},
		{38, CIRCUIT_LEDOUT},
		{39, CIRCUIT_LEDOUT},
		{40, CIRCUIT_LEDOUT},
		{41, CIRCUIT_LEDOUT},
		{42, CIRCUIT_LEDOUT},
		{43, CIRCUIT_LEDOUT},
		{44, CIRCUIT_RS485TX},
		{45, CIRCUIT_RS485RX},
		{46, CIRCUIT_I2C}
};

uint8_t channelconfig_getMaxPort() {
	return CHANNELCONFIG_MAX_PORT;
}

circuit_t channelconfig_getPortType(uint8_t port) {
	if (port>CHANNELCONFIG_MAX_PORT) return CIRCUIT_NONE;
	return portconfig[port].circuit;
}

void channelconfig_setPort(uint8_t port, uint8_t state) {
	if (state == 0) {
		switch (port) {
			case 0: PORTA &= ~(1<<PA4); break;
			case 1: PORTA &= ~(1<<PA5); break;
			case 2: PORTA &= ~(1<<PA7); break;
			case 3: PORTA &= ~(1<<PA6); break;
			case 4: PORTC &= ~(1<<PC7); break;
			case 5: PORTG &= ~(1<<PG2); break;
			case 6: PORTC &= ~(1<<PC5); break;
			case 7: PORTC &= ~(1<<PC6); break;
			case 8: PORTF &= ~(1<<PF0); break;
			case 9: PORTF &= ~(1<<PF1); break;
			case 10: PORTF &= ~(1<<PF2); break;
			case 11: PORTF &= ~(1<<PF3); break;
			case 12: PORTF &= ~(1<<PF4); break;
			case 13: PORTF &= ~(1<<PF5); break;
			case 14: PORTF &= ~(1<<PF6); break;
			case 15: PORTF &= ~(1<<PF7); break;
			case 16: PORTG &= ~(1<<PG0); break;
			case 17: PORTD &= ~(1<<PD7); break;
			case 18: PORTD &= ~(1<<PD4); break;
			case 19: PORTG &= ~(1<<PG4); break;
			case 20: PORTG &= ~(1<<PG3); break;
			case 21: PORTB &= ~(1<<PB7); break;
			case 22: PORTE &= ~(1<<PE2); break;
			case 23: PORTE &= ~(1<<PE3); break;
			case 24: PORTE &= ~(1<<PE4); break;
			case 25: PORTE &= ~(1<<PE5); break;
			case 26: PORTE &= ~(1<<PE6); break;
			case 27: PORTE &= ~(1<<PE7); break;
			case 28: PORTB &= ~(1<<PB0); break;
			case 29: PORTB &= ~(1<<PB2); break;
			case 30: PORTB &= ~(1<<PB3); break;
			case 31: PORTB &= ~(1<<PB4); break;
			case 32: PORTB &= ~(1<<PB5); break;
			case 33: PORTB &= ~(1<<PB6); break;
			case 34: PORTA &= ~(1<<PA3); break;
			case 35: PORTA &= ~(1<<PA2); break;
			case 36: PORTA &= ~(1<<PA1); break;
			case 37: PORTA &= ~(1<<PA0); break;
			case 38: PORTC &= ~(1<<PC1); break;
			case 39: PORTC &= ~(1<<PC2); break;
			case 40: PORTG &= ~(1<<PG1); break;
			case 41: PORTC &= ~(1<<PC0); break;
//			case 42: PORTC &= ~(1<<PC3); break;
//			case 43: PORTC &= ~(1<<PC4); break;
		}
	} else {
		switch (port) {
			case 0: PORTA |= (1<<PA4); break;
			case 1: PORTA |= (1<<PA5); break;
			case 2: PORTA |= (1<<PA7); break;
			case 3: PORTA |= (1<<PA6); break;
			case 4: PORTC |= (1<<PC7); break;
			case 5: PORTG |= (1<<PG2); break;
			case 6: PORTC |= (1<<PC5); break;
			case 7: PORTC |= (1<<PC6); break;
			case 8: PORTF |= (1<<PF0); break;
			case 9: PORTF |= (1<<PF1); break;
			case 10: PORTF |= (1<<PF2); break;
			case 11: PORTF |= (1<<PF3); break;
			case 12: PORTF |= (1<<PF4); break;
			case 13: PORTF |= (1<<PF5); break;
			case 14: PORTF |= (1<<PF6); break;
			case 15: PORTF |= (1<<PF7); break;
			case 16: PORTG |= (1<<PG0); break;
			case 17: PORTD |= (1<<PD7); break;
			case 18: PORTD |= (1<<PD4); break;
			case 19: PORTG |= (1<<PG4); break;
			case 20: PORTG |= (1<<PG3); break;
			case 21: PORTB |= (1<<PB7); break;
			case 22: PORTE |= (1<<PE2); break;
			case 23: PORTE |= (1<<PE3); break;
			case 24: PORTE |= (1<<PE4); break;
			case 25: PORTE |= (1<<PE5); break;
			case 26: PORTE |= (1<<PE6); break;
			case 27: PORTE |= (1<<PE7); break;
			case 28: PORTB |= (1<<PB0); break;
			case 29: PORTB |= (1<<PB2); break;
			case 30: PORTB |= (1<<PB3); break;
			case 31: PORTB |= (1<<PB4); break;
			case 32: PORTB |= (1<<PB5); break;
			case 33: PORTB |= (1<<PB6); break;
			case 34: PORTA |= (1<<PA3); break;
			case 35: PORTA |= (1<<PA2); break;
			case 36: PORTA |= (1<<PA1); break;
			case 37: PORTA |= (1<<PA0); break;
			case 38: PORTC |= (1<<PC1); break;
			case 39: PORTC |= (1<<PC2); break;
			case 40: PORTG |= (1<<PG1); break;
			case 41: PORTC |= (1<<PC0); break;
//			case 42: PORTC |= (1<<PC3); break;
//			case 43: PORTC |= (1<<PC4); break;
		}
	}
}


uint8_t channelconfig_getPort(uint8_t port) {
	switch (port) {
		case 0: return (PINA>>PINA4)&0x01; break;
		case 1: return (PINA>>PINA5)&0x01; break;
		case 2: return (PINA>>PINA7)&0x01; break;
		case 3: return (PINA>>PINA6)&0x01; break;
		case 4: return (PINC>>PINC7)&0x01; break;
		case 5: return (PING>>PING2)&0x01; break;
		case 6: return (PINC>>PINC5)&0x01; break;
		case 7: return (PINC>>PINC6)&0x01; break;
		case 8: return (PINF>>PINF0)&0x01; break;
		case 9: return (PINF>>PINF1)&0x01; break;
		case 10: return (PINF>>PINF2)&0x01; break;
		case 11: return (PINF>>PINF3)&0x01; break;
		case 12: return (PINF>>PINF4)&0x01; break;
		case 13: return (PINF>>PINF5)&0x01; break;
		case 14: return (PINF>>PINF6)&0x01; break;
		case 15: return (PINF>>PINF7)&0x01; break;
		case 16: return (PING>>PING0)&0x01; break;
		case 17: return (PIND>>PIND7)&0x01; break;
		case 18: return (PIND>>PIND4)&0x01; break;
		case 19: return (PING>>PING4)&0x01; break;
		case 20: return (PING>>PING3)&0x01; break;
		case 21: return (PINB>>PINB7)&0x01; break;
		case 22: return (PINE>>PINE2)&0x01; break;
		case 23: return (PINE>>PINE3)&0x01; break;
		case 24: return (PINE>>PINE4)&0x01; break;
		case 25: return (PINE>>PINE5)&0x01; break;
		case 26: return (PINE>>PINE6)&0x01; break;
		case 27: return (PINE>>PINE7)&0x01; break;
		case 28: return (PINB>>PINB0)&0x01; break;
		case 29: return (PINB>>PINB2)&0x01; break;
		case 30: return (PINB>>PINB3)&0x01; break;
		case 31: return (PINB>>PINB4)&0x01; break;
		case 32: return (PINB>>PINB5)&0x01; break;
		case 33: return (PINB>>PINB6)&0x01; break;
		case 34: return (PINA>>PINA3)&0x01; break;
		case 35: return (PINA>>PINA2)&0x01; break;
		case 36: return (PINA>>PINA1)&0x01; break;
		case 37: return (PINA>>PINA0)&0x01; break;
		case 38: return (PINC>>PINC1)&0x01; break;
		case 39: return (PINC>>PINC2)&0x01; break;
		case 40: return (PING>>PING1)&0x01; break;
		case 41: return (PINC>>PINC0)&0x01; break;
//		case 42: return (PINC>>PINC3)&0x01; break;
//		case 43: return (PINC>>PINC4)&0x01; break;
	}
	return 0;
}

void channelconfig_init_device(void) {
	PORTA = 0;
	DDRA = (1<<DDA0)|(1<<DDA1)|(1<<DDA2)|(1<<DDA3);						//LEDOUT1-4
	PORTB = 0;
	DDRB = (1<<DDB0)|(1<<DDB2)|(1<<DDB3)|(1<<DDB4)|(1<<DDB5)|(1<<DDB6); 	//OUT7-12
	PORTC = 0;
	DDRC = (1<<DDC0)|(1<<DDC1)|(1<<DDC2)|(1<<DDC3)|(1<<DDC4);			//LEDOUT5-6,8-10
	PORTE = 0;
	DDRE = (1<<DDE2)|(1<<DDE3)|(1<<DDE4)|(1<<DDE5)|(1<<DDE6)|(1<<DDE7); 	//OUT1-6
	PORTG = 0;
	DDRG = (1<<DDG1);												//LEDOUT7
	//DDRD = (1<<DDD0)|(1<<DDD1);	//TWI
}

void channelconfig_setStatusLED(uint8_t led,uint8_t state) {
	if (led==0) {
		if (!state) {
			PORTC |= (1<<PC3);
		} else {
			PORTC &= ~(1<<PC3);
		}
	}
	if (led==1) {
		if (!state) {
			PORTC |= (1<<PC4);
		} else {
			PORTC &= ~(1<<PC4);
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
