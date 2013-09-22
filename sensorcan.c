/*
 * MotionCAN.c
 *
 * Created: 29.09.2012 16:27:41
 *  Author: thomas
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

#include "global.h"
#include "channelconfig.h"
#include "homecan.h"

#define SENSORCAN_KEYPAD_PORT 10

typedef struct
{
	uint8_t port;
	circuit_t circuit;
} portconfig_t;

#define CHANNELCONFIG_MAX_PORT	12
static portconfig_t portconfig[CHANNELCONFIG_MAX_PORT+1] = {
		{ 0, CIRCUIT_1WIRE},
		{ 1, CIRCUIT_IRRX},
		{ 2, CIRCUIT_ANALOG},
		{ 3, CIRCUIT_ANALOG},
		{ 4, CIRCUIT_PWM},
		{ 5, CIRCUIT_PWM},
		{ 6, CIRCUIT_LEDOUT},
		{ 7, CIRCUIT_LEDOUT},
		{ 8, CIRCUIT_LEDOUT},
		{ 9, CIRCUIT_LEDOUT},
		{SENSORCAN_KEYPAD_PORT, CIRCUIT_KEYPAD},
		{11, CIRCUIT_LEDOUT},
		{12, CIRCUIT_INPUT}
};

void channelconfig_setPort(uint8_t port, uint8_t state) {
	if (state == 0) {
		switch (port) {
		case 0: break;
		case 1: break;
		case 2: break;
		case 3: break;
		case 4: PORTB &= ~(1<<PB5); break;
		case 5: PORTB &= ~(1<<PB4); break;
		case 6: PORTG |= (1<<PG0); break;
		case 7: PORTG |= (1<<PG1); break;
		case 8: PORTC |= (1<<PC0); break;
		case 9: PORTC |= (1<<PC1); break;
		case 10: break;
		case 11: PORTC |= (1<<PC2); break;
		case 12: break;
		}
	} else {
		switch (port) {
		case 0: break;
		case 1: break;
		case 2: break;
		case 3: break;
		case 4: PORTB |= (1<<PB5); break;
		case 5: PORTB |= (1<<PB4); break;
		case 6: PORTG &= ~(1<<PG0); break;
		case 7: PORTG &= ~(1<<PG1); break;
		case 8: PORTC &= ~(1<<PC0); break;
		case 9: PORTC &= ~(1<<PC1); break;
		case 10: break;
		case 11: PORTC &= ~(1<<PC2); break;
		case 12: break;
		}
	}
}



uint8_t channelconfig_getPort(uint8_t port) {
	switch (port) {
		case 1: return (PINF>>PINF3)&0x01; break;
		case 2: return (PINF>>PINF2)&0x01; break;
		case 3: return (PINB>>PINB6)&0x01; break;
		case 4: return (PINB>>PINB5)&0x01; break;
		case 5: return (PINB>>PINB4)&0x01; break;
		case 6: return (~(PING>>PING0))&0x01; break;
		case 7: return (~(PING>>PING1))&0x01; break;
		case 8: return (~(PINC>>PINC0))&0x01; break;
		case 9: return (~(PINC>>PINC1))&0x01; break;
		case 11: return (~(PINC>>PINC2))&0x01; break;
		case 12: return (~(PINA>>PINA4))&0x01; break;
	}
	return 0;
}


uint8_t channelconfig_getMaxPort() {
	return CHANNELCONFIG_MAX_PORT;
}

circuit_t channelconfig_getPortType(uint8_t port) {
	if (port>CHANNELCONFIG_MAX_PORT) return CIRCUIT_NONE;
	return portconfig[port].circuit;
}


void initLEDs(void) {
	//turn outputs off (inverted logic)
	PORTC |= ((1<<PC0) | (1<<PC1));
	PORTG |= ((1<<PG0) | (1<<PG1));
	//set LED ports to output
	DDRC |= (1<<DDC0) | (1<<DDC1);
	DDRG |= (1<<DDG0) | (1<<DDG1);
}

void initBuzzer(void) {
	DDRB &= ~(1<<DDB5);
	TCCR1A = (0<<COM1A1) | (1<<COM1A0) | (0<<WGM11) | (0<<WGM10);
	TCCR1B = (0<<WGM13) | (1<<WGM12) |  (0<<CS12) | (1<<CS11) | (0<<CS10);
}

void channelconfig_setBuzzer(uint16_t freq) {
/*
The output frequency is computed from the following formula: Fo = Fclk / (2.N.(1+OCR1A)), where N=8 is the clock prescaling factor and Fclk is 16MHz. To output a 400Hz square wave: OCR1A = 2500 � 1 = 2499, where 2500 is the period of the signal in �s.
OCR1A = Fclk / (Fo*2*N) -1
*/
		uint16_t regval = F_CPU/(freq*2*8) - 1;
		OCR1AH = regval>>8;
		OCR1AL = regval&0xff;
		if (freq!=0)
			DDRB |= (1<<DDB5);
		else
			DDRB &= ~(1<<DDB5);
}

#ifdef CONFIG_KEYPAD

#define KEY_COUNT 12
#define KEY_ROW_COUNT 4
#define KEY_ROW_STATE_INACTIVE 0
#define KEY_ROW_STATE_ACTIVE 1

void setKeyRow(uint8_t row,uint8_t state) {
	//if (row>=KEY_ROW_COUNT) return;
	if (state==KEY_ROW_STATE_ACTIVE) {
		switch (row) {
			case 0: 
				PORTA &= ~(1<<PA5);
				DDRA |= (1<<DDA5);
				break;
			case 1: 
				PORTA &= ~(1<<PA6);
				DDRA |= (1<<DDA6);
				break;
			case 2:	
				PORTC &= ~(1<<PC4);
				DDRC |= (1<<DDC4);
				break;
			case 3:	
				PORTC &= ~(1<<PC5);
				DDRC |= (1<<DDC5);
				break;
		}
	} else if (state==KEY_ROW_STATE_INACTIVE) {
		switch (row) {
			case 0:
				DDRA &= ~(1<<DDA5);
				PORTA |= (1<<PA5);			
				break;
			case 1:
				DDRA &= ~(1<<DDA6);
				PORTA |= (1<<PA6);
				break;
			case 2:
				DDRC &= ~(1<<DDC4);
				PORTC |= (1<<PC4);
				break;
			case 3:
				DDRC &= ~(1<<DDC5);
				PORTC |= (1<<PC5);
				break;
		}
	}	
}

void initKeyPad(void) {
	//set pull ups on columns
	DDRA &= ~(1<<DDA7);
	PORTA |= (1<<PA7);
	DDRG &= ~(1<<DDG2);
	PORTG |= (1<<PG2);
	DDRC &= ~(1<<DDC7);
	PORTC |= (1<<PC7);
	setKeyRow(0,KEY_ROW_STATE_INACTIVE);
	setKeyRow(1,KEY_ROW_STATE_INACTIVE);
	setKeyRow(2,KEY_ROW_STATE_INACTIVE);
	setKeyRow(3,KEY_ROW_STATE_INACTIVE);
	//init backlight (off)
	DDRC |= (1<<DDC2) | (1<<DDC3);
	PORTC |= (1<<PC2) | (1<<PC3);
	//init shunt
	DDRA |= (1<<DDA3);
	PORTA |= (1<<PA4);	//PA4 to input+pullup
	PORTA &= ~(1<<PA3);	//PA3 to low
}
	
uint8_t readKeyColumns(void) {
	uint8_t col1,col2,col3;	
	col1 = (~(PINA>>PINA7))&0x1; //col1
	col2 = (~(PING>>PING2))&0x1; //col2
	col3 = (~(PINC>>PINC7))&0x1; //col3		
	return col1 | (col2<<1) | (col3<<2);
}

uint8_t getKeyRow(uint8_t row) {
	switch (row) {
		case 0:
			return (PINA>>PINA5)&0x1;
			break;
		case 1:
			return (PINA>>PINA6)&0x1;
			break;
		case 2:
			return (PINC>>PINC4)&0x1;
			break;
		case 3:
			return (PINC>>PINC5)&0x1;
			break;
	}	
	return 0;
}

uint8_t checkKeyRow(uint8_t row) {
	switch (row) {
		case 0:
			setKeyRow(1,KEY_ROW_STATE_INACTIVE);
			setKeyRow(2,KEY_ROW_STATE_INACTIVE);
			setKeyRow(3,KEY_ROW_STATE_INACTIVE);
			setKeyRow(0,KEY_ROW_STATE_ACTIVE);				
			//check if only one key			
			if ((getKeyRow(1)&getKeyRow(2)&getKeyRow(3))==0) {
				channelconfig_setBuzzer(500);
				return 0;
			}				
			return readKeyColumns();
			break;
		case 1:
			setKeyRow(0,KEY_ROW_STATE_INACTIVE);
			setKeyRow(2,KEY_ROW_STATE_INACTIVE);
			setKeyRow(3,KEY_ROW_STATE_INACTIVE);
			setKeyRow(1,KEY_ROW_STATE_ACTIVE);
			//check if only one key
			if ((getKeyRow(0)&getKeyRow(2)&getKeyRow(3))==0) {
				channelconfig_setBuzzer(500);
				return 0;
			}				
			return readKeyColumns();
			break;
		case 2:
			setKeyRow(0,KEY_ROW_STATE_INACTIVE);
			setKeyRow(1,KEY_ROW_STATE_INACTIVE);
			setKeyRow(3,KEY_ROW_STATE_INACTIVE);
			setKeyRow(2,KEY_ROW_STATE_ACTIVE);
			//check if only one key			
			if ((getKeyRow(0)&getKeyRow(1)&getKeyRow(3))==0) {
				channelconfig_setBuzzer(500);
				return 0;
			}				
			return readKeyColumns();
			break;
		case 3:
			setKeyRow(0,KEY_ROW_STATE_INACTIVE);
			setKeyRow(1,KEY_ROW_STATE_INACTIVE);
			setKeyRow(2,KEY_ROW_STATE_INACTIVE);
			setKeyRow(3,KEY_ROW_STATE_ACTIVE);
			//check if only one key
			if ((getKeyRow(0)&getKeyRow(1)&getKeyRow(2))==0) {
				channelconfig_setBuzzer(500);
				return 0;
			}				
			return readKeyColumns();
			break;					
	}	
	return 0;	
}

#define MAX_SEQUENCE 8
volatile uint8_t sequence[MAX_SEQUENCE];
volatile uint8_t seqIdx = 0;
volatile uint8_t seqFlag = 0;

void appendKey(uint8_t key) {
	if (key==11) {
		//# detected, scrap old sequence send #
		sequence[0] = key;
		seqIdx = 1;
		seqFlag = 1;
	} else if (seqIdx==MAX_SEQUENCE || key==9) {
		//End of Sequence KEY detected or max length reached
		seqFlag = 1;
	} else {
		sequence[seqIdx++] = key;
	}
}

void channelconfig_keyTask(void) {
	uint8_t row;
	static uint16_t old_state = 0;			
	uint16_t state=0;
	uint8_t k;
	uint16_t pushed,released;
		
	channelconfig_setBuzzer(0);
	for (row=0;row<4;row++) {
		state |= checkKeyRow(row)<<(row*3);
	}	
	setKeyRow(0,KEY_ROW_STATE_INACTIVE);
	setKeyRow(1,KEY_ROW_STATE_INACTIVE);
	setKeyRow(2,KEY_ROW_STATE_INACTIVE);
	setKeyRow(3,KEY_ROW_STATE_INACTIVE);
	
	if (state!=old_state) {
		//key event occured
		//get new keys
		pushed = (state^old_state)&state;
		//get released keys
		released = (state^old_state)&old_state;
		for (k=0;k<KEY_COUNT;k++) {
			if ((pushed>>k)&0x1) {
				appendKey(k);
			}
			if ((released>>k)&0x1) {
				//do nothing
			}
		}	
	}
	old_state = state;	
}
#endif //CONFIG_KEYPAD

void channelconfig_init_device(void) {
	initBuzzer();
	initLEDs();
#ifdef CONFIG_KEYPAD
	initKeyPad();
#endif
}

void channelconfig_setStatusLED(uint8_t led,uint8_t state) {}

void channelconfig_10msUserISR() {}

void channelconfig_100msUserTask() {
#ifdef CONFIG_KEYPAD
	uint8_t tmp_sreg;  // temporaerer Speicher fuer das Statusregister
	homecan_t msg;
	tmp_sreg = SREG;   // Statusregister (also auch das I-Flag darin) sichern
	cli();             // Interrupts global deaktivieren
	if (seqFlag) {
		msg.address = homecan_getDeviceID();
		msg.msgtype = HOMECAN_MSGTYPE_KEY_SEQUENCE;
		msg.header.mode = HOMECAN_HEADER_MODE_SRC;
		msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
		msg.channel = channelconfig_getChannel(SENSORCAN_KEYPAD_PORT);
		msg.length = seqIdx;
		memcpy(msg.data,(const void*)&sequence[0],msg.length);
		if (msg.channel!=0) {
			homecan_transmit(&msg);
		}
		seqFlag = 0;
		seqIdx = 0;
	}
	SREG = tmp_sreg;     // Status-Register wieder herstellen
#endif
}

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
