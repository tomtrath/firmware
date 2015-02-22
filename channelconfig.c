/*
 * homecan.c
 *
 * Created: 04.11.2012 16:27:41
 *  Author: thomas
 */ 
 
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

#include "global.h"
#include "homecan.h"
#include "channelconfig.h"

#ifdef CONFIG_ELTAKO
#include "rs485eltako.h"
#endif

#ifdef CONFIG_KWB
#include "rs485kwb.h"
#endif

#ifdef CONFIG_IR
#include "irmp.h"
#include "irsnd.h"
#endif

#ifdef CONFIG_ONEWIRE
#include "onewire.h"
#ifdef CONFIG_TEMP
#include "ds18x20.h"
#endif
#endif

#ifdef CONFIG_I2C
#include "i2cmaster.h"
#ifdef CONFIG_TEMP
#include "tmp75.h"
#endif
#ifdef CONFIG_POTIO
#include "mcp4651.h"
#endif
#endif



#define EEPROM_CHANNELCONFIG_MARKER	0x01
#define EEPROM_CHANNELCONFIG_DATA	0x02

#define MARKER_MAGIC	0x55

#define HEARBEAT_PERIODIC

#define CHANNELCONFIG_MAX_CONFIG	63
channelconfig_t channelconfig[CHANNELCONFIG_MAX_CONFIG+1];

#ifdef CONFIG_SSR
#define CHANNELCONFIG_SSR_DELAY_MS	9
#define CHANNELCONFIG_SSR_MAX_REPEAT 10
#endif

#ifdef CONFIG_RAFFSTORE
#define CHANNELCONFIG_RAFFSTORE_WAIT_DEFAULT 80			//0,8s
#define CHANNELCONFIG_RAFFSTORE_UPDATE_INTERVALL 200 	//2s
static uint8_t update_timer = 0;
#endif

#ifdef CONFIG_ELTAKO
#define CHANNELCONFIG_DIMMER_STEP 5
#endif


#ifdef CONFIG_ONEWIRE
static uint8_t id[OW_ROMCODE_SIZE];
#endif

#ifdef CONFIG_IR
static volatile uint8_t irmp=0;
static volatile uint8_t irsnd=0;
#endif

#ifdef CONFIG_POTIO
#define CHANNELCONFIG_KWB_HK_OFF 	39
#define CHANNELCONFIG_KWB_HK_DAY	77
#define CHANNELCONFIG_KWB_HK_NIGHT	51
#define CHANNELCONFIG_KWB_HK_AUTO	64
#endif

#ifdef CONFIG_HOMECAN_GATEWAY
static uint8_t busloadChannel = 0;
#endif

static volatile uint8_t timer1s = 0;
static volatile uint8_t timer100ms = 0;
static volatile uint8_t counter = 0;


uint8_t channelconfig_getChannel(uint8_t port) {
	uint8_t ch;
	for (ch=0;ch<=CHANNELCONFIG_MAX_CONFIG;ch++) {
		if (channelconfig[ch].port[0]==port) return ch;
	}
	return 0;
}

#ifdef CONFIG_ELTAKO
void rxHandler(const rs485eltako_t *msg) {	
	uint8_t ch;
	for (ch=0;ch<=CHANNELCONFIG_MAX_CONFIG;ch++) {
		if (channelconfig[ch].function==FUNCTION_FTK) {
			if (msg->id==channelconfig[ch].ftkstate.id) {
				if (msg->org==RS485ELTAKO_ORG_1BS) {
					if (msg->data==RS485ELTAKO_FTK_OPENED) {
						channelconfig[ch].ftkstate.state = 0x01;
						channelconfig[ch].changed = 1;
					} else if (msg->data==RS485ELTAKO_FTK_CLOSED) {
						channelconfig[ch].ftkstate.state = 0x00;
						channelconfig[ch].changed = 1;
					}
				} else if (msg->org==RS485ELTAKO_ORG_RPS) {
					if (msg->data==RS485ELTAKO_FPE_OPENED) {
						channelconfig[ch].ftkstate.state = 0x01;
						channelconfig[ch].changed = 1;
					} else if (msg->data==RS485ELTAKO_FPE_CLOSED) {
						channelconfig[ch].ftkstate.state = 0x00;
						channelconfig[ch].changed = 1;
					}
				}
			} else {
				if (channelconfig[ch].ftkstate.sniffing) {
					//sniffing mode
					homecan_t msgHomecan;
					msgHomecan.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
					msgHomecan.header.mode = HOMECAN_HEADER_MODE_SRC;
					msgHomecan.msgtype = HOMECAN_MSGTYPE_FTKID;
					msgHomecan.address = homecan_getDeviceID();
					msgHomecan.channel = ch;
					msgHomecan.length = 4;
					memcpy(&msgHomecan.data[0],&msg->idbuf[0],4);
					while (!homecan_transmit(&msgHomecan)) {
						_delay_ms(1);
					}
				}
			}
		}
	}	
}
#endif

#ifdef CONFIG_KWB
void rxHandlerKWB(const rs485kwb_t *msg) {
	uint8_t ch;
	if (msg->msgtype==RS485KWB_SENSEMSG) {
		for (ch=0;ch<=CHANNELCONFIG_MAX_CONFIG;ch++) {
			if (channelconfig[ch].function==FUNCTION_KWB_TEMP) {
				channelconfig[ch].kwbtemp.value = msg->sense.temp[channelconfig[ch].kwbtemp.channel];
			}
		}
	} else if (msg->msgtype==RS485KWB_CTRLMSG) {
		for (ch=0;ch<=CHANNELCONFIG_MAX_CONFIG;ch++) {
			if (channelconfig[ch].function==FUNCTION_KWB_INPUT) {
				uint8_t newValue = 0xFF;
				switch (channelconfig[ch].kwbstate.channel) {
				case 0:
					newValue = msg->ctrl.data.ascheaustragung;
					break;
				case 1:
					newValue = msg->ctrl.data.boiler0pump;
					break;
				case 2:
					newValue = msg->ctrl.data.hk1mischer;
					break;
				case 3:
					newValue = msg->ctrl.data.hk1pump;
					break;
				case 4:
					newValue = msg->ctrl.data.hk2mischer;
					break;
				case 5:
					newValue = msg->ctrl.data.hk2pump;
					break;
				case 6:
					newValue = msg->ctrl.data.mainrelais;
					break;
				case 7:
					newValue = msg->ctrl.data.raumaustragung;
					break;
				case 8:
					newValue = msg->ctrl.data.reinigung;
					break;
				}
				if (channelconfig[ch].kwbstate.value!=newValue) {
					channelconfig[ch].kwbstate.value = newValue;
					channelconfig[ch].changed = 1;
				}
			}
		}
	}
}
#endif

#ifdef CONFIG_IR
ISR(TIMER1_COMPA_vect)
{
	if (irsnd) {
		if (!irsnd_ISR()) {
			if (irmp) {
				(void) irmp_ISR();
			}
		}
	}
}

void channelconfig_timer1_init (void)
{
	OCR1A   =  (F_CPU / F_INTERRUPTS) - 1;                                  // compare value: 1/15000 of CPU frequency
	TCCR1B  = (1 << WGM12) | (1 << CS10);                                   // switch CTC Mode on, set prescaler to 1
	TIMSK1  = 1 << OCIE1A;                                                  // OCIE1A: Interrupt by timer compare
}
#endif

void init_timer3_10ms(void) {
	//setup timer3 to 10ms IRQ
	OCR3AH = 0x02;
	OCR3AL = 0x71;
	TCCR3A = (0<<COM3A1) | (0<<COM3A0) | (0<<COM3B1) | (0<<COM3B0) |(0<<COM3C1) | (0<<COM3C0) |(0<<WGM31) | (0<<WGM30);
	TCCR3B = (0<<ICNC3) | (0<<ICES3) | (0<<WGM33) | (1<<WGM32) |  (1<<CS32) | (0<<CS31) | (0<<CS30);	//16Mhz/256 -> 62,5kHz
	TIMSK3 |= (1<<OCIE3A);
}

void channelconfig_init(void) {
	uint8_t p,marker,didmask;
	didmask = 0;
	channelconfig_init_device();
	marker = eeprom_read_byte((uint8_t *)EEPROM_CHANNELCONFIG_MARKER);
	if (marker==MARKER_MAGIC) {
		channelconfig_setStatusLED(0,1);
		eeprom_read_block(channelconfig,(uint8_t *)EEPROM_CHANNELCONFIG_DATA,sizeof(channelconfig));
	} else {
		channelconfig_setStatusLED(0,0);
		memset(channelconfig,0,sizeof(channelconfig));
	}
	for (p=0;p<=channelconfig_getMaxPort();p++) {
		if (channelconfig_getPortType(p)==CIRCUIT_ANALOG) {
			didmask |= 1<<p; //disable digital
		}
	}
	DIDR0 = didmask;

#ifdef CONFIG_ELTAKO
	rs485eltako_init();
	rs485eltako_setRxHandler(rxHandler);
#endif

#ifdef CONFIG_KWB
	rs485kwb_init();
	rs485kwb_setRxHandler(rxHandlerKWB);
#endif

#ifdef CONFIG_IR
	irmp_init();                                                            // initialize irmp
	irsnd_init();
	channelconfig_timer1_init();
#endif

#ifdef CONFIG_ONEWIRE
	{
		uint8_t diff;
		ow_reset();
		diff = OW_SEARCH_FIRST;
		DS18X20_find_sensor( &diff, &id[0] );
	}
#endif

#ifdef CONFIG_ANALOG
	ADMUX = (0<<REFS1) | (1<<REFS0) | (1<<ADLAR);	// | (0<<MUX4) | (0<<MUX3) | (0<<MUX2) | (1<<MUX1) | (0<<MUX0); //AVCC as ref, left adjust
	ADCSRA = (1<<ADEN) | (1<<ADSC) | (0<<ADATE) | (0<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //enable, 16MHz/128 = 125kHz, no auto trigger
	ADCSRB = (0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);
#endif

#ifdef CONFIG_I2C
	i2c_init();
#ifdef CONFIG_TEMP
	tmp75_init();
#endif
#ifdef CONFIG_POTIO
	mcp4651_init();
#endif
#endif

	homecan_init(HOMECAN_ADDRESS_FROM_EEPROM);

	init_timer3_10ms();
}

#ifdef CONFIG_ANALOG
ISR(ADC_vect) {

}
#endif

void channelconfig_10msISR(void) {
	uint8_t ch;
#ifdef CONFIG_RAFFSTORE
	update_timer++;
	if (update_timer>CHANNELCONFIG_RAFFSTORE_UPDATE_INTERVALL) {
		update_timer = 0;
	}
#endif
	for (ch=0;ch<=CHANNELCONFIG_MAX_CONFIG;ch++) {
		switch (channelconfig[ch].function) {
#ifdef CONFIG_KEYPAD
		case FUNCTION_KEYPAD:
			channelconfig_keyTask();
		break;
#endif
#ifdef CONFIG_RAFFSTORE
		case FUNCTION_RAFFSTORE:
			if (channelconfig[ch].raffstate.mode!=RAFFSTORE_IDLE) {
				if (abs((int16_t)channelconfig[ch].raffstate.angle-channelconfig[ch].raffstate.angleTarget)<((uint16_t)channelconfig[ch].raffstate.angleOpen/2) && labs((int32_t)channelconfig[ch].raffstate.position-channelconfig[ch].raffstate.positionTarget)<((uint16_t)channelconfig[ch].raffstate.positionUp/2)) {
					//Position & Angle Target reached
					//stop raffstore
					channelconfig_setPort(channelconfig[ch].port[0],0);
					channelconfig_setPort(channelconfig[ch].port[1],0);
					channelconfig[ch].raffstate.mode=RAFFSTORE_IDLE;
					channelconfig[ch].changed = 1;
				} else {
					//send update
					if (update_timer==CHANNELCONFIG_RAFFSTORE_UPDATE_INTERVALL) {
						channelconfig[ch].changed = 1;
					}

					if (channelconfig[ch].raffstate.positionTarget>channelconfig[ch].raffstate.position) {
						//need to go further down
						if (channelconfig[ch].raffstate.mode==RAFFSTORE_UP) {
							channelconfig[ch].raffstate.wait = CHANNELCONFIG_RAFFSTORE_WAIT_DEFAULT;
						}
						channelconfig[ch].raffstate.mode=RAFFSTORE_DOWN;
					} else if (channelconfig[ch].raffstate.positionTarget<channelconfig[ch].raffstate.position) {
						//need to go further up
						if (channelconfig[ch].raffstate.mode==RAFFSTORE_DOWN) {
							channelconfig[ch].raffstate.wait = CHANNELCONFIG_RAFFSTORE_WAIT_DEFAULT;
						}
						channelconfig[ch].raffstate.mode=RAFFSTORE_UP;
					} else {
						if (channelconfig[ch].raffstate.angleTarget>channelconfig[ch].raffstate.angle) {
							//need to go further down
							if (channelconfig[ch].raffstate.mode==RAFFSTORE_UP) {
								channelconfig[ch].raffstate.wait = CHANNELCONFIG_RAFFSTORE_WAIT_DEFAULT;
							}
							channelconfig[ch].raffstate.mode=RAFFSTORE_DOWN;
						} else if (channelconfig[ch].raffstate.angleTarget<channelconfig[ch].raffstate.angle) {
							//need to go further up
							if (channelconfig[ch].raffstate.mode==RAFFSTORE_DOWN) {
								channelconfig[ch].raffstate.wait = CHANNELCONFIG_RAFFSTORE_WAIT_DEFAULT;
							}
							channelconfig[ch].raffstate.mode=RAFFSTORE_UP;
						}
					}

					if (channelconfig[ch].raffstate.wait==0) {
						if (channelconfig[ch].raffstate.mode==RAFFSTORE_DOWN) {
							//moving downwards
							channelconfig_setPort(channelconfig[ch].port[0],0);
							channelconfig_setPort(channelconfig[ch].port[1],1);
							if (channelconfig[ch].raffstate.angle+channelconfig[ch].raffstate.angleClose<CHANNELCONFIG_RAFFSTORE_ANGLE_MAX) {
								channelconfig[ch].raffstate.angle += channelconfig[ch].raffstate.angleClose;
							} else if (channelconfig[ch].raffstate.angle<CHANNELCONFIG_RAFFSTORE_ANGLE_MAX) {
								channelconfig[ch].raffstate.angle = CHANNELCONFIG_RAFFSTORE_ANGLE_MAX;
							} else {
								if (channelconfig[ch].raffstate.position+channelconfig[ch].raffstate.positionDown<CHANNELCONFIG_RAFFSTORE_POSITION_MAX) {
									channelconfig[ch].raffstate.position += channelconfig[ch].raffstate.positionDown;
								} else if (channelconfig[ch].raffstate.position<CHANNELCONFIG_RAFFSTORE_POSITION_MAX) {
									channelconfig[ch].raffstate.position = CHANNELCONFIG_RAFFSTORE_POSITION_MAX;
								}
							}
						}
						if (channelconfig[ch].raffstate.mode==RAFFSTORE_UP) {
							//moving upwards
							channelconfig_setPort(channelconfig[ch].port[1],0);
							channelconfig_setPort(channelconfig[ch].port[0],1);
							if (channelconfig[ch].raffstate.angle>=channelconfig[ch].raffstate.angleOpen) {
								channelconfig[ch].raffstate.angle -= channelconfig[ch].raffstate.angleOpen;
							} else if (channelconfig[ch].raffstate.angle>0) {
								channelconfig[ch].raffstate.angle = 0;
							} else {
								if (channelconfig[ch].raffstate.position>=channelconfig[ch].raffstate.positionUp) {
									channelconfig[ch].raffstate.position -= channelconfig[ch].raffstate.positionUp;
								} else if (channelconfig[ch].raffstate.position>0) {
									channelconfig[ch].raffstate.position = 0;
								}
							}
						}
					} else {
						channelconfig_setPort(channelconfig[ch].port[0],0);
						channelconfig_setPort(channelconfig[ch].port[1],0);
						channelconfig[ch].raffstate.wait--;
					}
				}
			}
		break;
#endif
		default:
		break;
		}
	}
}

ISR(TIMER3_COMPA_vect) {
	//10ms interrupt
	channelconfig_10msISR();
	channelconfig_10msUserISR();
	counter++;
	if (counter%10==0) {
		timer100ms = 1;
	}
	if (counter==100) {
		timer1s = 1;
		counter = 0;
	}
}


void channelconfig_storeConfig(void) {
	eeprom_update_block(channelconfig,(uint8_t *)EEPROM_CHANNELCONFIG_DATA,sizeof(channelconfig));
	eeprom_update_byte((uint8_t *)EEPROM_CHANNELCONFIG_MARKER,MARKER_MAGIC);
}

void channelconfig_clearConfig(void) {
	uint8_t ch;
	channelconfig_t config;
	memset(&config,0,sizeof(channelconfig_t));
	config.function = FUNCTION_NONE;
	for (ch=0;ch<=CHANNELCONFIG_MAX_CONFIG;ch++) {
		channelconfig_configure(ch,&config);
	}
}

void transmitChannelConfig(uint8_t channel) {
	homecan_t msg;
	msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
	msg.header.mode = HOMECAN_HEADER_MODE_SRC;
	msg.msgtype = HOMECAN_MSGTYPE_CHANNEL_CONFIG;
	msg.address = homecan_getDeviceID();
	msg.channel = channel;
	msg.data[0] = channelconfig[channel].function;
	switch (channelconfig[channel].function) {
	case FUNCTION_NONE:
	case FUNCTION_RESERVED:
		msg.length = 1;
		break;
#ifdef CONFIG_INPUT
	case FUNCTION_INPUT:
		msg.length = 2;
		msg.data[1] = channelconfig[channel].port[0];
		break;
#endif
#ifdef CONFIG_OUTPUT
	case FUNCTION_OUTPUT:
		msg.length = 2;
		msg.data[1] = channelconfig[channel].port[0];
		break;
#endif
#ifdef CONFIG_ELTAKO
	case FUNCTION_DIMMER:
		msg.length = 2;
		msg.data[1] = channelconfig[channel].port[0];
		break;
	case FUNCTION_FTK:
		msg.length = 8;
		msg.data[1] = channelconfig[channel].port[0];
		msg.data[2] = channelconfig[channel].port[1];
		msg.data[3] = channelconfig[channel].ftkstate.sniffing;
		memcpy(&msg.data[4],&channelconfig[channel].ftkstate.id_buffer[0],4);
		break;
#endif
#ifdef CONFIG_TEMP
	case FUNCTION_TEMPSENS:
		msg.length = 4;
		msg.data[1] = channelconfig[channel].port[0];
		msg.data[2] = channelconfig[channel].port[1];
		msg.data[3] = channelconfig[channel].tempstate.intervall;
		break;
#endif
#ifdef CONFIG_LED
	case FUNCTION_LED:
		msg.length = 2;
		msg.data[1] = channelconfig[channel].port[0];
		break;
#endif
#ifdef CONFIG_BUZZER
	case FUNCTION_BUZZER:
		msg.length = 2;
		msg.data[1] = channelconfig[channel].port[0];
		break;
#endif
#ifdef CONFIG_IR
	case FUNCTION_IRTX:
	case FUNCTION_IRRX:
		msg.length = 2;
		msg.data[1] = channelconfig[channel].port[0];
		break;
#endif
#ifdef CONFIG_MOTION
	case FUNCTION_MOTION:
		msg.length = 2;
		msg.data[1] = channelconfig[channel].port[0];
		break;
#endif
#ifdef CONFIG_ANALOG
	case FUNCTION_HUMIDITY:
	case FUNCTION_LUMINOSITY:
		msg.length = 2;
		msg.data[1] = channelconfig[channel].port[0];
		break;
#endif
#ifdef CONFIG_KEYPAD
	case FUNCTION_KEYPAD:
		msg.length = 2;
		msg.data[1] = channelconfig[channel].port[0];
		break;
#endif
#ifdef CONFIG_RAFFSTORE
	case FUNCTION_RAFFSTORE:
		msg.length = 7;
		msg.data[1] = channelconfig[channel].port[0];
		msg.data[2] = channelconfig[channel].port[1];
		msg.data[3] = channelconfig[channel].raffstate.positionUp;
		msg.data[4] = channelconfig[channel].raffstate.positionDown;
		msg.data[5] = channelconfig[channel].raffstate.angleOpen;
		msg.data[6] = channelconfig[channel].raffstate.angleClose;
		break;
#endif
#ifdef CONFIG_SSR
	case FUNCTION_SSR:
		msg.length = 3;
		msg.data[1] = channelconfig[channel].port[0];
		msg.data[2] = channelconfig[channel].port[1];
		break;
#endif
#ifdef CONFIG_KWB
	case FUNCTION_KWB_INPUT:
		msg.length = 5;
		msg.data[1] = channelconfig[channel].port[0];
		msg.data[2] = channelconfig[channel].port[1];
		msg.data[3] = channelconfig[channel].kwbstate.channel;
		msg.data[4] = channelconfig[channel].kwbstate.intervall;
		break;
	case FUNCTION_KWB_TEMP:
		msg.length = 5;
		msg.data[1] = channelconfig[channel].port[0];
		msg.data[2] = channelconfig[channel].port[1];
		msg.data[3] = channelconfig[channel].kwbtemp.channel;
		msg.data[4] = channelconfig[channel].kwbtemp.intervall;
		break;
	case FUNCTION_KWB_HK:
		msg.length = 4;
		msg.data[1] = channelconfig[channel].port[0];
		msg.data[2] = channelconfig[channel].port[1];
		msg.data[3] = channelconfig[channel].kwbhk.mode;
		break;
#endif
#ifdef CONFIG_HOMECAN_GATEWAY
	case FUNCTION_BUSLOAD:
		msg.length = 2;
		msg.data[1] = channelconfig[channel].port[0];
		msg.data[2] = 0xFF;
		msg.data[3] = channelconfig[channel].busloadstate.intervall;
		break;
#endif
	}
	while (!homecan_transmit(&msg)) {
		_delay_ms(1);
	}
}

bool channelconfig_configure(uint8_t channel, const channelconfig_t *config) {
	if (channel > CHANNELCONFIG_MAX_CONFIG)
		return false;
	//TODO reset old function state to init state
	switch (channelconfig[channel].function) {
#ifdef CONFIG_OUTPUT
	case FUNCTION_OUTPUT:
		channelconfig_setPort(channelconfig[channel].port[0], 0);
		break;
#endif
#ifdef CONFIG_RAFFSTORE
	case FUNCTION_RAFFSTORE:
		channelconfig_setPort(channelconfig[channel].port[0], 0);
		channelconfig_setPort(channelconfig[channel].port[1], 0);
		break;
#endif
#ifdef CONFIG_SSR
	case FUNCTION_SSR: {
		uint8_t loop = 1;
		while (channelconfig_getPort(channelconfig[channel].port[1]) && loop <= CHANNELCONFIG_SSR_MAX_REPEAT) {
			channelconfig_setPort(channelconfig[channel].port[0], 0x01);
			_delay_ms(CHANNELCONFIG_SSR_DELAY_MS * loop);
			channelconfig_setPort(channelconfig[channel].port[0], 0x00);
			loop++;
		}
	}
		break;
#endif
#ifdef CONFIG_ELTAKO
	case FUNCTION_DIMMER:
		//TODO turn off Dimmer
		break;
#endif
#ifdef CONFIG_BUZZER
	case FUNCTION_BUZZER:
		//TODO turn off buzzer
		break;
#endif
#ifdef CONFIG_LED
	case FUNCTION_LED:
		channelconfig_setPort(channelconfig[channel].port[0], 0);
		break;
#endif
	default:
		break;
	}
	//clean up old config
	memset(&channelconfig[channel],0,sizeof(channelconfig_t));
	//check new config
	switch(config->function) {
#ifdef CONFIG_ELTAKO
	case FUNCTION_DIMMER:
		if (channelconfig_getPortType(config->port[0])==CIRCUIT_RS485TX) {
			//valid
		} else {
			return false;
		}
		break;
	case FUNCTION_FTK:
		if (channelconfig_getPortType(config->port[0])==CIRCUIT_RS485RX) {
			//valid
		} else {
			return false;
		}
		break;
#endif
#ifdef CONFIG_OUTPUT
	case FUNCTION_OUTPUT:
		if (channelconfig_getPortType(config->port[0])==CIRCUIT_OUT) {
			//valid
		} else {
			return false;
		}
		break;
#endif
#ifdef CONFIG_INPUT
	case FUNCTION_INPUT:
		if (channelconfig_getPortType(config->port[0])==CIRCUIT_INPUT || channelconfig_getPortType(config->port[0])==CIRCUIT_OCIN) {
			//valid
		} else {
			return false;
		}
		break;
#endif
#ifdef CONFIG_RAFFSTORE
	case FUNCTION_RAFFSTORE:
		if (channelconfig_getPortType(config->port[0])==CIRCUIT_OUT && channelconfig_getPortType(config->port[1])==CIRCUIT_OUT) {
			//valid
		} else {
			return false;
		}
		break;
#endif
#ifdef CONFIG_SSR
	case FUNCTION_SSR:
		if (channelconfig_getPortType(config->port[0])==CIRCUIT_OUT && (channelconfig_getPortType(config->port[1])==CIRCUIT_INPUT || channelconfig_getPortType(config->port[1])==CIRCUIT_OCIN)) {
			//valid
		} else {
			return false;
		}
		break;
#endif
#ifdef CONFIG_TEMP
	case FUNCTION_TEMPSENS:
		if (channelconfig_getPortType(config->port[0])==CIRCUIT_I2C || channelconfig_getPortType(config->port[0])==CIRCUIT_1WIRE) {
			//valid
		} else {
			return false;
		}
		break;
#endif
#ifdef CONFIG_LED
	case FUNCTION_LED:
		if (channelconfig_getPortType(config->port[0])==CIRCUIT_LEDOUT) {
			//valid
		} else {
			return false;
		}
		break;
#endif
#ifdef CONFIG_KEYPAD
	case FUNCTION_KEYPAD:
		if (channelconfig_getPortType(config->port[0])==CIRCUIT_KEYPAD) {
			//valid
		} else {
			return false;
		}
		break;
#endif
#ifdef CONFIG_MOTION
	case FUNCTION_MOTION:
		if (channelconfig_getPortType(config->port[0])==CIRCUIT_INPUT) {
			//valid
		} else {
			return false;
		}
		break;
#endif
#ifdef CONFIG_ANALOG
	case FUNCTION_HUMIDITY:
	case FUNCTION_LUMINOSITY:
		if (channelconfig_getPortType(config->port[0])==CIRCUIT_ANALOG) {
			//valid
		} else {
			return false;
		}
		break;
#endif
#ifdef CONFIG_KWB
	case FUNCTION_KWB_INPUT:
	case FUNCTION_KWB_TEMP:
		if (channelconfig_getPortType(config->port[0])==CIRCUIT_RS485RX) {
			//valid
		} else {
			return false;
		}
		break;
#endif
#ifdef CONFIG_POTIO
	case FUNCTION_KWB_HK:
		if (channelconfig_getPortType(config->port[0])==CIRCUIT_I2C) {
			//valid
		} else {
			return false;
		}
		break;
#endif
#ifdef CONFIG_HOMECAN_GATEWAY
	case FUNCTION_BUSLOAD:
		if (channelconfig_getPortType(config->port[0])==CIRCUIT_NONE) {
			busloadChannel = channel;
		} else {
			return false;
		}
		break;
#endif
	case FUNCTION_NONE:
		break;
	default:
		return false;
		break;
	}
	memcpy(&channelconfig[channel],config,sizeof(channelconfig_t));
	return true;
}

void channelconfig_receiveTask(void) {
	homecan_t msg;

#ifdef CONFIG_ELTAKO
	//check for rs485 messages
	rs485eltakoReceiveTask();
#endif
#ifdef CONFIG_KWB
	rs485kwbReceiveTask();
#endif

	//check for incoming can messages
	while (homecan_receive(&msg)) {
#ifdef CONFIG_HOMECAN_GATEWAY
		if (busloadChannel!=0) {
			channelconfig[busloadChannel].busloadstate.byteCount += msg.length+8;
		}
#endif
		if (msg.header.mode==HOMECAN_HEADER_MODE_DST && msg.address == homecan_getDeviceID()) {
			//message is for this device
			if (msg.channel<=CHANNELCONFIG_MAX_CONFIG) {
				switch (msg.msgtype) {
				case HOMECAN_MSGTYPE_CHANNEL_CONFIG:
					if (msg.channel>=0 && msg.channel<=CHANNELCONFIG_MAX_CONFIG){
						channelconfig_t config;
						memset(&config,0,sizeof(channelconfig_t));
						config.function = msg.data[0];
						switch (config.function) {
						case FUNCTION_NONE:
						case FUNCTION_RESERVED:
							break;
#ifdef CONFIG_INPUT
						case FUNCTION_INPUT:
							config.port[0] = msg.data[1];
							config.state = 0;
							break;
#endif
#ifdef CONFIG_OUTPUT
						case FUNCTION_OUTPUT:
							config.port[0] = msg.data[1];
							config.state = 0;
							break;
#endif
#ifdef CONFIG_RAFFSTORE
						case FUNCTION_RAFFSTORE:
							config.port[0] = msg.data[1];
							config.port[1] = msg.data[2];
							config.raffstate.positionUp = msg.data[3];
							config.raffstate.positionDown = msg.data[4];
							config.raffstate.angleOpen = msg.data[5];
							config.raffstate.angleClose= msg.data[6];

							config.raffstate.positionTarget = 0;
							config.raffstate.angleTarget = 0;
							config.raffstate.position = CHANNELCONFIG_RAFFSTORE_POSITION_MAX;
							config.raffstate.angle = CHANNELCONFIG_RAFFSTORE_ANGLE_MAX;
							config.raffstate.mode = RAFFSTORE_MOVE;
							config.raffstate.wait = 0;
							break;
#endif
#ifdef CONFIG_SSR
						case FUNCTION_SSR:
							config.port[0] = msg.data[1];
							config.port[1] = msg.data[2];
							config.state = 0;
							break;
#endif
#ifdef CONFIG_ELTAKO
						case FUNCTION_DIMMER:
							config.port[0] = msg.data[1];
							rs485eltako_t txMsg;
							txMsg.id = msg.channel;
							txMsg.org = RS485ELTAKO_ORG_4BS;
							txMsg.data = rs485eltako_createDimmerValue(0);
							while (!rs485eltako_transmitMessage(&txMsg)) {
								_delay_ms(1);
							}
							config.dimmerstate.value = 0;
							break;
						case FUNCTION_FTK:
							config.port[0] = msg.data[1];
							config.ftkstate.sniffing = msg.data[3];
							memcpy(&config.ftkstate.id_buffer[0],&msg.data[4],4);
							config.ftkstate.state = 0;
							break;
#endif
#ifdef CONFIG_TEMP
						case FUNCTION_TEMPSENS:
							config.port[0] = msg.data[1];
							config.tempstate.value = 0.0;
							config.tempstate.intervall = msg.data[3];
							config.tempstate.counter = 0;
							break;
#endif
#ifdef CONFIG_LED
						case FUNCTION_LED:
							config.port[0] = msg.data[1];
							config.ledstate.mode = LED_MODE_OFF;
							config.ledstate.time = 0;
							break;
#endif
#ifdef CONFIG_BUZZER
						case FUNCTION_BUZZER:
							config.port[0] = msg.data[1];
							config.changed = 1;
							config.buzzerstate.freq = 0;
							break;
#endif
#ifdef CONFIG_IR
						case FUNCTION_IRTX:
						case FUNCTION_IRRX:
							config.port[0] = msg.data[1];
							config.changed = 1;
							break;
#endif
#ifdef CONFIG_MOTION
						case FUNCTION_MOTION:
							config.port[0] = msg.data[1];
							config.changed = 1;
							config.state = 0;
							break;
#endif
#ifdef CONFIG_ANALOG
						case FUNCTION_HUMIDITY:
							config.port[0] = msg.data[1];
							config.humiditystate.intervall = msg.data[3];
							config.humiditystate.counter = 0;
							config.humiditystate.offset = msg.data[4];
							config.humiditystate.scale = msg.data[5];
							break;
						case FUNCTION_LUMINOSITY:
							config.port[0] = msg.data[1];
							config.analogstate.avg = 0;
							config.analogstate.intervall = msg.data[3];
							config.analogstate.counter = 0;
							break;
#endif
#ifdef CONFIG_KEYPAD
						case FUNCTION_KEYPAD:
							config.port[0] = msg.data[1];
							break;
#endif
#ifdef CONFIG_KWB
						case FUNCTION_KWB_INPUT:
							config.port[0] = msg.data[1];
							config.kwbstate.value = 0xFF;
							config.kwbstate.channel = msg.data[3];
							config.kwbstate.intervall = msg.data[4];
							config.kwbstate.counter = 0;
							break;
						case FUNCTION_KWB_TEMP:
							config.port[0] = msg.data[1];
							config.kwbtemp.value = 0.0;
							config.kwbtemp.channel = msg.data[3];
							config.kwbtemp.intervall = msg.data[4];
							config.kwbtemp.counter = 0;
							break;
#ifdef CONFIG_POTIO
						case FUNCTION_KWB_HK:
							config.port[0] = msg.data[1];
							config.kwbhk.hk = msg.data[3];
							config.kwbhk.mode = 2;	//Automatik Mode
							break;
#endif
#endif
#ifdef CONFIG_HOMECAN_GATEWAY
						case FUNCTION_BUSLOAD:
							busloadChannel = msg.channel;
							config.port[0] = msg.data[1];
							config.busloadstate.byteCount = 0;
							config.busloadstate.counter = 0;
							config.busloadstate.intervall = msg.data[3];
							break;
#endif
						}
						config.changed = 1;
						channelconfig_configure(msg.channel,&config);
						transmitChannelConfig(msg.channel);
					}
					break;
				case HOMECAN_MSGTYPE_GET_CONFIG:
					transmitChannelConfig(msg.channel);
					break;
				case HOMECAN_MSGTYPE_STORE_CONFIG:
					channelconfig_storeConfig();
					transmitChannelConfig(0);
					break;
				case HOMECAN_MSGTYPE_CLEAR_CONFIG:
					channelconfig_clearConfig();
					transmitChannelConfig(0);
					break;
				case HOMECAN_MSGTYPE_REQUEST_STATE:
					channelconfig[msg.channel].changed = 1;
					//transmitChannelState(msg.channel);
					break;
				case HOMECAN_MSGTYPE_ONOFF:
#ifdef CONFIG_OUTPUT
					if (channelconfig[msg.channel].function==FUNCTION_OUTPUT) {
						channelconfig_setPort(channelconfig[msg.channel].port[0],msg.data[0]);
					}
#endif
#ifdef CONFIG_SSR
					if (channelconfig[msg.channel].function==FUNCTION_SSR) {
						if (msg.data[0]!=channelconfig_getPort(channelconfig[msg.channel].port[1])) {
							channelconfig_setPort(channelconfig[msg.channel].port[0],0x01);
							_delay_ms(CHANNELCONFIG_SSR_DELAY_MS);
							channelconfig_setPort(channelconfig[msg.channel].port[0],0x00);
						}
					}
#endif
#ifdef CONFIG_ELTAKO
					if (channelconfig[msg.channel].function==FUNCTION_DIMMER) {
						rs485eltako_t txMsg;
						txMsg.id = msg.channel;
						txMsg.org = RS485ELTAKO_ORG_4BS;
						txMsg.data = rs485eltako_createDimmerValue(msg.data[0]==0?0:255);
						while (!rs485eltako_transmitMessage(&txMsg)) {
							_delay_ms(1);
						}
						channelconfig[msg.channel].changed = 1;
						channelconfig[msg.channel].dimmerstate.value = msg.data[0]==0?0:255;
					}
#endif
#ifdef CONFIG_LED
					if (channelconfig[msg.channel].function==FUNCTION_LED) {
						channelconfig[msg.channel].ledstate.mode = msg.data[0]==0?LED_MODE_OFF:LED_MODE_ON;
						channelconfig[msg.channel].ledstate.time = 0;
						channelconfig[msg.channel].changed = 1;
					}
#endif
					break;
#ifdef CONFIG_ELTAKO
				case HOMECAN_MSGTYPE_DIMMER:
					if (channelconfig[msg.channel].function==FUNCTION_DIMMER) {
						rs485eltako_t txMsg;
						txMsg.id = msg.channel;
						txMsg.org = RS485ELTAKO_ORG_4BS;
						txMsg.data = rs485eltako_createDimmerValue(msg.data[0]);
						while (!rs485eltako_transmitMessage(&txMsg)) {
							_delay_ms(1);
						}
						channelconfig[msg.channel].changed = 1;
						channelconfig[msg.channel].dimmerstate.value = msg.data[0];
					}
					break;
				case HOMECAN_MSGTYPE_DIMMER_LEARN:
					if (channelconfig[msg.channel].function==FUNCTION_DIMMER) {
						rs485eltako_t txMsg;
						txMsg.id = msg.channel;
						txMsg.org = RS485ELTAKO_ORG_4BS;
						txMsg.data = RS485ELTAKO_DIMMER_LEARN;
						while (!rs485eltako_transmitMessage(&txMsg)) {
							_delay_ms(1);
						}
					}
					break;
#endif
#ifdef CONFIG_RAFFSTORE
				case HOMECAN_MSGTYPE_POSITION:
					if (channelconfig[msg.channel].function==FUNCTION_RAFFSTORE) {
						channelconfig[msg.channel].raffstate.positionTarget = (((uint32_t)msg.data[0])*CHANNELCONFIG_RAFFSTORE_POSITION_MAX)/255;
						if (channelconfig[msg.channel].raffstate.mode==RAFFSTORE_IDLE) {
							channelconfig[msg.channel].raffstate.mode = RAFFSTORE_MOVE;
						}
					}
					break;
				case HOMECAN_MSGTYPE_SHADE:
					if (channelconfig[msg.channel].function==FUNCTION_RAFFSTORE) {
						channelconfig[msg.channel].raffstate.angleTarget = (((uint32_t)msg.data[0])*CHANNELCONFIG_RAFFSTORE_ANGLE_MAX)/255;
						if (channelconfig[msg.channel].raffstate.mode==RAFFSTORE_IDLE) {
							channelconfig[msg.channel].raffstate.mode = RAFFSTORE_MOVE;
						}
					}
					break;
				case HOMECAN_MSGTYPE_UPDOWN:
					if (channelconfig[msg.channel].function==FUNCTION_RAFFSTORE) {
						if (msg.data[0]==0) {
							channelconfig[msg.channel].raffstate.positionTarget = 0;
							channelconfig[msg.channel].raffstate.angleTarget = 0;
						} else {
							channelconfig[msg.channel].raffstate.positionTarget = CHANNELCONFIG_RAFFSTORE_POSITION_MAX;
							channelconfig[msg.channel].raffstate.angleTarget = CHANNELCONFIG_RAFFSTORE_ANGLE_MAX;
						}
						if (channelconfig[msg.channel].raffstate.mode==RAFFSTORE_IDLE) {
							channelconfig[msg.channel].raffstate.mode = RAFFSTORE_MOVE;
						}
					}
					break;
				case HOMECAN_MSGTYPE_STOPMOVE:
					if (channelconfig[msg.channel].function==FUNCTION_RAFFSTORE) {
						channelconfig[msg.channel].raffstate.angleTarget = channelconfig[msg.channel].raffstate.angle;
						channelconfig[msg.channel].raffstate.positionTarget = channelconfig[msg.channel].raffstate.position;
						if (channelconfig[msg.channel].raffstate.mode==RAFFSTORE_IDLE) {
							channelconfig[msg.channel].raffstate.mode = RAFFSTORE_MOVE;
						}
					}
					break;
#endif
#ifdef CONFIG_ELTAKO
				case HOMECAN_MSGTYPE_INCDEC:
					if (channelconfig[msg.channel].function==FUNCTION_DIMMER) {
						if (msg.data[0]==0) {
							if (channelconfig[msg.channel].dimmerstate.value<=255-CHANNELCONFIG_DIMMER_STEP) {
									channelconfig[msg.channel].dimmerstate.value+=CHANNELCONFIG_DIMMER_STEP;
							} else if (channelconfig[msg.channel].dimmerstate.value<255) {
								channelconfig[msg.channel].dimmerstate.value++;
							}
						} else {
							if (channelconfig[msg.channel].dimmerstate.value>=CHANNELCONFIG_DIMMER_STEP) {
								channelconfig[msg.channel].dimmerstate.value-=CHANNELCONFIG_DIMMER_STEP;
							} else if (channelconfig[msg.channel].dimmerstate.value>0) {
								channelconfig[msg.channel].dimmerstate.value--;
							}
						}
						rs485eltako_t txMsg;
						txMsg.id = msg.channel;
						txMsg.org = RS485ELTAKO_ORG_4BS;
						txMsg.data = rs485eltako_createDimmerValue(channelconfig[msg.channel].dimmerstate.value);
						while (!rs485eltako_transmitMessage(&txMsg)) {
							_delay_ms(1);
						}
						channelconfig[msg.channel].changed = 1;
					}
					break;
				case HOMECAN_MSGTYPE_FTKID:
					break;
#endif
#ifdef CONFIG_BUZZER
				case HOMECAN_MSGTYPE_BUZZER:
					if (channelconfig[msg.channel].function==FUNCTION_BUZZER) {
						channelconfig[msg.channel].buzzerstate.freq = msg.data[0] + (((uint16_t)msg.data[1])<<8);
						channelconfig[msg.channel].changed = 1;
					}
					break;
#endif
#ifdef CONFIG_IR
				case HOMECAN_MSGTYPE_IR:
					if (channelconfig[msg.channel].function==FUNCTION_IRTX) {
						IRMP_DATA irmp_data;
						irmp_data.protocol = msg.data[0];
						irmp_data.address  = msg.data[1] | (((uint16_t)msg.data[2])<<8);
						irmp_data.command  = msg.data[3] | (((uint16_t)msg.data[4])<<8);
						irmp_data.flags    = msg.data[5];
						irsnd_send_data (&irmp_data, TRUE);
					}
					break;
#endif
#ifdef CONFIG_KWB
#ifdef CONFIG_POTIO
				case HOMECAN_MSGTYPE_KWB_HK:
					if (channelconfig[msg.channel].function==FUNCTION_KWB_HK) {
						channelconfig[msg.channel].kwbhk.mode = msg.data[0];
						switch (channelconfig[msg.channel].kwbhk.mode) {
						case 0:
							mcp4651_setWiper(channelconfig[msg.channel].kwbhk.hk,CHANNELCONFIG_KWB_HK_OFF);
							break;
						case 1:
							mcp4651_setWiper(channelconfig[msg.channel].kwbhk.hk,CHANNELCONFIG_KWB_HK_NIGHT);
							break;
						case 2:
							mcp4651_setWiper(channelconfig[msg.channel].kwbhk.hk,CHANNELCONFIG_KWB_HK_DAY);
							break;
						case 3:
							mcp4651_setWiper(channelconfig[msg.channel].kwbhk.hk,CHANNELCONFIG_KWB_HK_AUTO);
							break;
						}
						channelconfig[msg.channel].changed = 1;
					}
					break;
#endif
#endif
				case HOMECAN_MSGTYPE_BOOTLOADER:
				case HOMECAN_MSGTYPE_CALL_BOOTLOADER: //already handled inside homecan.c
				//all following are only updates, no plan to react on updates inside homecan, updates are handled by openhab
				case HOMECAN_MSGTYPE_HEARTBEAT:
#ifdef CONFIG_MOTION
				case HOMECAN_MSGTYPE_MOTION:
#endif
				case HOMECAN_MSGTYPE_OPENCLOSED:
				case HOMECAN_MSGTYPE_TEMPERATURE:
				case HOMECAN_MSGTYPE_HUMIDITY:
				case HOMECAN_MSGTYPE_LUMINOSITY:
				case HOMECAN_MSGTYPE_UV_INDEX:
				case HOMECAN_MSGTYPE_AIR_PRESSURE:
				case HOMECAN_MSGTYPE_WIND_SPEED:
				case HOMECAN_MSGTYPE_WIND_DIRECTION:
				case HOMECAN_MSGTYPE_RAIN:
				case HOMECAN_MSGTYPE_FLOAT:
				case HOMECAN_MSGTYPE_UINT32:
#ifdef CONFIG_KEYPAD
				case HOMECAN_MSGTYPE_KEY_SEQUENCE:
#endif
				case HOMECAN_MSGTYPE_STRING:
					break;
/*
				case HOMECAN_MSGTYPE_LED:
					if (channelconfig[msg.channel].function==FUNCTION_LED) {
						channelconfig[msg.channel].ledstate.mode = msg.data[0];
						channelconfig[msg.channel].ledstate.time = msg.data[1];
						channelconfig[msg.channel].changed = 1;
					}
					break;
*/
				}
			}
		}
	}
}

#ifdef HEARBEAT_PERIODIC
uint8_t hearbeatCounter = 0;
#define HEARTBEAT_INTERVALL 30
#endif

void channelconfig_1sTask(void) {
	uint8_t ch;

#ifdef HEARBEAT_PERIODIC
	hearbeatCounter++;
	if (hearbeatCounter>=HEARTBEAT_INTERVALL) {
		hearbeatCounter = 0;
		homecan_transmitHeartbeat();
	}
#endif
	//check all channels, send updates if something changed
	for (ch=0;ch<=CHANNELCONFIG_MAX_CONFIG;ch++) {
		switch (channelconfig[ch].function) {
#ifdef CONFIG_TEMP
		case FUNCTION_TEMPSENS:
	#ifdef CONFIG_ONEWIRE
			if (channelconfig_getPortType(channelconfig[ch].port[0])==CIRCUIT_1WIRE) {
				if (channelconfig[ch].tempstate.counter==0) {
					DS18X20_start_meas( DS18X20_POWER_EXTERN, NULL );
				} else if (channelconfig[ch].tempstate.counter==1) {
					int16_t decicelsius;
					float newVal;
					DS18X20_read_decicelsius_single( id[0], &decicelsius );
					newVal = decicelsius/10.0;
					channelconfig[ch].tempstate.value = newVal;
					channelconfig[ch].changed = 1;
				}
				channelconfig[ch].tempstate.counter++;
				if (channelconfig[ch].tempstate.counter==channelconfig[ch].tempstate.intervall) {
					channelconfig[ch].tempstate.counter = 0;
				}
			}
	#endif
	#ifdef CONFIG_I2C
			if (channelconfig_getPortType(channelconfig[ch].port[0])==CIRCUIT_I2C) {
				if (channelconfig[ch].tempstate.counter==0) {
					float newVal;
					newVal = tmp75_readTemperature();
					channelconfig[ch].tempstate.value = newVal;
					channelconfig[ch].changed = 1;
				}
				channelconfig[ch].tempstate.counter++;
				if (channelconfig[ch].tempstate.counter==channelconfig[ch].tempstate.intervall) {
					channelconfig[ch].tempstate.counter = 0;
				}
			}
	#endif
			break;
#endif
#ifdef CONFIG_KWB
		case FUNCTION_KWB_TEMP:
			if (channelconfig[ch].kwbtemp.counter==0) {
				channelconfig[ch].changed = 1;
			}
			channelconfig[ch].kwbtemp.counter++;
			if (channelconfig[ch].kwbtemp.counter==channelconfig[ch].kwbtemp.intervall) {
				channelconfig[ch].kwbtemp.counter = 0;
			}
			break;
		case FUNCTION_KWB_INPUT:
			if (channelconfig[ch].kwbstate.counter==0) {
				channelconfig[ch].changed = 1;
			}
			channelconfig[ch].kwbstate.counter++;
			if (channelconfig[ch].kwbstate.counter==channelconfig[ch].kwbstate.intervall) {
				channelconfig[ch].kwbstate.counter = 0;
			}
			break;
#endif
#ifdef CONFIG_ANALOG
		case FUNCTION_LUMINOSITY:
			ADMUX = (ADMUX&0xE0)|((channelconfig[ch].port[0])&0x1F);	//switch analog channel
			ADCSRA |= (1<<ADSC);        	//do single conversion
			while(!(ADCSRA & (1<<ADIF)));	//wait for conversion done, ADIF flag active
			channelconfig[ch].analogstate.avg += ADCH;

			if (channelconfig[ch].analogstate.counter==channelconfig[ch].analogstate.intervall-1) {
				channelconfig[ch].analogstate.value = ((float)channelconfig[ch].analogstate.avg)/channelconfig[ch].analogstate.intervall;
				channelconfig[ch].changed = 1;
			}
			channelconfig[ch].analogstate.counter++;
			if (channelconfig[ch].analogstate.counter==channelconfig[ch].analogstate.intervall) {
				channelconfig[ch].analogstate.counter = 0;
				channelconfig[ch].analogstate.avg = 0;
			}
			break;
		case FUNCTION_HUMIDITY:
			if (channelconfig[ch].humiditystate.counter==channelconfig[ch].humiditystate.intervall) {
				if (channelconfig[ch].humiditystate.scale==0) {
					channelconfig[ch].humiditystate.value = channelconfig[ch].humiditystate.accumulator/channelconfig[ch].humiditystate.intervall;
				} else {
					channelconfig[ch].humiditystate.value = (((channelconfig[ch].humiditystate.accumulator/channelconfig[ch].humiditystate.intervall)*channelconfig[ch].humiditystate.scale)/256)+channelconfig[ch].humiditystate.offset;
				}
				channelconfig[ch].humiditystate.counter = 0;
				channelconfig[ch].humiditystate.accumulator = 0;
				channelconfig[ch].changed = 1;
			} else {
				uint16_t ADCr = 0;
				uint8_t i;

				ADMUX = (ADMUX&0xE0)|((channelconfig[ch].port[0])&0x1F);	//switch analog channel
				//do a dummy readout first
				ADCSRA |= (1<<ADSC);        	//do single conversion
				while(!(ADCSRA & (1<<ADIF)));	//wait for conversion done, ADIF flag active

				for(i=0;i<32;i++)            // do the ADC conversion several times for better accuracy
				{
					ADCSRA |= (1<<ADSC);        // do single conversion
					while(!(ADCSRA & (1<<ADIF)));    // wait for conversion done, ADIF flag active

					ADCr += ADCH;    // read out ADCH register and accumulate result (8 samples) for later averaging
				}

				channelconfig[ch].humiditystate.accumulator += (ADCr >> 5);	// average the samples, and add to accumulator for lontime avg
				channelconfig[ch].humiditystate.counter++;
			}
			break;
#endif
#ifdef CONFIG_HOMECAN_GATEWAY
		case FUNCTION_BUSLOAD:		
			channelconfig[ch].busloadstate.counter++;
			if (channelconfig[ch].busloadstate.counter==channelconfig[ch].busloadstate.intervall && channelconfig[ch].busloadstate.intervall!=0) {
				channelconfig[ch].busloadstate.counter = 0;
				channelconfig[ch].changed = 1;
			}
			break;
#endif
		default:
			break;
		}
	}
}


void transmitChannelState(uint8_t ch) {
	//check channel, send updates if  changed
	homecan_t msg __attribute__ ((unused));
	float *data __attribute__ ((unused));

	if (channelconfig[ch].changed) {
		switch (channelconfig[ch].function) {
		case FUNCTION_NONE:
		case FUNCTION_RESERVED:
			break;
#ifdef CONFIG_INPUT
		case FUNCTION_INPUT:
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_ONOFF;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = 1;
			msg.data[0] = channelconfig[ch].state;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			break;
#endif
#ifdef CONFIG_OUTPUT
		case FUNCTION_OUTPUT:
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_ONOFF;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = 1;
			msg.data[0] = channelconfig[ch].state;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			break;
#endif
#ifdef CONFIG_MOTION
		case FUNCTION_MOTION:
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_MOTION;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = 1;
			msg.data[0] = channelconfig[ch].state;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			break;
#endif
#ifdef CONFIG_SSR
		case FUNCTION_SSR:
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_ONOFF;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = 1;
			msg.data[0] = channelconfig[ch].state;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			break;
#endif
#ifdef CONFIG_ELTAKO
		case FUNCTION_DIMMER:
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_DIMMER;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = 1;
			msg.data[0] = channelconfig[ch].state;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			break;
		case FUNCTION_FTK:
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_OPENCLOSED;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = 1;
			msg.data[0] = channelconfig[ch].ftkstate.state;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			break;
#endif
#ifdef CONFIG_LED
		case FUNCTION_LED:
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_ONOFF;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = 1;
			msg.data[0] = channelconfig[ch].ledstate.mode==LED_MODE_OFF?0:1;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			break;
#endif
#ifdef CONFIG_RAFFSTORE
		case FUNCTION_RAFFSTORE:
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_POSITION;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = 1;
			msg.data[0] = (((uint32_t)channelconfig[ch].raffstate.position)*255)/CHANNELCONFIG_RAFFSTORE_POSITION_MAX;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_SHADE;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = 1;
			msg.data[0] = (((uint32_t)channelconfig[ch].raffstate.angle)*255)/CHANNELCONFIG_RAFFSTORE_ANGLE_MAX;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			/*
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_STOPMOVE;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = 1;
			msg.data[0] = channelconfig[ch].raffstate.mode==RAFFSTORE_IDLE?0:1;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			break;
			*/
#endif
#ifdef CONFIG_KEYPAD
		case FUNCTION_KEYPAD:
			//handled inside 10ms ISR
			break;
#endif
#ifdef CONFIG_IR
		case FUNCTION_IRTX:
			//IR handled directly in 100ms Task
			break;
		case FUNCTION_IRRX:
			//IR handled directly in 100ms Task
			break;
#endif
#ifdef CONFIG_BUZZER
		case FUNCTION_BUZZER:
			{
				uint16_t freq = channelconfig[ch].buzzerstate.freq;
				msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
				msg.header.mode = HOMECAN_HEADER_MODE_SRC;
				msg.msgtype = HOMECAN_MSGTYPE_BUZZER;
				msg.address = homecan_getDeviceID();
				msg.channel = ch;
				msg.length = 2;
				msg.data[0] = freq&0xFF;
				msg.data[1] = freq>>8;
				while (!homecan_transmit(&msg)) {
					_delay_ms(1);
				}
			}
			break;
#endif
#ifdef CONFIG_KWB
		case FUNCTION_KWB_TEMP:
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_TEMPERATURE;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = sizeof(float);
			data = (float*)&msg.data[0];
			*data = channelconfig[ch].kwbtemp.value;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			break;
		case FUNCTION_KWB_INPUT:
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_ONOFF;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = 1;
			msg.data[0] = channelconfig[ch].kwbstate.value;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			break;
#ifdef CONFIG_POTIO
		case FUNCTION_KWB_HK:
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_KWB_HK;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = 1;
			msg.data[0] = channelconfig[ch].kwbhk.mode;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			break;
#endif
#endif
#ifdef CONFIG_TEMP
		case FUNCTION_TEMPSENS:
	#ifdef CONFIG_ONEWIRE
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_TEMPERATURE;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = sizeof(float);
			data = (float*)&msg.data[0];
			*data = (float)channelconfig[ch].tempstate.value;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
	#endif
	#ifdef CONFIG_I2C
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_TEMPERATURE;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = sizeof(float);
			data = (float*)&msg.data[0];
			*data = (float)channelconfig[ch].tempstate.value;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
	#endif
			break;
#endif
#ifdef CONFIG_ANALOG
		case FUNCTION_LUMINOSITY:
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_LUMINOSITY;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = sizeof(float);
			data = (float*)&msg.data[0];
			*data = channelconfig[ch].analogstate.value;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			break;
		case FUNCTION_HUMIDITY:
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_HUMIDITY;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = 1;
			msg.data[0] = channelconfig[ch].humiditystate.value;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			break;
#endif
#ifdef CONFIG_HOMECAN_GATEWAY
		case FUNCTION_BUSLOAD:
			msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
			msg.header.mode = HOMECAN_HEADER_MODE_SRC;
			msg.msgtype = HOMECAN_MSGTYPE_FLOAT;
			msg.address = homecan_getDeviceID();
			msg.channel = ch;
			msg.length = sizeof(float);
			data = (float*)&msg.data[0];
			*data = ((float)channelconfig[ch].busloadstate.byteCount+homecan_getTxByteCount())/channelconfig[ch].busloadstate.intervall;
			channelconfig[ch].busloadstate.byteCount = 0;
			while (!homecan_transmit(&msg)) {
				_delay_ms(1);
			}
			break;
#endif

		}
		channelconfig[ch].changed = 0;
	}
}


void channelconfig_100msTask(void) {
	uint8_t ch;
	homecan_t msg __attribute__ ((unused));
#ifdef CONFIG_IR
	IRMP_DATA irmp_data;
#endif

	//check all channels if something changed
	for (ch=0;ch<=CHANNELCONFIG_MAX_CONFIG;ch++) {
		uint8_t in __attribute__ ((unused));
		switch (channelconfig[ch].function) {
		case FUNCTION_NONE:
		case FUNCTION_RESERVED:
			break;
#ifdef CONFIG_INPUT
		case FUNCTION_INPUT:
			in = channelconfig_getPort(channelconfig[ch].port[0]);
			if (channelconfig[ch].state != in) {
				channelconfig[ch].state = in;
				channelconfig[ch].changed = 1;
			}
			break;
#endif
#ifdef CONFIG_OUTPUT
		case FUNCTION_OUTPUT:
			in = channelconfig_getPort(channelconfig[ch].port[0]);
			if (channelconfig[ch].state != in) {
				channelconfig[ch].state = in;
				channelconfig[ch].changed = 1;
			}
			break;
#endif
#ifdef CONFIG_MOTION
		case FUNCTION_MOTION:
			in = channelconfig_getPort(channelconfig[ch].port[0]);
			if (channelconfig[ch].state != in) {
				channelconfig[ch].state = in;
				channelconfig[ch].changed = 1;
			}
			break;
#endif
#ifdef CONFIG_SSR
		case FUNCTION_SSR:
			in = channelconfig_getPort(channelconfig[ch].port[1]);
			if (channelconfig[ch].state != in) {
				channelconfig[ch].state = in;
				channelconfig[ch].changed = 1;
			}
			break;
#endif
#ifdef CONFIG_ELTAKO
		case FUNCTION_DIMMER:
			break;
		case FUNCTION_FTK:
			break;
#endif
#ifdef CONFIG_LED
		case FUNCTION_LED:
			if (channelconfig[ch].ledstate.time>0) {
				channelconfig[ch].ledstate.time--;
				if (channelconfig[ch].ledstate.time==0) {
					channelconfig[ch].ledstate.mode= LED_MODE_OFF;
					channelconfig[ch].changed = 1;
				}
			}
			switch (channelconfig[ch].ledstate.mode) {
			case LED_MODE_OFF:
				channelconfig_setPort(channelconfig[ch].port[0],0);
				break;
			case LED_MODE_ON:
				channelconfig_setPort(channelconfig[ch].port[0],1);
				break;
			case LED_MODE_SLOW:
				channelconfig_setPort(channelconfig[ch].port[0],(channelconfig[ch].ledstate.time>>3)&0x1);
				break;
			case LED_MODE_FAST:
				channelconfig_setPort(channelconfig[ch].port[0],(channelconfig[ch].ledstate.time)&0x1);
				break;
			}
			break;
#endif

#ifdef CONFIG_ANALOG
		case FUNCTION_HUMIDITY:
		case FUNCTION_LUMINOSITY:
			break;
#endif
#ifdef CONFIG_TEMP
		case FUNCTION_TEMPSENS:
			break;
#endif
#ifdef CONFIG_KEYPAD
		case FUNCTION_KEYPAD:
			//handled inside 10ms ISR
			break;
#endif
#ifdef CONFIG_RAFFSTORE
		case FUNCTION_RAFFSTORE:
			//handled inside 10ms ISR
			break;
#endif
#ifdef CONFIG_IR
		case FUNCTION_IRTX:
			//only send received HomeCAN msgs over IR, but nothing else to do
			break;
		case FUNCTION_IRRX:
			if (irmp_get_data (&irmp_data))
			{
				msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
				msg.header.mode = HOMECAN_HEADER_MODE_SRC;
				msg.msgtype = HOMECAN_MSGTYPE_IR;
				msg.address = homecan_getDeviceID();
				msg.channel = ch;
				msg.length = 6;
				msg.data[0] = irmp_data.protocol;
				msg.data[1] = irmp_data.address&0xFF;
				msg.data[2] = irmp_data.address>>8;
				msg.data[3] = irmp_data.command&0xFF;
				msg.data[4] = irmp_data.command>>8;
				msg.data[5] = irmp_data.flags;
				while (!homecan_transmit(&msg)) {
					_delay_ms(1);
				}
			}
			break;
#endif
#ifdef CONFIG_BUZZER
		case FUNCTION_BUZZER:
			break;
#endif
#ifdef CONFIG_KWB
		case FUNCTION_KWB_TEMP:
			//slow, handled in 1s task
			break;
		case FUNCTION_KWB_INPUT:
			break;
	#ifdef CONFIG_POTIO
		case FUNCTION_KWB_HK:
 			break;
	#endif
#endif
#ifdef CONFIG_HOMECAN_GATEWAY
		case FUNCTION_BUSLOAD:
			//slow, handled in 1s task
			break;
#endif
		}
		transmitChannelState(ch);
	}
}

void channelconfig_task() {
	channelconfig_receiveTask();
	if (timer100ms) {
		timer100ms = 0;
		channelconfig_setStatusLED(1,1);
		channelconfig_100msTask();
		channelconfig_100msUserTask();
		channelconfig_setStatusLED(1,0);
	}
	if (timer1s) {
		timer1s = 0;
		channelconfig_1sTask();
		channelconfig_1sUserTask();
	}
}
