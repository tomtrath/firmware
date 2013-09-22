/*
 * homecan.c
 *
 * Created: 04.11.2012 16:27:41
 *  Author: thomas
 */ 
 
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>

#include "global.h"
#include "homecan.h"

#ifdef CONFIG_HOMECAN_UDP
#include "enc28j60.h"
#include "ip_arp_udp_tcp.h"
#include "net.h"
#endif

#ifdef CONFIG_HOMECAN_CAN
#include "can.h"

static can_t msgtx,msgrx;
static can_filter_t filter = {
	.id = 0x100,
	.mask = 0,
	.flags = {
		.rtr = 0,
		.extended = 0x3
	}
};
#endif

#ifdef CONFIG_HOMECAN_UDP
static uint8_t mymac[6] = {0x00,0x04,0xA3,0x00,0x00,0x03};
static uint8_t broadcastmac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static uint8_t myip[4] = {192,168,1,12};
static uint8_t serverip[4] = {192,168,1,255};


#define BUFFER_SIZE_RX 1518
#define BUFFER_SIZE_TX 250
static uint8_t rxbuf[BUFFER_SIZE_RX+1];
static uint8_t txbuf[BUFFER_SIZE_TX+1];
#endif

static uint8_t deviceID;

uint8_t homecan_getDeviceID() {
	return deviceID;
}

void homecan_init(uint8_t address) {
	wdt_disable();
#ifdef CONFIG_HOMECAN_CAN
	can_init(BITRATE_125_KBPS);
#endif
	if (address==0) {
		//get address from EEPROM
		deviceID = eeprom_read_byte(EEPROM_DEVICE_ID);
	} else {
		deviceID = address;
	}
#ifdef CONFIG_HOMECAN_CAN
#ifdef CONFIG_HOMECAN_GATEWAY
		filter.mask = 0x00;
		filter.id = 0x00;
#else
		filter.flags.extended = 0x3;
		filter.flags.rtr = 0x00;
		filter.mask = 0x0000ff00;
		filter.id = ((uint32_t)deviceID)<<8;
#endif
	can_set_filter(0, &filter);
	can_set_filter(1, &filter);
	can_set_filter(2, &filter);
#endif
#ifdef CONFIG_HOMECAN_UDP
	enc28j60Init(mymac);

	// Magjack leds configuration, see enc28j60 datasheet, page 11
	// LEDB=yellow LEDA=green
	// 0x476 is PHLCON LEDA=links status, LEDB=receive/transmit
	enc28j60PhyWrite(PHLCON,0x476);

	//init the ethernet/ip layer:
	init_udp_or_www_server(mymac,myip);
#endif
}

#ifdef CONFIG_HOMECAN_CAN
bool homecan_transmitCAN(const homecan_t *msg) {
	if (!can_check_free_buffer()) return false;
	// assemble id
	msgtx.id = 	msg->channel |
				((uint32_t)msg->address)<<8 | 
				((uint32_t)msg->msgtype)<<16 |
				((uint32_t)msg->header.mode)<<24 | 				
				((uint32_t)msg->header.priority&0xF)<<25;	
	msgtx.flags.rtr = 0;
	msgtx.flags.extended = 1;
	msgtx.length = msg->length;
	memcpy(&(msgtx.data[0]),&(msg->data[0]),msg->length);
	can_send_message(&msgtx);
	return true;
}
#endif

#ifdef CONFIG_HOMECAN_UDP
bool homecan_transmitUDP(const homecan_t *msg) {
	if (msg->msgtype==HOMECAN_MSGTYPE_BOOTLOADER) {
		send_udp_prepare(txbuf,HOMECAN_UDP_PORT_BOOTLOADER, serverip, HOMECAN_UDP_PORT_BOOTLOADER,broadcastmac);
		memcpy(&txbuf[UDP_DATA_P],&(msg->data[0]),msg->length);
		send_udp_transmit(txbuf,msg->length);
		return true;
	} else {
		send_udp_prepare(txbuf,HOMECAN_UDP_PORT, serverip, HOMECAN_UDP_PORT,broadcastmac);
		txbuf[UDP_DATA_P+0] = (uint8_t)(msg->header.priority<<1 | msg->header.mode);
		txbuf[UDP_DATA_P+1] = msg->msgtype;
		txbuf[UDP_DATA_P+2] = msg->address;
		txbuf[UDP_DATA_P+3] = msg->channel;
		memcpy(&txbuf[UDP_DATA_P+4],&(msg->data[0]),msg->length);
		send_udp_transmit(txbuf,msg->length+4);
		return true;
	}
	return false;
}
#endif

#ifdef CONFIG_HOMECAN_CAN
bool homecan_receiveCAN(homecan_t *msg) {
	if (can_get_message(&msgrx)) {
		msg->address = (msgrx.id>>8)&0xFF;
		msg->header.priority = (msgrx.id>>25)&0xF;
		msg->header.mode = (msgrx.id>>24)&0x1;
		msg->msgtype = (msgrx.id>>16)&0xFF;
		msg->channel = msgrx.id&0xFF;
		msg->length = msgrx.length;
		memcpy(&msg->data[0],&msgrx.data[0],msgrx.length);
		return true;
	} else {
		return false;
	}
}
#endif

#ifdef CONFIG_HOMECAN_UDP
//Extract packet data and put into msg
void extractMsg(homecan_t *msg,uint8_t* packet,uint8_t payloadlen) {
	msg->header.priority = packet[0]>>1;
	msg->header.mode = packet[0]&0x01;
	msg->msgtype = packet[1];
	msg->address = packet[2];
	msg->channel = packet[3];
	memcpy(&msg->data[0],&(packet[4]),payloadlen-4);
	msg->length = payloadlen-4;
}

void extractBootloaderMsg(homecan_t *msg,uint8_t* packet,uint8_t payloadlen) {
	 msg->header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
	 msg->header.mode = HOMECAN_HEADER_MODE_DST;
	 msg->msgtype = HOMECAN_MSGTYPE_BOOTLOADER;
	 msg->address = packet[0];
	 msg->channel = 00;
	 msg->length = payloadlen;
	 memcpy(&msg->data[0],&(packet[0]),payloadlen);
}

bool homecan_receiveUDP(homecan_t *msg) {
	uint16_t plen;
	plen = enc28j60PacketReceive(BUFFER_SIZE_RX, rxbuf);
	packetloop_arp_icmp_tcp(rxbuf,plen);
	if (plen!=0) {
		if (rxbuf[IP_PROTO_P]==IP_PROTO_UDP_V){
			if (rxbuf[UDP_DST_PORT_H_P]==HOMECAN_UDP_PORT_BOOTLOADER>>8 && rxbuf[UDP_DST_PORT_L_P]==(HOMECAN_UDP_PORT_BOOTLOADER&0xFF) && rxbuf[UDP_LEN_L_P]-UDP_HEADER_LEN==8) {
				//BOOTLOADER port
				extractBootloaderMsg(msg,&rxbuf[UDP_DATA_P],rxbuf[UDP_LEN_L_P]-UDP_HEADER_LEN);
				return true;
			} else if (rxbuf[UDP_DST_PORT_H_P]==HOMECAN_UDP_PORT>>8 && rxbuf[UDP_DST_PORT_L_P]==(HOMECAN_UDP_PORT&0xFF)) {
				//homecan port
				extractMsg(msg,&rxbuf[UDP_DATA_P],rxbuf[UDP_LEN_L_P]-UDP_HEADER_LEN);
				return true;
			}
		}
	}
	return false;
}
#endif

bool homecan_transmit(const homecan_t *msg) {
	//priority is sending per UDP, Gateways, can only send per UDP by this function
#ifdef CONFIG_HOMECAN_UDP
	return homecan_transmitUDP(msg);
#else
#ifdef CONFIG_HOMECAN_CAN
	return homecan_transmitCAN(msg);
#endif
#endif
	return false;
}

bool homecan_receive(homecan_t *msg) {
	bool res = false;
#ifdef CONFIG_HOMECAN_CAN
	if (res==false) {
		res = homecan_receiveCAN(msg);
#ifdef CONFIG_HOMECAN_GATEWAY
		if (res==true) {
			if (msg->address==deviceID && msg->header.mode==HOMECAN_HEADER_MODE_DST) {
				return true;
			} else {
				//forward to udp
				homecan_transmitUDP(msg);
			}
		}
#endif
	}
#endif
#ifdef CONFIG_HOMECAN_UDP
	if (res==false) {
		res = homecan_receiveUDP(msg);
#ifdef CONFIG_HOMECAN_GATEWAY
		if (res==true) {
			if (msg->address==deviceID && msg->header.mode==HOMECAN_HEADER_MODE_DST) {
				return true;
			} else {
				//forward to CAN
				while (!homecan_transmitCAN(msg)) {
					_delay_ms(1);
				}
			}
		}
#endif
	}
#endif
	if (res==true) {
		//check if call for bootloader
		if (msg->address==deviceID && msg->msgtype==HOMECAN_MSGTYPE_CALL_BOOTLOADER) {
			cli();
			wdt_enable(WDTO_500MS);
			while (1);
		}
		if (msg->address==0 && msg->msgtype==HOMECAN_MSGTYPE_HEARTBEAT) {
			//we are no gateway but received a heartbeat from a gateway
			_delay_ms(deviceID);
			homecan_transmitHeartbeat();
			return false;
		}
	}
	return res;
}


void homecan_transmitHeartbeat() {
	homecan_t msg;
	msg.address = deviceID;
	msg.channel = 0;
	msg.header.priority = HOMECAN_HEADER_PRIO_DEFAULT;
	msg.header.mode = HOMECAN_HEADER_MODE_SRC;
	msg.msgtype = HOMECAN_MSGTYPE_HEARTBEAT;
	msg.length = 0;
	while (!homecan_transmit(&msg)) {
		_delay_ms(1);
	}
}

