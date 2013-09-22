#include <stdint.h>
#include "mcp4651.h"
#include "i2cmaster.h"

#define MCP4651  0x50


void mcp4651_init(void) {
	i2c_start_wait(MCP4651+I2C_WRITE);     	// set device address and write mode
   	i2c_write(0x40);	                   	// write pointer address to TCON, disable general call
   	i2c_write(0xFF);
  	i2c_stop();
}

void mcp4651_setWiper(uint8_t wiper,uint16_t value) {
	if (wiper>1) return;
	i2c_start_wait(MCP4651+I2C_WRITE);     	// set device address and write mode
   	i2c_write(((wiper&0x03)<<4)|((value&0x1FF)>>8));
   	i2c_write(value&0xFF);
  	i2c_stop();	
}

