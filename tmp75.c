#include "tmp75.h"
#include "i2cmaster.h"

#define TMP75  0x90

void tmp75_init(void) {
	i2c_start_wait(TMP75+I2C_WRITE);     	// set device address and write mode
	i2c_write(0x01);
	i2c_write(0x60);
	i2c_stop();
}

float tmp75_readTemperature(void) {
	uint16_t ret;
	i2c_start_wait(TMP75+I2C_WRITE);     	// set device address and write mode

   	i2c_write(0x00);                        	// write pointer address
   	i2c_rep_start(TMP75+I2C_READ);       	// set device address and read mode
	ret = i2c_readAck()<<8;                    // read one byte 
   	ret |= i2c_readNak();                    // read one byte 
  	i2c_stop();	

  	return ((float)((int16_t)ret))/256.0;
  	//return 1.234;
}

