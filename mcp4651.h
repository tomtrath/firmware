#ifndef __MCP4651_H
#define __MCP4651_H

#include <stdint.h>

#define MCP4651_WIPER1	0
#define MCP4651_WIPER2	1

void mcp4651_init(void);
void mcp4651_setWiper(uint8_t wiper,uint16_t value);

#endif
