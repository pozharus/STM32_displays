/*
 * Inel_SVI.h
 *
 *  Created on: Apr 22, 2021
 *      Author: oem
 */

#define Inel_SVI_LEDS 0x60000001

#define Inel_SVI_SEGMENTS 0x60000002
#define Inel_SVI_ANODES 0x60000003

uint8_t anodes[4] = {8, 4, 2, 1};
char Inel_SVI_Numbrs[10] =
{
		0x3F,	0x06,	0x5B,	0x4F,	0x66,
		0x6D,	0x7D,	0x07,	0x7F,	0x6F
};

static uint32_t LM_COL0 = 0x60000010;

