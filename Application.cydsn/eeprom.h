/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#ifndef __EEPROM_H__
#define __EEPROM_H__

typedef struct {
	unsigned char macAddress[6];
	unsigned char ipAddress[4];
	unsigned char gateWay[4];
	unsigned char netMask[4];
	union {
		unsigned char flags[2];
		struct {
			unsigned char useDhcp:1;
		} bits;
	} u;
	uint16_t checkSum;
} epmLayout_t;

#endif /* __EEPROM_H__ */

//[] END OF FILE
