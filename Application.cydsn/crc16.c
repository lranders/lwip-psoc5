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
#include <stdint.h>

uint16_t CalcChecksum16(uint8_t *buffer, uint16_t size)
{
    uint16_t crc = 0xffff;
    uint16_t tmp;
    uint8_t  i;

    if(size == 0)
    {
        return(~crc);
    }

    do
    {
        for (i = 0, tmp = 0x00ff & *buffer++; i < 8; i++, tmp >>= 1)
        {
            if ((crc & 0x0001) ^ (tmp & 0x0001))
            {
                crc = (crc >> 1) ^ 0x8408;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    while(--size);

    crc = ~crc;
    tmp = crc;
    crc = (crc << 8) | (tmp >> 8 & 0xFF);

    return(crc);
}

/* [] END OF FILE */
