/*******************************************************************************
* File Name: BLA.c
* Version 1.10
*
*  Description:
*   Provides an API for the Bootloadable application. The API includes a
*   single function for starting bootloader.
*
********************************************************************************
* Copyright 2008-2013, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "BLA.h"


/*******************************************************************************
* Function Name: BLA_Load
********************************************************************************
* Summary:
*  Begins the bootloading algorithm, downloading a new ACD image from the host.
*
* Parameters:
*  None
*
* Returns:
*  This method will never return. It will load a new application and reset
*  the device.
*
*******************************************************************************/
void BLA_Load(void) 
{
    /* Schedule Bootloader to start after reset */
    BLA_SET_RUN_TYPE(BLA_START_BTLDR);

    /* Reset device */
    BLA_SOFTWARE_RESET;
}


/*******************************************************************************
* Function Name: BLA_SetFlashByte
********************************************************************************
* Summary:
*  Sets byte at specified address in Flash.
*
* Parameters:  
*  None
*
* Returns:
*  None
*
*******************************************************************************/
void BLA_SetFlashByte(uint32 address, uint8 runType) 
{
    uint32 flsAddr = address - CYDEV_FLASH_BASE;
    uint8 rowData[CYDEV_FLS_ROW_SIZE];

    #if !(CY_PSOC4)
        uint8 arrayId = (flsAddr / CYDEV_FLS_SECTOR_SIZE);
    #endif  /* !(CY_PSOC4) */

    uint16 rowNum = ((flsAddr % CYDEV_FLS_SECTOR_SIZE) / CYDEV_FLS_ROW_SIZE);
    uint32 baseAddr = address - (address % CYDEV_FLS_ROW_SIZE);
    uint16 idx;


    for (idx = 0; idx < CYDEV_FLS_ROW_SIZE; idx++)
    {
        rowData[idx] = BLA_GET_CODE_DATA(baseAddr + idx);
    }
    rowData[address % CYDEV_FLS_ROW_SIZE] = runType;

    #if(CY_PSOC4)
        CySysFlashWriteRow(rowNum, rowData);
    #else
        CyWriteRowData(arrayId, rowNum, rowData);
    #endif  /* (CY_PSOC4) */
}


/* [] END OF FILE */
