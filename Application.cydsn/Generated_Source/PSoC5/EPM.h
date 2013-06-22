/*******************************************************************************
* File Name: EPM.h
* Version 2.10
*
* Description:
*  Provides the function definitions for the EEPROM APIs.
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_EEPROM_EPM_H)
#define CY_EEPROM_EPM_H

#include "cydevice_trm.h"
#include "CyFlash.h"

#if !defined(CY_PSOC5LP)
    #error Component EEPROM_v2_10 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */


/***************************************
*        Function Prototypes
***************************************/

#if (CY_PSOC3 || CY_PSOC5LP) 
    void EPM_Enable(void) ;
    void EPM_Start(void); 
    void EPM_Stop(void) ;
#endif /* (CY_PSOC3 || CY_PSOC5LP) */

cystatus EPM_EraseSector(uint8 sectorNumber) ;
cystatus EPM_Write(const uint8 * rowData, uint8 rowNumber) ;
cystatus EPM_StartWrite(const uint8 * rowData, uint8 rowNumber) \
            ;
cystatus EPM_QueryWrite(void) ;
cystatus EPM_ByteWrite(uint8 dataByte, uint8 rowNumber, uint8 byteNumber) \
            ;


/****************************************
*           API Constants
****************************************/

#define EPM_EEPROM_SIZE    		CYDEV_EE_SIZE
#define EPM_SPC_BYTE_WRITE_SIZE    (0x01u)


/*******************************************************************************
* Following code are OBSOLETE and must not be used starting from EEPROM 2.10
*******************************************************************************/
#define SPC_BYTE_WRITE_SIZE             (EPM_SPC_BYTE_WRITE_SIZE)

#endif /* CY_EEPROM_EPM_H */

/* [] END OF FILE */
