/*******************************************************************************
* File Name: .h
* Version 3.30
*
* Description:
*  This file provides private constants and parameter values for the I2C
*  component.
*
* Note:
*
********************************************************************************
* Copyright 2012, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_I2C_PVT_BOOT_H)
#define CY_I2C_PVT_BOOT_H

#include "BOOT.h"

#define BOOT_TIMEOUT_ENABLED_INC    (0u)
#if(0u != BOOT_TIMEOUT_ENABLED_INC)
    #include "BOOT_TMOUT.h"
#endif /* (0u != BOOT_TIMEOUT_ENABLED_INC) */


/**********************************
*   Variables with external linkage
**********************************/

extern BOOT_BACKUP_STRUCT BOOT_backup;

extern volatile uint8 BOOT_state;   /* Current state of I2C FSM */

/* Master variables */
#if(BOOT_MODE_MASTER_ENABLED)
    extern volatile uint8 BOOT_mstrStatus;   /* Master Status byte  */
    extern volatile uint8 BOOT_mstrControl;  /* Master Control byte */

    /* Transmit buffer variables */
    extern volatile uint8 * BOOT_mstrRdBufPtr;   /* Pointer to Master Read buffer */
    extern volatile uint8   BOOT_mstrRdBufSize;  /* Master Read buffer size       */
    extern volatile uint8   BOOT_mstrRdBufIndex; /* Master Read buffer Index      */

    /* Receive buffer variables */
    extern volatile uint8 * BOOT_mstrWrBufPtr;   /* Pointer to Master Write buffer */
    extern volatile uint8   BOOT_mstrWrBufSize;  /* Master Write buffer size       */
    extern volatile uint8   BOOT_mstrWrBufIndex; /* Master Write buffer Index      */

#endif /* (BOOT_MODE_MASTER_ENABLED) */

/* Slave variables */
#if(BOOT_MODE_SLAVE_ENABLED)
    extern volatile uint8 BOOT_slStatus;         /* Slave Status  */

    /* Transmit buffer variables */
    extern volatile uint8 * BOOT_slRdBufPtr;     /* Pointer to Transmit buffer  */
    extern volatile uint8   BOOT_slRdBufSize;    /* Slave Transmit buffer size  */
    extern volatile uint8   BOOT_slRdBufIndex;   /* Slave Transmit buffer Index */

    /* Receive buffer variables */
    extern volatile uint8 * BOOT_slWrBufPtr;     /* Pointer to Receive buffer  */
    extern volatile uint8   BOOT_slWrBufSize;    /* Slave Receive buffer size  */
    extern volatile uint8   BOOT_slWrBufIndex;   /* Slave Receive buffer Index */

    #if(BOOT_SW_ADRR_DECODE)
        extern volatile uint8 BOOT_slAddress;     /* Software address variable */
    #endif   /* (BOOT_SW_ADRR_DECODE) */

#endif /* (BOOT_MODE_SLAVE_ENABLED) */

#if((BOOT_FF_IMPLEMENTED) && (BOOT_WAKEUP_ENABLED))
    extern volatile uint8 BOOT_wakeupSource;
#endif /* ((BOOT_FF_IMPLEMENTED) && (BOOT_WAKEUP_ENABLED)) */


#endif /* CY_I2C_PVT_BOOT_H */


/* [] END OF FILE */
