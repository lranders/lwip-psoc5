/*******************************************************************************
* File Name: ETH_PM.c
* Version 2.40
*
* Description:
*  This file contains the setup, control and status commands to support
*  component operations in low power mode.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "ETH_PVT.h"

static ETH_BACKUP_STRUCT ETH_backup =
{
    ETH_DISABLED,
    ETH_BITCTR_INIT,
    #if(CY_UDB_V0)
        ETH_TX_INIT_INTERRUPTS_MASK,
        ETH_RX_INIT_INTERRUPTS_MASK
    #endif /* CY_UDB_V0 */
};


/*******************************************************************************
* Function Name: ETH_SaveConfig
********************************************************************************
*
* Summary:
*  Saves SPIM configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  ETH_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void ETH_SaveConfig(void) 
{
    /* Store Status Mask registers */
    #if(CY_UDB_V0)
       ETH_backup.cntrPeriod      = ETH_COUNTER_PERIOD_REG;
       ETH_backup.saveSrTxIntMask = ETH_TX_STATUS_MASK_REG;
       ETH_backup.saveSrRxIntMask = ETH_RX_STATUS_MASK_REG;
    #endif /* (CY_UDB_V0) */
}


/*******************************************************************************
* Function Name: ETH_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores SPIM configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  ETH_backup - used when non-retention registers are restored.
*
* Side Effects:
*  If this API is called without first calling SaveConfig then in the following
*  registers will be default values from Customizer:
*  ETH_STATUS_MASK_REG and ETH_COUNTER_PERIOD_REG.
*
*******************************************************************************/
void ETH_RestoreConfig(void) 
{
    /* Restore the data, saved by SaveConfig() function */
    #if(CY_UDB_V0)
        ETH_COUNTER_PERIOD_REG = ETH_backup.cntrPeriod;
        ETH_TX_STATUS_MASK_REG = ((uint8) ETH_backup.saveSrTxIntMask);
        ETH_RX_STATUS_MASK_REG = ((uint8) ETH_backup.saveSrRxIntMask);
    #endif /* (CY_UDB_V0) */
}


/*******************************************************************************
* Function Name: ETH_Sleep
********************************************************************************
*
* Summary:
*  Prepare SPIM Component goes to sleep.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  ETH_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void ETH_Sleep(void) 
{
    /* Save components enable state */
    ETH_backup.enableState = ((uint8) ETH_IS_ENABLED);

    ETH_Stop();
    ETH_SaveConfig();
}


/*******************************************************************************
* Function Name: ETH_Wakeup
********************************************************************************
*
* Summary:
*  Prepare SPIM Component to wake up.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  ETH_backup - used when non-retention registers are restored.
*  ETH_txBufferWrite - modified every function call - resets to
*  zero.
*  ETH_txBufferRead - modified every function call - resets to
*  zero.
*  ETH_rxBufferWrite - modified every function call - resets to
*  zero.
*  ETH_rxBufferRead - modified every function call - resets to
*  zero.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void ETH_Wakeup(void) 
{
    ETH_RestoreConfig();

    #if(ETH_RX_SOFTWARE_BUF_ENABLED)
        ETH_rxBufferFull  = 0u;
        ETH_rxBufferRead  = 0u;
        ETH_rxBufferWrite = 0u;
    #endif /* (ETH_RX_SOFTWARE_BUF_ENABLED) */

    #if(ETH_TX_SOFTWARE_BUF_ENABLED)
        ETH_txBufferFull  = 0u;
        ETH_txBufferRead  = 0u;
        ETH_txBufferWrite = 0u;
    #endif /* (ETH_TX_SOFTWARE_BUF_ENABLED) */

    /* Clear any data from the RX and TX FIFO */
    ETH_ClearFIFO();

    /* Restore components block enable state */
    if(0u != ETH_backup.enableState)
    {
        ETH_Enable();
    }
}


/* [] END OF FILE */
