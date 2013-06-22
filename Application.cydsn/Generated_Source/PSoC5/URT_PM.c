/*******************************************************************************
* File Name: URT_PM.c
* Version 2.30
*
* Description:
*  This file provides Sleep/WakeUp APIs functionality.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "URT.h"


/***************************************
* Local data allocation
***************************************/

static URT_BACKUP_STRUCT  URT_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: URT_SaveConfig
********************************************************************************
*
* Summary:
*  Saves the current user configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  URT_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void URT_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(URT_CONTROL_REG_REMOVED == 0u)
            URT_backup.cr = URT_CONTROL_REG;
        #endif /* End URT_CONTROL_REG_REMOVED */

        #if( (URT_RX_ENABLED) || (URT_HD_ENABLED) )
            URT_backup.rx_period = URT_RXBITCTR_PERIOD_REG;
            URT_backup.rx_mask = URT_RXSTATUS_MASK_REG;
            #if (URT_RXHW_ADDRESS_ENABLED)
                URT_backup.rx_addr1 = URT_RXADDRESS1_REG;
                URT_backup.rx_addr2 = URT_RXADDRESS2_REG;
            #endif /* End URT_RXHW_ADDRESS_ENABLED */
        #endif /* End URT_RX_ENABLED | URT_HD_ENABLED*/

        #if(URT_TX_ENABLED)
            #if(URT_TXCLKGEN_DP)
                URT_backup.tx_clk_ctr = URT_TXBITCLKGEN_CTR_REG;
                URT_backup.tx_clk_compl = URT_TXBITCLKTX_COMPLETE_REG;
            #else
                URT_backup.tx_period = URT_TXBITCTR_PERIOD_REG;
            #endif /*End URT_TXCLKGEN_DP */
            URT_backup.tx_mask = URT_TXSTATUS_MASK_REG;
        #endif /*End URT_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(URT_CONTROL_REG_REMOVED == 0u)
            URT_backup.cr = URT_CONTROL_REG;
        #endif /* End URT_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: URT_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  URT_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void URT_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(URT_CONTROL_REG_REMOVED == 0u)
            URT_CONTROL_REG = URT_backup.cr;
        #endif /* End URT_CONTROL_REG_REMOVED */

        #if( (URT_RX_ENABLED) || (URT_HD_ENABLED) )
            URT_RXBITCTR_PERIOD_REG = URT_backup.rx_period;
            URT_RXSTATUS_MASK_REG = URT_backup.rx_mask;
            #if (URT_RXHW_ADDRESS_ENABLED)
                URT_RXADDRESS1_REG = URT_backup.rx_addr1;
                URT_RXADDRESS2_REG = URT_backup.rx_addr2;
            #endif /* End URT_RXHW_ADDRESS_ENABLED */
        #endif  /* End (URT_RX_ENABLED) || (URT_HD_ENABLED) */

        #if(URT_TX_ENABLED)
            #if(URT_TXCLKGEN_DP)
                URT_TXBITCLKGEN_CTR_REG = URT_backup.tx_clk_ctr;
                URT_TXBITCLKTX_COMPLETE_REG = URT_backup.tx_clk_compl;
            #else
                URT_TXBITCTR_PERIOD_REG = URT_backup.tx_period;
            #endif /*End URT_TXCLKGEN_DP */
            URT_TXSTATUS_MASK_REG = URT_backup.tx_mask;
        #endif /*End URT_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(URT_CONTROL_REG_REMOVED == 0u)
            URT_CONTROL_REG = URT_backup.cr;
        #endif /* End URT_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: URT_Sleep
********************************************************************************
*
* Summary:
*  Stops and saves the user configuration. Should be called
*  just prior to entering sleep.
*
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  URT_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void URT_Sleep(void)
{

    #if(URT_RX_ENABLED || URT_HD_ENABLED)
        if((URT_RXSTATUS_ACTL_REG  & URT_INT_ENABLE) != 0u)
        {
            URT_backup.enableState = 1u;
        }
        else
        {
            URT_backup.enableState = 0u;
        }
    #else
        if((URT_TXSTATUS_ACTL_REG  & URT_INT_ENABLE) !=0u)
        {
            URT_backup.enableState = 1u;
        }
        else
        {
            URT_backup.enableState = 0u;
        }
    #endif /* End URT_RX_ENABLED || URT_HD_ENABLED*/

    URT_Stop();
    URT_SaveConfig();
}


/*******************************************************************************
* Function Name: URT_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration. Should be called
*  just after awaking from sleep.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  URT_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void URT_Wakeup(void)
{
    URT_RestoreConfig();
    #if( (URT_RX_ENABLED) || (URT_HD_ENABLED) )
        URT_ClearRxBuffer();
    #endif /* End (URT_RX_ENABLED) || (URT_HD_ENABLED) */
    #if(URT_TX_ENABLED || URT_HD_ENABLED)
        URT_ClearTxBuffer();
    #endif /* End URT_TX_ENABLED || URT_HD_ENABLED */

    if(URT_backup.enableState != 0u)
    {
        URT_Enable();
    }
}


/* [] END OF FILE */
