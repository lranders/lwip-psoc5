/*******************************************************************************
* File Name: BOOT_PM.c
* Version 3.30
*
* Description:
*  This file provides Low power mode APIs for I2C component.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "BOOT_PVT.h"

BOOT_BACKUP_STRUCT BOOT_backup =
{
    BOOT_DISABLE, /* enableState */

    #if(BOOT_FF_IMPLEMENTED)
        BOOT_DEFAULT_XCFG,  /* xcfg */
        BOOT_DEFAULT_CFG,   /* cfg  */

        #if(BOOT_MODE_SLAVE_ENABLED)
            BOOT_DEFAULT_ADDR, /* addr */
        #endif /* (BOOT_MODE_SLAVE_ENABLED) */

        #if(CY_PSOC5A)
            LO8(BOOT_DEFAULT_DIVIDE_FACTOR),  /* div */
        #else
            LO8(BOOT_DEFAULT_DIVIDE_FACTOR), /* div1 */
            HI8(BOOT_DEFAULT_DIVIDE_FACTOR), /* div2 */
        #endif /* (CY_PSOC5A) */

    #else  /* (BOOT_UDB_IMPLEMENTED) */
        BOOT_DEFAULT_CFG,    /* control */

        #if(CY_UDB_V0)
            BOOT_INT_ENABLE_MASK, /* aux_ctl */

            #if(BOOT_MODE_SLAVE_ENABLED)
                BOOT_DEFAULT_ADDR, /* addr_d0 */
            #endif /* (BOOT_MODE_SLAVE_ENABLED) */
        #endif /* (CY_UDB_V0) */
    #endif /* (BOOT_FF_IMPLEMENTED) */

    #if(BOOT_TIMEOUT_ENABLED)
        BOOT_DEFAULT_TMOUT_PERIOD,
        BOOT_DEFAULT_TMOUT_INTR_MASK,

        #if(BOOT_TIMEOUT_PRESCALER_ENABLED && CY_UDB_V0)
            BOOT_DEFAULT_TMOUT_PRESCALER_PRD,
        #endif /* (BOOT_TIMEOUT_PRESCALER_ENABLED) */

    #endif /* (BOOT_TIMEOUT_ENABLED) */
};

#if((BOOT_FF_IMPLEMENTED) && (BOOT_WAKEUP_ENABLED))
    volatile uint8 BOOT_wakeupSource;
#endif /* ((BOOT_FF_IMPLEMENTED) && (BOOT_WAKEUP_ENABLED)) */


/*******************************************************************************
* Function Name: BOOT_SaveConfig
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: disables I2C Master(if was enabled before go
*  to sleep), enables I2C backup regulator. Waits while on-going transaction be
*  will completed and I2C will be ready go to sleep. All incoming transaction
*  will be NACKed till power down will be asserted. The address match event
*  wakes up the chip.
*  Wakeup on address match disabled: saves I2C configuration and non-retention
*  register values.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global Variables:
*  BOOT_backup - used to save component configuration and
*       none-retention registers before enter sleep mode.
*
* Reentrant:
*  No
*
*******************************************************************************/
void BOOT_SaveConfig(void) 
{
    #if(BOOT_FF_IMPLEMENTED)
        #if(BOOT_WAKEUP_ENABLED)
            uint8 enableInterrupts;
        #endif /* (BOOT_WAKEUP_ENABLED) */

        /* Store regiters in either Sleep mode */
        BOOT_backup.cfg  = BOOT_CFG_REG;
        BOOT_backup.xcfg = BOOT_XCFG_REG;

        #if(BOOT_MODE_SLAVE_ENABLED)
            BOOT_backup.addr = BOOT_ADDR_REG;
        #endif /* (BOOT_MODE_SLAVE_ENABLED) */

        #if(CY_PSOC5A)
            BOOT_backup.clkDiv   = BOOT_CLKDIV_REG;
        #else
            BOOT_backup.clkDiv1  = BOOT_CLKDIV1_REG;
            BOOT_backup.clkDiv2  = BOOT_CLKDIV2_REG;
        #endif /* (CY_PSOC5A) */

        #if(BOOT_WAKEUP_ENABLED)
            /* Need to disable Master */
            BOOT_CFG_REG &= ((uint8) ~BOOT_ENABLE_MASTER);

            /* Enable the I2C regulator backup */
            enableInterrupts = CyEnterCriticalSection();
            BOOT_PWRSYS_CR1_REG |= BOOT_PWRSYS_CR1_I2C_REG_BACKUP;
            CyExitCriticalSection(enableInterrupts);

            /* 1) Set force NACK to ignore I2C transactions
               2) Wait while I2C will be ready go to Sleep
               3) These bits are cleared on wake up */
            BOOT_XCFG_REG |= BOOT_XCFG_FORCE_NACK;
            while(0u == (BOOT_XCFG_REG & BOOT_XCFG_RDY_TO_SLEEP))
            {
                ; /* Wait when block is ready to Sleep */
            }

            /* Setup wakeup interrupt */
            BOOT_DisableInt();
            (void) CyIntSetVector(BOOT_ISR_NUMBER, &BOOT_WAKEUP_ISR);
            BOOT_wakeupSource = 0u;
            BOOT_EnableInt();

        #endif /* (BOOT_WAKEUP_ENABLED) */

    #else
        /* Store only address match bit */
        BOOT_backup.control = (BOOT_CFG_REG & BOOT_CTRL_ANY_ADDRESS_MASK);

        #if(CY_UDB_V0)
            /* Store interrupt mask bits */
            BOOT_backup.intMask = BOOT_INT_MASK_REG;

            #if(BOOT_MODE & BOOT_MODE_SLAVE)
                BOOT_backup.addr = BOOT_ADDR_REG;
            #endif /* (BOOT_MODE & BOOT_MODE_SLAVE) */

        #endif /* (CY_UDB_V0) */

    #endif /* (BOOT_FF_IMPLEMENTED) */

    #if(BOOT_TIMEOUT_ENABLED)
        BOOT_TimeoutSaveConfig();   /* Save Timeout config */
    #endif /* (BOOT_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: BOOT_Sleep
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: All incoming transaction will be NACKed till
*  power down will be asserted. The address match event wakes up the chip.
*  Wakeup on address match disabled: Disables active mode power template bits or
*  clock gating as appropriate. Saves I2C configuration and non-retention
*  register values.
*  Disables I2C interrupt.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Reentrant:
*  No
*
*******************************************************************************/
void BOOT_Sleep(void) 
{
    #if(BOOT_WAKEUP_ENABLED)
        /* The I2C block should be always enabled if used as wakeup source */
        BOOT_backup.enableState = BOOT_DISABLE;

        #if(BOOT_TIMEOUT_ENABLED)
            BOOT_TimeoutStop();
        #endif /* (BOOT_TIMEOUT_ENABLED) */

    #else

        BOOT_backup.enableState = ((uint8) BOOT_IS_ENABLED);

        if(BOOT_IS_ENABLED)
        {
            BOOT_Stop();
        }
    #endif /* (BOOT_WAKEUP_ENABLED) */

    BOOT_SaveConfig();
}


/*******************************************************************************
* Function Name: BOOT_RestoreConfig
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: enables I2C Master (if was enabled before go
*  to sleep), disables I2C backup regulator.
*  Wakeup on address match disabled: Restores I2C configuration and
*  non-retention register values.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global Variables:
*  BOOT_backup - used to save component configuration and
*  none-retention registers before exit sleep mode.
*
*******************************************************************************/
void BOOT_RestoreConfig(void) 
{
    #if(BOOT_FF_IMPLEMENTED)
        uint8 enableInterrupts;

        if(BOOT_CHECK_PWRSYS_I2C_BACKUP)    /* Enabled if was in Sleep */
        {
            /* Disable back-up regulator */
            enableInterrupts = CyEnterCriticalSection();
            BOOT_PWRSYS_CR1_REG &= ((uint8) ~BOOT_PWRSYS_CR1_I2C_REG_BACKUP);
            CyExitCriticalSection(enableInterrupts);

            /* Re-enable Master */
            BOOT_CFG_REG = BOOT_backup.cfg;
        }
        else /* The I2C_REG_BACKUP was cleaned by PM API: it means Hibernate or wake-up not set */
        {
            #if(BOOT_WAKEUP_ENABLED)
                /* Disable power to I2C block before register restore */
                enableInterrupts = CyEnterCriticalSection();
                BOOT_ACT_PWRMGR_REG  &= ((uint8) ~BOOT_ACT_PWR_EN);
                BOOT_STBY_PWRMGR_REG &= ((uint8) ~BOOT_STBY_PWR_EN);
                CyExitCriticalSection(enableInterrupts);

                /* Enable component after restore complete */
                BOOT_backup.enableState = BOOT_ENABLE;
            #endif /* (BOOT_WAKEUP_ENABLED) */

            /* Restore component registers: Hibernate disable power */
            BOOT_XCFG_REG = BOOT_backup.xcfg;
            BOOT_CFG_REG  = BOOT_backup.cfg;

            #if(BOOT_MODE_SLAVE_ENABLED)
                BOOT_ADDR_REG = BOOT_backup.addr;
            #endif /* (BOOT_MODE_SLAVE_ENABLED) */

            #if(CY_PSOC5A)
                BOOT_CLKDIV_REG  = BOOT_backup.clkDiv;
            #else
                BOOT_CLKDIV1_REG = BOOT_backup.clkDiv1;
                BOOT_CLKDIV2_REG = BOOT_backup.clkDiv2;
            #endif /* (CY_PSOC5A) */
        }

        #if(BOOT_WAKEUP_ENABLED)
            BOOT_DisableInt();
            (void) CyIntSetVector(BOOT_ISR_NUMBER, &BOOT_ISR);
            if(0u != BOOT_wakeupSource)
            {
                BOOT_SetPendingInt();   /* Generate interrupt to process incomming transcation */
            }
            BOOT_EnableInt();
        #endif /* (BOOT_WAKEUP_ENABLED) */

    #else

        #if(CY_UDB_V0)
            uint8 enableInterrupts;

            BOOT_INT_MASK_REG |= BOOT_backup.intMask;

            enableInterrupts = CyEnterCriticalSection();
            BOOT_INT_ENABLE_REG |= BOOT_INT_ENABLE_MASK;
            CyExitCriticalSection(enableInterrupts);

            #if(BOOT_MODE_MASTER_ENABLED)
                /* Restore Master Clock generator */
                BOOT_MCLK_PRD_REG = BOOT_DEFAULT_MCLK_PRD;
                BOOT_MCLK_CMP_REG = BOOT_DEFAULT_MCLK_CMP;
            #endif /* (BOOT_MODE_MASTER_ENABLED) */

            #if(BOOT_MODE_SLAVE_ENABLED)
                BOOT_ADDR_REG = BOOT_backup.addr;

                /* Restore slave bit counter period */
                BOOT_PERIOD_REG = BOOT_DEFAULT_PERIOD;
            #endif /* (BOOT_MODE_SLAVE_ENABLED) */

        #endif /* (CY_UDB_V0) */

        BOOT_CFG_REG = BOOT_backup.control;

    #endif /* (BOOT_FF_IMPLEMENTED) */

    #if(BOOT_TIMEOUT_ENABLED)
        BOOT_TimeoutRestoreConfig();
    #endif /* (BOOT_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: BOOT_Wakeup
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: enables I2C Master (if was enabled before go
*  to sleep) and disables I2C backup regulator.
*  Wakeup on address match disabled: Restores I2C configuration and
*  non-retention register values. Restores Active mode power template bits or
*  clock gating as appropriate.
*  The I2C interrupt remains disabled after function call.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Reentrant:
*  No
*
*******************************************************************************/
void BOOT_Wakeup(void) 
{
    BOOT_RestoreConfig();   /* Restore I2C register settings */

    /* Restore component enable state */
    if(0u != BOOT_backup.enableState)
    {
        BOOT_Enable();
        BOOT_EnableInt();
    }
    else
    {
        #if(BOOT_TIMEOUT_ENABLED)
            BOOT_TimeoutEnable();
        #endif /* (BOOT_TIMEOUT_ENABLED) */
    }
}


/* [] END OF FILE */
