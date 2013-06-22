/*******************************************************************************
* File Name: BOOT.c
* Version 3.30
*
* Description:
*  This file provides the source code of APIs for the I2C component.
*  Actual protocol and operation code resides in the interrupt service routine
*  file.
*
* Note:
*
*******************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "BOOT_PVT.h"


/**********************************
*      System variables
**********************************/

uint8 BOOT_initVar = 0u;    /* Defines if component was initialized */

volatile uint8 BOOT_state;  /* Current state of I2C FSM */


/*******************************************************************************
* Function Name: BOOT_Init
********************************************************************************
*
* Summary:
*  Initializes I2C registers with initial values provided from customizer.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  None
*
* Reentrant:
*  No
*
*******************************************************************************/
void BOOT_Init(void) 
{
    #if(BOOT_FF_IMPLEMENTED)
        BOOT_CFG_REG  = BOOT_DEFAULT_CFG;
        BOOT_XCFG_REG = BOOT_DEFAULT_XCFG;

        #if(CY_PSOC5A)
            BOOT_CLKDIV_REG  = LO8(BOOT_DEFAULT_DIVIDE_FACTOR);
        #else
            BOOT_CLKDIV1_REG = LO8(BOOT_DEFAULT_DIVIDE_FACTOR);
            BOOT_CLKDIV2_REG = HI8(BOOT_DEFAULT_DIVIDE_FACTOR);
        #endif /* (CY_PSOC5A) */

    #else
        uint8 enableInterrupts;

        BOOT_CFG_REG      = BOOT_DEFAULT_CFG;      /* control  */
        BOOT_INT_MASK_REG = BOOT_DEFAULT_INT_MASK; /* int_mask */

        /* Enable interrupts from block */
        enableInterrupts = CyEnterCriticalSection();
        BOOT_INT_ENABLE_REG |= BOOT_INTR_ENABLE; /* aux_ctl */
        CyExitCriticalSection(enableInterrupts);

        #if(BOOT_MODE_MASTER_ENABLED)
            BOOT_MCLK_PRD_REG = BOOT_DEFAULT_MCLK_PRD;
            BOOT_MCLK_CMP_REG = BOOT_DEFAULT_MCLK_CMP;
         #endif /* (BOOT_MODE_MASTER_ENABLED) */

        #if(BOOT_MODE_SLAVE_ENABLED)
            BOOT_PERIOD_REG = BOOT_DEFAULT_PERIOD;
        #endif  /* (BOOT_MODE_SLAVE_ENABLED) */

    #endif /* (BOOT_FF_IMPLEMENTED) */

    #if(BOOT_TIMEOUT_ENABLED)
        BOOT_TimeoutInit();
    #endif /* (BOOT_TIMEOUT_ENABLED) */

    /* Disable Interrupt and set vector and priority */
    CyIntDisable    (BOOT_ISR_NUMBER);
    CyIntSetPriority(BOOT_ISR_NUMBER, BOOT_ISR_PRIORITY);
    #if(BOOT_INTERN_I2C_INTR_HANDLER)
        (void) CyIntSetVector(BOOT_ISR_NUMBER, &BOOT_ISR);
    #endif /* (BOOT_INTERN_I2C_INTR_HANDLER) */


    /* Put state machine in idle state */
    BOOT_state = BOOT_SM_IDLE;

    #if(BOOT_MODE_SLAVE_ENABLED)
        /* Reset status and buffers index */
        BOOT_SlaveClearReadBuf();
        BOOT_SlaveClearWriteBuf();
        BOOT_slStatus = 0u; /* Reset slave status */

        /* Set default address */
        BOOT_SlaveSetAddress(BOOT_DEFAULT_ADDR);
    #endif /* (BOOT_MODE_SLAVE_ENABLED) */

    #if(BOOT_MODE_MASTER_ENABLED)
        /* Reset status and buffers index */
        BOOT_MasterClearReadBuf();
        BOOT_MasterClearWriteBuf();
        (void) BOOT_MasterClearStatus();
    #endif /* (BOOT_MODE_MASTER_ENABLED) */
}


/*******************************************************************************
* Function Name: BOOT_Enable
********************************************************************************
*
* Summary:
*  Enables I2C operations.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  None
*
*******************************************************************************/
void BOOT_Enable(void) 
{
    #if(BOOT_FF_IMPLEMENTED)
        uint8 enableInterrupts;

        /* Enable power to I2C FF block */
        enableInterrupts = CyEnterCriticalSection();
        BOOT_ACT_PWRMGR_REG  |= BOOT_ACT_PWR_EN;
        BOOT_STBY_PWRMGR_REG |= BOOT_STBY_PWR_EN;
        CyExitCriticalSection(enableInterrupts);

    #else

        #if(BOOT_MODE_SLAVE_ENABLED)
            uint8 enableInterrupts;
        #endif /* (BOOT_MODE_SLAVE_ENABLED) */

        #if(BOOT_MODE_SLAVE_ENABLED)
            /* Enable slave bit counter */
            enableInterrupts = CyEnterCriticalSection();
            BOOT_COUNTER_AUX_CTL_REG |= BOOT_CNT7_ENABLE;   /* aux_ctl */
            CyExitCriticalSection(enableInterrupts);
        #endif /* (BOOT_MODE_SLAVE_ENABLED) */

        BOOT_CFG_REG |= BOOT_ENABLE_MS;

    #endif /* (BOOT_FF_IMPLEMENTED) */

    #if(BOOT_TIMEOUT_ENABLED)
        BOOT_TimeoutEnable();
    #endif /* (BOOT_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: BOOT_Start
********************************************************************************
*
* Summary:
*  Starts the I2C hardware. Enables Active mode power template bits or clock
*  gating as appropriate. It is required to be executed before I2C bus
*  operation.
*  The I2C interrupt remains disabled after this function call.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  This component automatically enables it's interrupt.  If I2C is enabled
*  without the interrupt enabled, it could lock up the I2C bus.
*
* Global variables:
*  BOOT_initVar - used to check initial configuration, modified
*  on first function call.
*
* Reentrant:
*  No
*
*******************************************************************************/
void BOOT_Start(void) 
{
    /* Initialize I2C registers, reset I2C buffer index and clears status */
    if(0u == BOOT_initVar)
    {
        BOOT_Init();
        BOOT_initVar = 1u; /* Component initialized */
    }

    BOOT_Enable();
    BOOT_EnableInt();
}


/*******************************************************************************
* Function Name: BOOT_Stop
********************************************************************************
*
* Summary:
*  Disables I2C hardware and disables I2C interrupt. Disables Active mode power
*  template bits or clock gating as appropriate.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void BOOT_Stop(void) 
{
    #if((BOOT_FF_IMPLEMENTED)  || \
        (BOOT_UDB_IMPLEMENTED && BOOT_MODE_SLAVE_ENABLED))
        uint8 enableInterrupts;
    #endif /* ((BOOT_FF_IMPLEMENTED)  || \
               (BOOT_UDB_IMPLEMENTED && BOOT_MODE_SLAVE_ENABLED)) */

    BOOT_DisableInt();

    BOOT_DISABLE_INT_ON_STOP;   /* Interrupt on Stop can be enabled by write */
    (void) BOOT_CSR_REG;        /* Clear CSR reg */
    
    #if(BOOT_TIMEOUT_ENABLED)
        BOOT_TimeoutStop();
    #endif  /* End (BOOT_TIMEOUT_ENABLED) */

    #if(BOOT_FF_IMPLEMENTED)
        #if(CY_PSOC3 || CY_PSOC5LP)
            /* Store registers which are held in reset when Master and Slave bits are cleared */
            #if(BOOT_MODE_SLAVE_ENABLED)
                BOOT_backup.addr = BOOT_ADDR_REG;
            #endif /* (BOOT_MODE_SLAVE_ENABLED) */

            BOOT_backup.clkDiv1  = BOOT_CLKDIV1_REG;
            BOOT_backup.clkDiv2  = BOOT_CLKDIV2_REG;


            /* Reset FF block */
            BOOT_CFG_REG &= ((uint8) ~BOOT_ENABLE_MS);
            CyDelayUs(BOOT_FF_RESET_DELAY);
            BOOT_CFG_REG |= ((uint8)  BOOT_ENABLE_MS);


            /* Restore registers */
            #if(BOOT_MODE_SLAVE_ENABLED)
                BOOT_ADDR_REG = BOOT_backup.addr;
            #endif /* (BOOT_MODE_SLAVE_ENABLED) */

            BOOT_CLKDIV1_REG = BOOT_backup.clkDiv1;
            BOOT_CLKDIV2_REG = BOOT_backup.clkDiv2;

        #endif /* (CY_PSOC3 || CY_PSOC5LP) */

        /* Disable power to I2C block */
        enableInterrupts = CyEnterCriticalSection();
        BOOT_ACT_PWRMGR_REG  &= ((uint8) ~BOOT_ACT_PWR_EN);
        BOOT_STBY_PWRMGR_REG &= ((uint8) ~BOOT_STBY_PWR_EN);
        CyExitCriticalSection(enableInterrupts);

    #else

        #if(BOOT_MODE_SLAVE_ENABLED)
            /* Disable slave bit counter */
            enableInterrupts = CyEnterCriticalSection();
            BOOT_COUNTER_AUX_CTL_REG &= ((uint8) ~BOOT_CNT7_ENABLE);
            CyExitCriticalSection(enableInterrupts);
        #endif /* (BOOT_MODE_SLAVE_ENABLED) */

        BOOT_CFG_REG &= ((uint8) ~BOOT_ENABLE_MS);

    #endif /* (BOOT_FF_IMPLEMENTED) */

    BOOT_ClearPendingInt();  /* Clear interrupt triggers on reset */

    BOOT_state = BOOT_SM_IDLE;  /* Reset software FSM */
}


/* [] END OF FILE */
