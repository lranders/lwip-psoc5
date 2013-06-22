/*******************************************************************************
* File Name: BOOT_INT.c
* Version 3.30
*
* Description:
*  This file provides the source code of Interrupt Service Routine (ISR)
*  for I2C component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "BOOT_PVT.h"


/*******************************************************************************
*  Place your includes, defines and code here
********************************************************************************/
/* `#START BOOT_ISR_intc` */

/* `#END` */


/*******************************************************************************
* Function Name: BOOT_ISR
********************************************************************************
*
* Summary:
*  Handler for I2C interrupt. The Slave and Master operations are handled here.
*
* Parameters:
*  void
*
* Return:
*  void
*
* Reentrant:
*  No
*
*******************************************************************************/
CY_ISR(BOOT_ISR)
{
    #if(BOOT_MODE_SLAVE_ENABLED)
       uint8  tmp8;
    #endif  /* (BOOT_MODE_SLAVE_ENABLED) */

    uint8  tmpCsr;

    #if(BOOT_TIMEOUT_FF_ENABLED)
        if(0u != BOOT_TimeoutGetStatus())
        {
            BOOT_TimeoutReset();
            BOOT_state = BOOT_SM_EXIT_IDLE;
            /* BOOT_CSR_REG should be cleared after reset */
        }
    #endif /* (BOOT_TIMEOUT_FF_ENABLED) */


    tmpCsr = BOOT_CSR_REG;      /* Make copy as interrupts clear */

    #if(BOOT_MODE_MULTI_MASTER_SLAVE_ENABLED)
        if(BOOT_CHECK_START_GEN(BOOT_MCSR_REG))
        {
            BOOT_CLEAR_START_GEN;

            /* Set READ complete, but was aborted */
            BOOT_mstrStatus |= (BOOT_MSTAT_ERR_XFER |
                                            BOOT_GET_MSTAT_CMPLT);

            /* The slave was addressed */
            BOOT_state = BOOT_SM_SLAVE;
        }
    #endif /* (BOOT_MODE_MULTI_MASTER_SLAVE_ENABLED) */


    #if(BOOT_MODE_MULTI_MASTER_ENABLED)
        if(BOOT_CHECK_LOST_ARB(tmpCsr))
        {
            /* Set errors */
            BOOT_mstrStatus |= (BOOT_MSTAT_ERR_XFER     |
                                            BOOT_MSTAT_ERR_ARB_LOST |
                                            BOOT_GET_MSTAT_CMPLT);

            BOOT_DISABLE_INT_ON_STOP; /* Interrupt on Stop is enabled by write */

            #if(BOOT_MODE_MULTI_MASTER_SLAVE_ENABLED)
                if(BOOT_CHECK_ADDRESS_STS(tmpCsr))
                {
                    /* The slave was addressed */
                    BOOT_state = BOOT_SM_SLAVE;
                }
                else
                {
                    BOOT_BUS_RELEASE;

                    BOOT_state = BOOT_SM_EXIT_IDLE;
                }
            #else
                BOOT_BUS_RELEASE;

                BOOT_state = BOOT_SM_EXIT_IDLE;

            #endif /* (BOOT_MODE_MULTI_MASTER_SLAVE_ENABLED) */
        }
    #endif /* (BOOT_MODE_MULTI_MASTER_ENABLED) */

    /* Check for Master operation mode */
    if(BOOT_CHECK_SM_MASTER)
    {
        #if(BOOT_MODE_MASTER_ENABLED)
            if(BOOT_CHECK_BYTE_COMPLETE(tmpCsr))
            {
                switch (BOOT_state)
                {
                case BOOT_SM_MSTR_WR_ADDR:  /* After address is sent, WRITE data */
                case BOOT_SM_MSTR_RD_ADDR:  /* After address is sent, READ  data */

                    tmpCsr &= ((uint8) ~BOOT_CSR_STOP_STATUS); /* Clear STOP bit history on address phase */
                    
                    if(BOOT_CHECK_ADDR_ACK(tmpCsr))
                    {
                        /* Setup for transmit or receive of data */
                        if(BOOT_state == BOOT_SM_MSTR_WR_ADDR)   /* TRANSMIT data */
                        {
                            /* Check if at least one byte to transfer */
                            if(BOOT_mstrWrBufSize > 0u)
                            {
                                /* Load the 1st data byte */
                                BOOT_DATA_REG = BOOT_mstrWrBufPtr[0u];
                                BOOT_TRANSMIT_DATA;
                                BOOT_mstrWrBufIndex = 1u;   /* Set index to 2nd element */

                                /* Set transmit state until done */
                                BOOT_state = BOOT_SM_MSTR_WR_DATA;
                            }
                            /* End of buffer: complete writing */
                            else if(BOOT_CHECK_NO_STOP(BOOT_mstrControl))
                            {
                                #if(CY_PSOC5A)
                                    /* Do not handles 0 bytes transfer - HALT is NOT allowed */
                                    BOOT_ENABLE_INT_ON_STOP;
                                    BOOT_GENERATE_STOP;
                                
                                #else
                                    /* Set WRITE complete and Master HALTED */
                                    BOOT_mstrStatus |= (BOOT_MSTAT_XFER_HALT |
                                                                    BOOT_MSTAT_WR_CMPLT);

                                    BOOT_state = BOOT_SM_MSTR_HALT; /* Expect RESTART */
                                    BOOT_DisableInt();
                                
                                #endif /* (CY_PSOC5A) */
                            }
                            else
                            {
                                BOOT_ENABLE_INT_ON_STOP; /* Enable interrupt on STOP, to catch it */
                                BOOT_GENERATE_STOP;
                            }
                        }
                        else  /* Master Receive data */
                        {
                            BOOT_READY_TO_READ; /* Release bus to read data */

                            BOOT_state  = BOOT_SM_MSTR_RD_DATA;
                        }
                    }
                    /* Address is NACKed */
                    else if(BOOT_CHECK_ADDR_NAK(tmpCsr))
                    {
                        /* Set Address NAK error */
                        BOOT_mstrStatus |= (BOOT_MSTAT_ERR_XFER |
                                                        BOOT_MSTAT_ERR_ADDR_NAK);
                                                        
                        if(BOOT_CHECK_NO_STOP(BOOT_mstrControl))
                        {
                            BOOT_mstrStatus |= (BOOT_MSTAT_XFER_HALT | 
                                                            BOOT_GET_MSTAT_CMPLT);

                            BOOT_state = BOOT_SM_MSTR_HALT; /* Expect RESTART */
                            BOOT_DisableInt();
                        }
                        else  /* Do normal Stop */
                        {
                            BOOT_ENABLE_INT_ON_STOP; /* Enable interrupt on STOP, to catch it */
                            BOOT_GENERATE_STOP;
                        }
                    }
                    else
                    {
                        /* Address phase is not set for some reason: error */
                        #if(BOOT_TIMEOUT_ENABLED)
                            /* Exit from interrupt to take a chance for timeout timer handle this case */
                            BOOT_DisableInt();
                            BOOT_ClearPendingInt();
                        #else
                            /* Block execution flow: unexpected condition */
                            CYASSERT(0u != 0u);
                        #endif /* (BOOT_TIMEOUT_ENABLED) */
                    }
                    break;

                case BOOT_SM_MSTR_WR_DATA:

                    if(BOOT_CHECK_DATA_ACK(tmpCsr))
                    {
                        /* Check if end of buffer */
                        if(BOOT_mstrWrBufIndex  < BOOT_mstrWrBufSize)
                        {
                            BOOT_DATA_REG =
                                                     BOOT_mstrWrBufPtr[BOOT_mstrWrBufIndex];
                            BOOT_TRANSMIT_DATA;
                            BOOT_mstrWrBufIndex++;
                        }
                        /* End of buffer: complete writing */
                        else if(BOOT_CHECK_NO_STOP(BOOT_mstrControl))
                        {
                            /* Set WRITE complete and Master HALTED */
                            BOOT_mstrStatus |= (BOOT_MSTAT_XFER_HALT |
                                                            BOOT_MSTAT_WR_CMPLT);

                            BOOT_state = BOOT_SM_MSTR_HALT;    /* Expect RESTART */
                            BOOT_DisableInt();
                        }
                        else  /* Do normal STOP */
                        {
                            BOOT_Workaround();          /* Workaround: empty function */
                            BOOT_ENABLE_INT_ON_STOP;    /* Enable interrupt on STOP, to catch it */
                            BOOT_GENERATE_STOP;
                        }
                    }
                    /* Last byte NAKed: end writing */
                    else if(BOOT_CHECK_NO_STOP(BOOT_mstrControl))
                    {
                        /* Set WRITE complete, SHORT transfer and Master HALTED */
                        BOOT_mstrStatus |= (BOOT_MSTAT_ERR_XFER       |
                                                        BOOT_MSTAT_ERR_SHORT_XFER |
                                                        BOOT_MSTAT_XFER_HALT      |
                                                        BOOT_MSTAT_WR_CMPLT);

                        BOOT_state = BOOT_SM_MSTR_HALT;    /* Expect RESTART */
                        BOOT_DisableInt();
                    }
                    else  /* Do normal STOP */
                    {
                        BOOT_ENABLE_INT_ON_STOP;    /* Enable interrupt on STOP, to catch it */
                        BOOT_GENERATE_STOP;

                        /* Set SHORT and ERR transfer */
                        BOOT_mstrStatus |= (BOOT_MSTAT_ERR_SHORT_XFER |
                                                        BOOT_MSTAT_ERR_XFER);
                    }
                    
                    break;

                case BOOT_SM_MSTR_RD_DATA:

                    BOOT_mstrRdBufPtr[BOOT_mstrRdBufIndex] = BOOT_DATA_REG;
                    BOOT_mstrRdBufIndex++;

                    /* Check if end of buffer */
                    if(BOOT_mstrRdBufIndex < BOOT_mstrRdBufSize)
                    {
                        BOOT_ACK_AND_RECEIVE;       /* ACK and receive byte */
                    }
                    /* End of buffer: complete reading */
                    else if(BOOT_CHECK_NO_STOP(BOOT_mstrControl))
                    {                        
                        /* Set READ complete and Master HALTED */
                        BOOT_mstrStatus |= (BOOT_MSTAT_XFER_HALT |
                                                        BOOT_MSTAT_RD_CMPLT);
                        
                        BOOT_state = BOOT_SM_MSTR_HALT;    /* Expect RESTART */
                        BOOT_DisableInt();
                    }
                    else
                    {
                        BOOT_ENABLE_INT_ON_STOP;
                        BOOT_NAK_AND_RECEIVE;       /* NACK and TRY to generate STOP */
                    }
                    break;

                default: /* This is an invalid state and should not occur */

                    #if(BOOT_TIMEOUT_ENABLED)
                        /* Exit from interrupt to take a chance for timeout timer handle this case */
                        BOOT_DisableInt();
                        BOOT_ClearPendingInt();
                    #else
                        /* Block execution flow: unexpected condition */
                        CYASSERT(0u != 0u);
                    #endif /* (BOOT_TIMEOUT_ENABLED) */

                    break;
                }
            }

            /* Catches the Stop: end of transaction */
            if(BOOT_CHECK_STOP_STS(tmpCsr))
            {
                BOOT_mstrStatus |= BOOT_GET_MSTAT_CMPLT;

                BOOT_DISABLE_INT_ON_STOP;
                BOOT_state = BOOT_SM_IDLE;
            }
        #endif /* (BOOT_MODE_MASTER_ENABLED) */
    }
    else if(BOOT_CHECK_SM_SLAVE)
    {
        #if(BOOT_MODE_SLAVE_ENABLED)
            
            if((BOOT_CHECK_STOP_STS(tmpCsr)) || /* Stop || Restart */
               (BOOT_CHECK_BYTE_COMPLETE(tmpCsr) && BOOT_CHECK_ADDRESS_STS(tmpCsr)))
            {
                /* Catch end of master write transcation: use interrupt on Stop */
                /* The STOP bit history on address phase does not have correct state */
                if(BOOT_SM_SL_WR_DATA == BOOT_state)
                {
                    BOOT_DISABLE_INT_ON_STOP;

                    BOOT_slStatus &= ((uint8) ~BOOT_SSTAT_WR_BUSY);
                    BOOT_slStatus |= ((uint8)  BOOT_SSTAT_WR_CMPLT);

                    BOOT_state = BOOT_SM_IDLE;
                }
            }

            if(BOOT_CHECK_BYTE_COMPLETE(tmpCsr))
            {
                /* The address only issued after Start or ReStart: so check address
                   to catch this events:
                    FF : sets Addr phase with byte_complete interrupt trigger.
                    UDB: sets Addr phase immediately after Start or ReStart. */
                if(BOOT_CHECK_ADDRESS_STS(tmpCsr))
                {
                    /* Check for software address detection */
                    #if(BOOT_SW_ADRR_DECODE)
                        tmp8 = BOOT_GET_SLAVE_ADDR(BOOT_DATA_REG);

                        if(tmp8 == BOOT_slAddress)   /* Check for address match */
                        {
                            if(0u != (BOOT_DATA_REG & BOOT_READ_FLAG))
                            {
                                /* Place code to prepare read buffer here                  */
                                /* `#START BOOT_SW_PREPARE_READ_BUF_interrupt` */

                                /* `#END` */

                                /* Prepare next opeation to read, get data and place in data register */
                                if(BOOT_slRdBufIndex < BOOT_slRdBufSize)
                                {
                                    /* Load first data byte from array */
                                    BOOT_DATA_REG = BOOT_slRdBufPtr[BOOT_slRdBufIndex];
                                    BOOT_ACK_AND_TRANSMIT;
                                    BOOT_slRdBufIndex++;

                                    BOOT_slStatus |= BOOT_SSTAT_RD_BUSY;
                                }
                                else    /* Overflow: provide 0xFF on the bus */
                                {
                                    BOOT_DATA_REG = BOOT_OVERFLOW_RETURN;
                                    BOOT_ACK_AND_TRANSMIT;

                                    BOOT_slStatus  |= (BOOT_SSTAT_RD_BUSY |
                                                                   BOOT_SSTAT_RD_ERR_OVFL);
                                }

                                BOOT_state = BOOT_SM_SL_RD_DATA;
                            }
                            else  /* Write transaction: receive 1st byte */
                            {
                                BOOT_ACK_AND_RECEIVE;
                                BOOT_state = BOOT_SM_SL_WR_DATA;

                                BOOT_slStatus |= BOOT_SSTAT_WR_BUSY;
                                BOOT_ENABLE_INT_ON_STOP;
                            }
                        }    
                        else
                        {
                            /*     Place code to compare for additional address here    */
                            /* `#START BOOT_SW_ADDR_COMPARE_interruptStart` */

                            /* `#END` */
                            
                            BOOT_NAK_AND_RECEIVE;   /* NACK address */

                            /* Place code to end of condition for NACK generation here */
                            /* `#START BOOT_SW_ADDR_COMPARE_interruptEnd`  */

                            /* `#END` */
                        }
                        
                    #else /* (BOOT_HW_ADRR_DECODE) */
                        
                        if(0u != (BOOT_DATA_REG & BOOT_READ_FLAG))
                        {
                            /* Place code to prepare read buffer here                  */
                            /* `#START BOOT_HW_PREPARE_READ_BUF_interrupt` */

                            /* `#END` */

                            /* Prepare next opeation to read, get data and place in data register */
                            if(BOOT_slRdBufIndex < BOOT_slRdBufSize)
                            {
                                /* Load first data byte from array */
                                BOOT_DATA_REG = BOOT_slRdBufPtr[BOOT_slRdBufIndex];
                                BOOT_ACK_AND_TRANSMIT;
                                BOOT_slRdBufIndex++;

                                BOOT_slStatus |= BOOT_SSTAT_RD_BUSY;
                            }
                            else    /* Overflow: provide 0xFF on the bus */
                            {
                                BOOT_DATA_REG = BOOT_OVERFLOW_RETURN;
                                BOOT_ACK_AND_TRANSMIT;

                                BOOT_slStatus  |= (BOOT_SSTAT_RD_BUSY |
                                                               BOOT_SSTAT_RD_ERR_OVFL);
                            }

                            BOOT_state = BOOT_SM_SL_RD_DATA;
                        }
                        else  /* Write transaction: receive 1st byte */
                        {
                            BOOT_ACK_AND_RECEIVE;
                            BOOT_state = BOOT_SM_SL_WR_DATA;

                            BOOT_slStatus |= BOOT_SSTAT_WR_BUSY;
                            BOOT_ENABLE_INT_ON_STOP;
                        }
                        
                    #endif /* (BOOT_SW_ADRR_DECODE) */
                }
                /* Data states */
                /* Data master writes into slave */
                else if(BOOT_state == BOOT_SM_SL_WR_DATA)
                {
                    if(BOOT_slWrBufIndex < BOOT_slWrBufSize)
                    {
                        tmp8 = BOOT_DATA_REG;
                        BOOT_ACK_AND_RECEIVE;
                        BOOT_slWrBufPtr[BOOT_slWrBufIndex] = tmp8;
                        BOOT_slWrBufIndex++;
                    }
                    else  /* of array: complete write, send NACK */
                    {
                        BOOT_NAK_AND_RECEIVE;

                        BOOT_slStatus |= BOOT_SSTAT_WR_ERR_OVFL;
                    }
                }
                /* Data master reads from slave */
                else if(BOOT_state == BOOT_SM_SL_RD_DATA)
                {
                    if(BOOT_CHECK_DATA_ACK(tmpCsr))
                    {
                        if(BOOT_slRdBufIndex < BOOT_slRdBufSize)
                        {
                             /* Get data from array */
                            BOOT_DATA_REG = BOOT_slRdBufPtr[BOOT_slRdBufIndex];
                            BOOT_TRANSMIT_DATA;
                            BOOT_slRdBufIndex++;
                        }
                        else   /* Overflow: provide 0xFF on the bus */
                        {
                            BOOT_DATA_REG = BOOT_OVERFLOW_RETURN;
                            BOOT_TRANSMIT_DATA;

                            BOOT_slStatus |= BOOT_SSTAT_RD_ERR_OVFL;
                        }
                    }
                    else  /* Last byte was NACKed: read complete */
                    {
                        /* Only NACK appears on the bus */
                        BOOT_DATA_REG = BOOT_OVERFLOW_RETURN;
                        BOOT_NAK_AND_TRANSMIT;

                        BOOT_slStatus &= ((uint8) ~BOOT_SSTAT_RD_BUSY);
                        BOOT_slStatus |= ((uint8)  BOOT_SSTAT_RD_CMPLT);

                        BOOT_state = BOOT_SM_IDLE;
                    }
                }
                else
                {
                    #if(BOOT_TIMEOUT_ENABLED)
                        /* Exit from interrupt to take a chance for timeout timer handle this case */
                        BOOT_DisableInt();
                        BOOT_ClearPendingInt();
                    #else
                        /* Block execution flow: unexpected condition */
                        CYASSERT(0u != 0u);
                    #endif /* (BOOT_TIMEOUT_ENABLED) */
                }
            }
        #endif /* (BOOT_MODE_SLAVE_ENABLED) */
    }
    else
    {
        /* The FSM skips master and slave processing: return to IDLE */
        BOOT_state = BOOT_SM_IDLE;
    }
}


#if((BOOT_FF_IMPLEMENTED) && (BOOT_WAKEUP_ENABLED))
    /*******************************************************************************
    * Function Name: BOOT_WAKEUP_ISR
    ********************************************************************************
    *
    * Summary:
    *  Empty interrupt handler to trigger after wakeup.
    *
    * Parameters:
    *  void
    *
    * Return:
    *  void
    *
    *******************************************************************************/
    CY_ISR(BOOT_WAKEUP_ISR)
    {
        BOOT_wakeupSource = 1u;  /* I2C was wakeup source */
        /* The SCL is stretched unitl the I2C_Wake() is called */
    }
#endif /* ((BOOT_FF_IMPLEMENTED) && (BOOT_WAKEUP_ENABLED))*/


/* [] END OF FILE */
