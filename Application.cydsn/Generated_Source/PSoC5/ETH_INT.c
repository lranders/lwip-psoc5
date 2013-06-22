/*******************************************************************************
* File Name: ETH_INT.c
* Version 2.40
*
* Description:
*  This file provides all Interrupt Service Routine (ISR) for the SPI Master
*  component.
*
* Note:
*  None.
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "ETH_PVT.h"

/* User code required at start of ISR */
/* `#START ETH_ISR_START_DEF` */

/* `#END` */


/*******************************************************************************
* Function Name: ETH_TX_ISR
********************************************************************************
*
* Summary:
*  Interrupt Service Routine for TX portion of the SPI Master.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  ETH_txBufferWrite - used for the account of the bytes which
*  have been written down in the TX software buffer.
*  ETH_txBufferRead - used for the account of the bytes which
*  have been read from the TX software buffer, modified when exist data to
*  sending and FIFO Not Full.
*  ETH_txBuffer[ETH_TX_BUFFER_SIZE] - used to store
*  data to sending.
*  All described above Global variables are used when Software Buffer is used.
*
*******************************************************************************/
CY_ISR(ETH_TX_ISR)
{
    #if(ETH_TX_SOFTWARE_BUF_ENABLED)
        uint8 tmpStatus;
    #endif /* (ETH_TX_SOFTWARE_BUF_ENABLED) */

    /* User code required at start of ISR */
    /* `#START ETH_TX_ISR_START` */

    /* `#END` */

    #if(ETH_TX_SOFTWARE_BUF_ENABLED)
        /* Check if TX data buffer is not empty and there is space in TX FIFO */
        while(ETH_txBufferRead != ETH_txBufferWrite)
        {
            tmpStatus = ETH_GET_STATUS_TX(ETH_swStatusTx);
            ETH_swStatusTx = tmpStatus;

            if(0u != (ETH_swStatusTx & ETH_STS_TX_FIFO_NOT_FULL))
            {
                if(0u == ETH_txBufferFull)
                {
                   ETH_txBufferRead++;

                    if(ETH_txBufferRead >= ETH_TX_BUFFER_SIZE)
                    {
                        ETH_txBufferRead = 0u;
                    }
                }
                else
                {
                    ETH_txBufferFull = 0u;
                }

                /* Move data from the Buffer to the FIFO */
                CY_SET_REG8(ETH_TXDATA_PTR,
                    ETH_txBuffer[ETH_txBufferRead]);
            }
            else
            {
                break;
            }
        }

        if(ETH_txBufferRead == ETH_txBufferWrite)
        {
            /* TX Buffer is EMPTY: disable interrupt on TX NOT FULL */
            ETH_TX_STATUS_MASK_REG &= ((uint8) ~ETH_STS_TX_FIFO_NOT_FULL);
        }

    #endif /* (ETH_TX_SOFTWARE_BUF_ENABLED) */

    /* User code required at end of ISR (Optional) */
    /* `#START ETH_TX_ISR_END` */

    /* `#END` */
}


/*******************************************************************************
* Function Name: ETH_RX_ISR
********************************************************************************
*
* Summary:
*  Interrupt Service Routine for RX portion of the SPI Master.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  ETH_rxBufferWrite - used for the account of the bytes which
*  have been written down in the RX software buffer modified when FIFO contains
*  new data.
*  ETH_rxBufferRead - used for the account of the bytes which
*  have been read from the RX software buffer, modified when overflow occurred.
*  ETH_rxBuffer[ETH_RX_BUFFER_SIZE] - used to store
*  received data, modified when FIFO contains new data.
*  All described above Global variables are used when Software Buffer is used.
*
*******************************************************************************/
CY_ISR(ETH_RX_ISR)
{
    #if(ETH_RX_SOFTWARE_BUF_ENABLED)
        uint8 tmpStatus;
        uint8 rxData;
    #endif /* (ETH_RX_SOFTWARE_BUF_ENABLED) */

    /* User code required at start of ISR */
    /* `#START ETH_RX_ISR_START` */

    /* `#END` */

    #if(ETH_RX_SOFTWARE_BUF_ENABLED)

        tmpStatus = ETH_GET_STATUS_RX(ETH_swStatusRx);
        ETH_swStatusRx = tmpStatus;

        /* Check if RX data FIFO has some data to be moved into the RX Buffer */
        while(0u != (ETH_swStatusRx & ETH_STS_RX_FIFO_NOT_EMPTY))
        {
            rxData = CY_GET_REG8(ETH_RXDATA_PTR);

            /* Set next pointer. */
            ETH_rxBufferWrite++;
            if(ETH_rxBufferWrite >= ETH_RX_BUFFER_SIZE)
            {
                ETH_rxBufferWrite = 0u;
            }

            if(ETH_rxBufferWrite == ETH_rxBufferRead)
            {
                ETH_rxBufferRead++;
                if(ETH_rxBufferRead >= ETH_RX_BUFFER_SIZE)
                {
                    ETH_rxBufferRead = 0u;
                }

                ETH_rxBufferFull = 1u;
            }

            /* Move data from the FIFO to the Buffer */
            ETH_rxBuffer[ETH_rxBufferWrite] = rxData;

            tmpStatus = ETH_GET_STATUS_RX(ETH_swStatusRx);
            ETH_swStatusRx = tmpStatus;
        }

    #endif /* (ETH_RX_SOFTWARE_BUF_ENABLED) */

    /* User code required at end of ISR (Optional) */
    /* `#START ETH_RX_ISR_END` */

    /* `#END` */
}

/* [] END OF FILE */
