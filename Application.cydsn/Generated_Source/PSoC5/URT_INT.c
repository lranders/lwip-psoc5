/*******************************************************************************
* File Name: URT_INT.c
* Version 2.30
*
* Description:
*  This file provides all Interrupt Service functionality of the UART component
*
* Note:
*  Any unusual or non-standard behavior should be noted here. Other-
*  wise, this section should remain blank.
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "URT.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (URT_RX_ENABLED || URT_HD_ENABLED) && \
     (URT_RXBUFFERSIZE > URT_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: URT_RXISR
    ********************************************************************************
    *
    * Summary:
    *  Interrupt Service Routine for RX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  URT_rxBuffer - RAM buffer pointer for save received data.
    *  URT_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  URT_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  URT_rxBufferOverflow - software overflow flag. Set to one
    *     when URT_rxBufferWrite index overtakes
    *     URT_rxBufferRead index.
    *  URT_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when URT_rxBufferWrite is equal to
    *    URT_rxBufferRead
    *  URT_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  URT_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(URT_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START URT_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = URT_RXSTATUS_REG;

        if((readData & (URT_RX_STS_BREAK | URT_RX_STS_PAR_ERROR |
                        URT_RX_STS_STOP_ERROR | URT_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START URT_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & URT_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (URT_RXHW_ADDRESS_ENABLED)
                if(URT_rxAddressMode == (uint8)URT__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & URT_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & URT_RX_STS_ADDR_MATCH) != 0u)
                        {
                            URT_rxAddressDetected = 1u;
                        }
                        else
                        {
                            URT_rxAddressDetected = 0u;
                        }
                    }

                    readData = URT_RXDATA_REG;
                    if(URT_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        URT_rxBuffer[URT_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    URT_rxBuffer[URT_rxBufferWrite] = URT_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                URT_rxBuffer[URT_rxBufferWrite] = URT_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(URT_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    URT_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                URT_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(URT_rxBufferWrite >= URT_RXBUFFERSIZE)
                {
                    URT_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(URT_rxBufferWrite == URT_rxBufferRead)
                {
                    URT_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(URT_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        URT_RXSTATUS_MASK_REG  &= (uint8)~URT_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(URT_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End URT_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = URT_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START URT_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End URT_RX_ENABLED && (URT_RXBUFFERSIZE > URT_FIFO_LENGTH) */


#if(URT_TX_ENABLED && (URT_TXBUFFERSIZE > URT_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: URT_TXISR
    ********************************************************************************
    *
    * Summary:
    * Interrupt Service Routine for the TX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  URT_txBuffer - RAM buffer pointer for transmit data from.
    *  URT_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  URT_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(URT_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START URT_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((URT_txBufferRead != URT_txBufferWrite) &&
             ((URT_TXSTATUS_REG & URT_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(URT_txBufferRead >= URT_TXBUFFERSIZE)
            {
                URT_txBufferRead = 0u;
            }

            URT_TXDATA_REG = URT_txBuffer[URT_txBufferRead];

            /* Set next pointer. */
            URT_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START URT_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End URT_TX_ENABLED && (URT_TXBUFFERSIZE > URT_FIFO_LENGTH) */


/* [] END OF FILE */
