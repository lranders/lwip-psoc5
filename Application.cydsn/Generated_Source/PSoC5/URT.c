/*******************************************************************************
* File Name: URT.c
* Version 2.30
*
* Description:
*  This file provides all API functionality of the UART component
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
#include "CyLib.h"
#if(URT_INTERNAL_CLOCK_USED)
    #include "URT_IntClock.h"
#endif /* End URT_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 URT_initVar = 0u;
#if( URT_TX_ENABLED && (URT_TXBUFFERSIZE > URT_FIFO_LENGTH))
    volatile uint8 URT_txBuffer[URT_TXBUFFERSIZE];
    volatile uint8 URT_txBufferRead = 0u;
    uint8 URT_txBufferWrite = 0u;
#endif /* End URT_TX_ENABLED */
#if( ( URT_RX_ENABLED || URT_HD_ENABLED ) && \
     (URT_RXBUFFERSIZE > URT_FIFO_LENGTH) )
    volatile uint8 URT_rxBuffer[URT_RXBUFFERSIZE];
    volatile uint8 URT_rxBufferRead = 0u;
    volatile uint8 URT_rxBufferWrite = 0u;
    volatile uint8 URT_rxBufferLoopDetect = 0u;
    volatile uint8 URT_rxBufferOverflow = 0u;
    #if (URT_RXHW_ADDRESS_ENABLED)
        volatile uint8 URT_rxAddressMode = URT_RXADDRESSMODE;
        volatile uint8 URT_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End URT_RX_ENABLED */


/*******************************************************************************
* Function Name: URT_Start
********************************************************************************
*
* Summary:
*  Initialize and Enable the UART component.
*  Enable the clock input to enable operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The URT_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the URT_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void URT_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(URT_initVar == 0u)
    {
        URT_Init();
        URT_initVar = 1u;
    }
    URT_Enable();
}


/*******************************************************************************
* Function Name: URT_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  URT_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void URT_Init(void) 
{
    #if(URT_RX_ENABLED || URT_HD_ENABLED)

        #if(URT_RX_INTERRUPT_ENABLED && (URT_RXBUFFERSIZE > URT_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(URT_RX_VECT_NUM, &URT_RXISR);
            CyIntSetPriority(URT_RX_VECT_NUM, URT_RX_PRIOR_NUM);
        #endif /* End URT_RX_INTERRUPT_ENABLED */

        #if (URT_RXHW_ADDRESS_ENABLED)
            URT_SetRxAddressMode(URT_RXAddressMode);
            URT_SetRxAddress1(URT_RXHWADDRESS1);
            URT_SetRxAddress2(URT_RXHWADDRESS2);
        #endif /* End URT_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        URT_RXBITCTR_PERIOD_REG = URT_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        URT_RXSTATUS_MASK_REG  = URT_INIT_RX_INTERRUPTS_MASK;
    #endif /* End URT_RX_ENABLED || URT_HD_ENABLED*/

    #if(URT_TX_ENABLED)
        #if(URT_TX_INTERRUPT_ENABLED && (URT_TXBUFFERSIZE > URT_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(URT_TX_VECT_NUM, &URT_TXISR);
            CyIntSetPriority(URT_TX_VECT_NUM, URT_TX_PRIOR_NUM);
        #endif /* End URT_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(URT_TXCLKGEN_DP)
            URT_TXBITCLKGEN_CTR_REG = URT_BIT_CENTER;
            URT_TXBITCLKTX_COMPLETE_REG = (URT_NUMBER_OF_DATA_BITS +
                        URT_NUMBER_OF_START_BIT) * URT_OVER_SAMPLE_COUNT;
        #else
            URT_TXBITCTR_PERIOD_REG = ((URT_NUMBER_OF_DATA_BITS +
                        URT_NUMBER_OF_START_BIT) * URT_OVER_SAMPLE_8) - 1u;
        #endif /* End URT_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(URT_TX_INTERRUPT_ENABLED && (URT_TXBUFFERSIZE > URT_FIFO_LENGTH))
            URT_TXSTATUS_MASK_REG = URT_TX_STS_FIFO_EMPTY;
        #else
            URT_TXSTATUS_MASK_REG = URT_INIT_TX_INTERRUPTS_MASK;
        #endif /*End URT_TX_INTERRUPT_ENABLED*/

    #endif /* End URT_TX_ENABLED */

    #if(URT_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        URT_WriteControlRegister( \
            (URT_ReadControlRegister() & (uint8)~URT_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(URT_PARITY_TYPE << URT_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End URT_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: URT_Enable
********************************************************************************
*
* Summary:
*  Enables the UART block operation
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  URT_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void URT_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(URT_RX_ENABLED || URT_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        URT_RXBITCTR_CONTROL_REG |= URT_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        URT_RXSTATUS_ACTL_REG  |= URT_INT_ENABLE;
        #if(URT_RX_INTERRUPT_ENABLED && (URT_RXBUFFERSIZE > URT_FIFO_LENGTH))
            CyIntEnable(URT_RX_VECT_NUM);
            #if (URT_RXHW_ADDRESS_ENABLED)
                URT_rxAddressDetected = 0u;
            #endif /* End URT_RXHW_ADDRESS_ENABLED */
        #endif /* End URT_RX_INTERRUPT_ENABLED */
    #endif /* End URT_RX_ENABLED || URT_HD_ENABLED*/

    #if(URT_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!URT_TXCLKGEN_DP)
            URT_TXBITCTR_CONTROL_REG |= URT_CNTR_ENABLE;
        #endif /* End URT_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        URT_TXSTATUS_ACTL_REG |= URT_INT_ENABLE;
        #if(URT_TX_INTERRUPT_ENABLED && (URT_TXBUFFERSIZE > URT_FIFO_LENGTH))
            CyIntEnable(URT_TX_VECT_NUM);
        #endif /* End URT_TX_INTERRUPT_ENABLED*/
     #endif /* End URT_TX_ENABLED */

    #if(URT_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        URT_IntClock_Start();
    #endif /* End URT_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: URT_Stop
********************************************************************************
*
* Summary:
*  Disable the UART component
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void URT_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(URT_RX_ENABLED || URT_HD_ENABLED)
        URT_RXBITCTR_CONTROL_REG &= (uint8)~URT_CNTR_ENABLE;
    #endif /* End URT_RX_ENABLED */

    #if(URT_TX_ENABLED)
        #if(!URT_TXCLKGEN_DP)
            URT_TXBITCTR_CONTROL_REG &= (uint8)~URT_CNTR_ENABLE;
        #endif /* End URT_TXCLKGEN_DP */
    #endif /* URT_TX_ENABLED */

    #if(URT_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        URT_IntClock_Stop();
    #endif /* End URT_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(URT_RX_ENABLED || URT_HD_ENABLED)
        URT_RXSTATUS_ACTL_REG  &= (uint8)~URT_INT_ENABLE;
        #if(URT_RX_INTERRUPT_ENABLED && (URT_RXBUFFERSIZE > URT_FIFO_LENGTH))
            URT_DisableRxInt();
        #endif /* End URT_RX_INTERRUPT_ENABLED */
    #endif /* End URT_RX_ENABLED */

    #if(URT_TX_ENABLED)
        URT_TXSTATUS_ACTL_REG &= (uint8)~URT_INT_ENABLE;
        #if(URT_TX_INTERRUPT_ENABLED && (URT_TXBUFFERSIZE > URT_FIFO_LENGTH))
            URT_DisableTxInt();
        #endif /* End URT_TX_INTERRUPT_ENABLED */
    #endif /* End URT_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: URT_ReadControlRegister
********************************************************************************
*
* Summary:
*  Read the current state of the control register
*
* Parameters:
*  None.
*
* Return:
*  Current state of the control register.
*
*******************************************************************************/
uint8 URT_ReadControlRegister(void) 
{
    #if( URT_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(URT_CONTROL_REG);
    #endif /* End URT_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: URT_WriteControlRegister
********************************************************************************
*
* Summary:
*  Writes an 8-bit value into the control register
*
* Parameters:
*  control:  control register value
*
* Return:
*  None.
*
*******************************************************************************/
void  URT_WriteControlRegister(uint8 control) 
{
    #if( URT_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       URT_CONTROL_REG = control;
    #endif /* End URT_CONTROL_REG_REMOVED */
}


#if(URT_RX_ENABLED || URT_HD_ENABLED)

    #if(URT_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: URT_EnableRxInt
        ********************************************************************************
        *
        * Summary:
        *  Enable RX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Enable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void URT_EnableRxInt(void) 
        {
            CyIntEnable(URT_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: URT_DisableRxInt
        ********************************************************************************
        *
        * Summary:
        *  Disable RX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Disable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void URT_DisableRxInt(void) 
        {
            CyIntDisable(URT_RX_VECT_NUM);
        }

    #endif /* URT_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: URT_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configure which status bits trigger an interrupt event
    *
    * Parameters:
    *  IntSrc:  An or'd combination of the desired status bit masks (defined in
    *           the header file)
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void URT_SetRxInterruptMode(uint8 intSrc) 
    {
        URT_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: URT_ReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Returns data in RX Data register without checking status register to
    *  determine if data is valid
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Received data from RX register
    *
    * Global Variables:
    *  URT_rxBuffer - RAM buffer pointer for save received data.
    *  URT_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  URT_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  URT_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 URT_ReadRxData(void) 
    {
        uint8 rxData;

        #if(URT_RXBUFFERSIZE > URT_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(URT_RX_INTERRUPT_ENABLED)
                URT_DisableRxInt();
            #endif /* URT_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = URT_rxBufferRead;
            loc_rxBufferWrite = URT_rxBufferWrite;

            if( (URT_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = URT_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= URT_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                URT_rxBufferRead = loc_rxBufferRead;

                if(URT_rxBufferLoopDetect != 0u )
                {
                    URT_rxBufferLoopDetect = 0u;
                    #if( (URT_RX_INTERRUPT_ENABLED) && (URT_FLOW_CONTROL != 0u) && \
                         (URT_RXBUFFERSIZE > URT_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( URT_HD_ENABLED )
                            if((URT_CONTROL_REG & URT_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                URT_RXSTATUS_MASK_REG  |= URT_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            URT_RXSTATUS_MASK_REG  |= URT_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end URT_HD_ENABLED */
                    #endif /* URT_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = URT_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(URT_RX_INTERRUPT_ENABLED)
                URT_EnableRxInt();
            #endif /* End URT_RX_INTERRUPT_ENABLED */

        #else /* URT_RXBUFFERSIZE > URT_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = URT_RXDATA_REG;

        #endif /* URT_RXBUFFERSIZE > URT_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: URT_ReadRxStatus
    ********************************************************************************
    *
    * Summary:
    *  Read the current state of the status register
    *  And detect software buffer overflow.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Current state of the status register.
    *
    * Global Variables:
    *  URT_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   URT_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   URT_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 URT_ReadRxStatus(void) 
    {
        uint8 status;

        status = URT_RXSTATUS_REG & URT_RX_HW_MASK;

        #if(URT_RXBUFFERSIZE > URT_FIFO_LENGTH)
            if( URT_rxBufferOverflow != 0u )
            {
                status |= URT_RX_STS_SOFT_BUFF_OVER;
                URT_rxBufferOverflow = 0u;
            }
        #endif /* URT_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: URT_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Reads UART RX buffer immediately, if data is not available or an error
    *  condition exists, zero is returned; otherwise, character is read and
    *  returned.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Character read from UART RX buffer. ASCII characters from 1 to 255 are valid.
    *  A returned zero signifies an error condition or no data available.
    *
    * Global Variables:
    *  URT_rxBuffer - RAM buffer pointer for save received data.
    *  URT_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  URT_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  URT_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 URT_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(URT_RXBUFFERSIZE > URT_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(URT_RX_INTERRUPT_ENABLED)
                URT_DisableRxInt();
            #endif /* URT_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = URT_rxBufferRead;
            loc_rxBufferWrite = URT_rxBufferWrite;

            if( (URT_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = URT_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= URT_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                URT_rxBufferRead = loc_rxBufferRead;

                if(URT_rxBufferLoopDetect > 0u )
                {
                    URT_rxBufferLoopDetect = 0u;
                    #if( (URT_RX_INTERRUPT_ENABLED) && (URT_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( URT_HD_ENABLED )
                            if((URT_CONTROL_REG & URT_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                URT_RXSTATUS_MASK_REG  |= URT_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            URT_RXSTATUS_MASK_REG  |= URT_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end URT_HD_ENABLED */
                    #endif /* URT_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = URT_RXSTATUS_REG;
                if((rxStatus & URT_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = URT_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (URT_RX_STS_BREAK | URT_RX_STS_PAR_ERROR |
                                   URT_RX_STS_STOP_ERROR | URT_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(URT_RX_INTERRUPT_ENABLED)
                URT_EnableRxInt();
            #endif /* URT_RX_INTERRUPT_ENABLED */

        #else /* URT_RXBUFFERSIZE > URT_FIFO_LENGTH */

            rxStatus =URT_RXSTATUS_REG;
            if((rxStatus & URT_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = URT_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (URT_RX_STS_BREAK | URT_RX_STS_PAR_ERROR |
                               URT_RX_STS_STOP_ERROR | URT_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* URT_RXBUFFERSIZE > URT_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: URT_GetByte
    ********************************************************************************
    *
    * Summary:
    *  Grab the next available byte of data from the recieve FIFO
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  MSB contains Status Register and LSB contains UART RX data
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint16 URT_GetByte(void) 
    {
        return ( ((uint16)URT_ReadRxStatus() << 8u) | URT_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: URT_GetRxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Determine the amount of bytes left in the RX buffer and return the count in
    *  bytes
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  uint8: Integer count of the number of bytes left
    *  in the RX buffer
    *
    * Global Variables:
    *  URT_rxBufferWrite - used to calculate left bytes.
    *  URT_rxBufferRead - used to calculate left bytes.
    *  URT_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 URT_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(URT_RXBUFFERSIZE > URT_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(URT_RX_INTERRUPT_ENABLED)
                URT_DisableRxInt();
            #endif /* URT_RX_INTERRUPT_ENABLED */

            if(URT_rxBufferRead == URT_rxBufferWrite)
            {
                if(URT_rxBufferLoopDetect > 0u)
                {
                    size = URT_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(URT_rxBufferRead < URT_rxBufferWrite)
            {
                size = (URT_rxBufferWrite - URT_rxBufferRead);
            }
            else
            {
                size = (URT_RXBUFFERSIZE - URT_rxBufferRead) + URT_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(URT_RX_INTERRUPT_ENABLED)
                URT_EnableRxInt();
            #endif /* End URT_RX_INTERRUPT_ENABLED */

        #else /* URT_RXBUFFERSIZE > URT_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((URT_RXSTATUS_REG & URT_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End URT_RXBUFFERSIZE > URT_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: URT_ClearRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the RX RAM buffer by setting the read and write pointers both to zero.
    *  Clears hardware RX FIFO.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  URT_rxBufferWrite - cleared to zero.
    *  URT_rxBufferRead - cleared to zero.
    *  URT_rxBufferLoopDetect - cleared to zero.
    *  URT_rxBufferOverflow - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may
    *  have remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM or FIFO buffer will be lost.
    *******************************************************************************/
    void URT_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        URT_RXDATA_AUX_CTL_REG |=  URT_RX_FIFO_CLR;
        URT_RXDATA_AUX_CTL_REG &= (uint8)~URT_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(URT_RXBUFFERSIZE > URT_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(URT_RX_INTERRUPT_ENABLED)
                URT_DisableRxInt();
            #endif /* End URT_RX_INTERRUPT_ENABLED */

            URT_rxBufferRead = 0u;
            URT_rxBufferWrite = 0u;
            URT_rxBufferLoopDetect = 0u;
            URT_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(URT_RX_INTERRUPT_ENABLED)
                URT_EnableRxInt();
            #endif /* End URT_RX_INTERRUPT_ENABLED */
        #endif /* End URT_RXBUFFERSIZE > URT_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: URT_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  URT__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  URT__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  URT__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  URT__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  URT__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  URT_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  URT_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void URT_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(URT_RXHW_ADDRESS_ENABLED)
            #if(URT_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* URT_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = URT_CONTROL_REG & (uint8)~URT_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << URT_CTRL_RXADDR_MODE0_SHIFT);
                URT_CONTROL_REG = tmpCtrl;
                #if(URT_RX_INTERRUPT_ENABLED && \
                   (URT_RXBUFFERSIZE > URT_FIFO_LENGTH) )
                    URT_rxAddressMode = addressMode;
                    URT_rxAddressDetected = 0u;
                #endif /* End URT_RXBUFFERSIZE > URT_FIFO_LENGTH*/
            #endif /* End URT_CONTROL_REG_REMOVED */
        #else /* URT_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End URT_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: URT_SetRxAddress1
    ********************************************************************************
    *
    * Summary:
    *  Set the first hardware address compare value
    *
    * Parameters:
    *  address
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void URT_SetRxAddress1(uint8 address) 

    {
        URT_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: URT_SetRxAddress2
    ********************************************************************************
    *
    * Summary:
    *  Set the second hardware address compare value
    *
    * Parameters:
    *  address
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void URT_SetRxAddress2(uint8 address) 
    {
        URT_RXADDRESS2_REG = address;
    }

#endif  /* URT_RX_ENABLED || URT_HD_ENABLED*/


#if( (URT_TX_ENABLED) || (URT_HD_ENABLED) )

    #if(URT_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: URT_EnableTxInt
        ********************************************************************************
        *
        * Summary:
        *  Enable TX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Enable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void URT_EnableTxInt(void) 
        {
            CyIntEnable(URT_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: URT_DisableTxInt
        ********************************************************************************
        *
        * Summary:
        *  Disable TX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Disable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void URT_DisableTxInt(void) 
        {
            CyIntDisable(URT_TX_VECT_NUM);
        }

    #endif /* URT_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: URT_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configure which status bits trigger an interrupt event
    *
    * Parameters:
    *  intSrc: An or'd combination of the desired status bit masks (defined in
    *          the header file)
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void URT_SetTxInterruptMode(uint8 intSrc) 
    {
        URT_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: URT_WriteTxData
    ********************************************************************************
    *
    * Summary:
    *  Write a byte of data to the Transmit FIFO or TX buffer to be sent when the
    *  bus is available. WriteTxData sends a byte without checking for buffer room
    *  or status. It is up to the user to separately check status.
    *
    * Parameters:
    *  TXDataByte: byte of data to place in the transmit FIFO
    *
    * Return:
    * void
    *
    * Global Variables:
    *  URT_txBuffer - RAM buffer pointer for save data for transmission
    *  URT_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  URT_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  URT_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void URT_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(URT_initVar != 0u)
        {
            #if(URT_TXBUFFERSIZE > URT_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(URT_TX_INTERRUPT_ENABLED)
                    URT_DisableTxInt();
                #endif /* End URT_TX_INTERRUPT_ENABLED */

                if( (URT_txBufferRead == URT_txBufferWrite) &&
                    ((URT_TXSTATUS_REG & URT_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    URT_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(URT_txBufferWrite >= URT_TXBUFFERSIZE)
                    {
                        URT_txBufferWrite = 0u;
                    }

                    URT_txBuffer[URT_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    URT_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(URT_TX_INTERRUPT_ENABLED)
                    URT_EnableTxInt();
                #endif /* End URT_TX_INTERRUPT_ENABLED */

            #else /* URT_TXBUFFERSIZE > URT_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                URT_TXDATA_REG = txDataByte;

            #endif /* End URT_TXBUFFERSIZE > URT_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: URT_ReadTxStatus
    ********************************************************************************
    *
    * Summary:
    *  Read the status register for the component
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Contents of the status register
    *
    * Theory:
    *  This function reads the status register which is clear on read. It is up to
    *  the user to handle all bits in this return value accordingly, even if the bit
    *  was not enabled as an interrupt source the event happened and must be handled
    *  accordingly.
    *
    *******************************************************************************/
    uint8 URT_ReadTxStatus(void) 
    {
        return(URT_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: URT_PutChar
    ********************************************************************************
    *
    * Summary:
    *  Wait to send byte until TX register or buffer has room.
    *
    * Parameters:
    *  txDataByte: The 8-bit data value to send across the UART.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  URT_txBuffer - RAM buffer pointer for save data for transmission
    *  URT_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  URT_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  URT_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void URT_PutChar(uint8 txDataByte) 
    {
            #if(URT_TXBUFFERSIZE > URT_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((URT_TXBUFFERSIZE > URT_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(URT_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = URT_txBufferWrite;
                    loc_txBufferRead = URT_txBufferRead;
                    #if ((URT_TXBUFFERSIZE > URT_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(URT_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(URT_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((URT_TXSTATUS_REG & URT_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    URT_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= URT_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    URT_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((URT_TXBUFFERSIZE > URT_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(URT_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    URT_txBufferWrite = loc_txBufferWrite;
                    #if ((URT_TXBUFFERSIZE > URT_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(URT_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* URT_TXBUFFERSIZE > URT_FIFO_LENGTH */

                while((URT_TXSTATUS_REG & URT_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                URT_TXDATA_REG = txDataByte;

            #endif /* End URT_TXBUFFERSIZE > URT_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: URT_PutString
    ********************************************************************************
    *
    * Summary:
    *  Write a Sequence of bytes on the Transmit line. Data comes from RAM or ROM.
    *
    * Parameters:
    *  string: char pointer to character string of Data to Send.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  URT_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  This function will block if there is not enough memory to place the whole
    *  string, it will block until the entire string has been written to the
    *  transmit buffer.
    *
    *******************************************************************************/
    void URT_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(URT_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                URT_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: URT_PutArray
    ********************************************************************************
    *
    * Summary:
    *  Write a Sequence of bytes on the Transmit line. Data comes from RAM or ROM.
    *
    * Parameters:
    *  string: Address of the memory array residing in RAM or ROM.
    *  byteCount: Number of Bytes to be transmitted.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  URT_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void URT_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(URT_initVar != 0u)
        {
            do
            {
                URT_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: URT_PutCRLF
    ********************************************************************************
    *
    * Summary:
    *  Write a character and then carriage return and line feed.
    *
    * Parameters:
    *  txDataByte: uint8 Character to send.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  URT_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void URT_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(URT_initVar != 0u)
        {
            URT_PutChar(txDataByte);
            URT_PutChar(0x0Du);
            URT_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: URT_GetTxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Determine the amount of space left in the TX buffer and return the count in
    *  bytes
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Integer count of the number of bytes left in the TX buffer
    *
    * Global Variables:
    *  URT_txBufferWrite - used to calculate left space.
    *  URT_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 URT_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(URT_TXBUFFERSIZE > URT_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(URT_TX_INTERRUPT_ENABLED)
                URT_DisableTxInt();
            #endif /* End URT_TX_INTERRUPT_ENABLED */

            if(URT_txBufferRead == URT_txBufferWrite)
            {
                size = 0u;
            }
            else if(URT_txBufferRead < URT_txBufferWrite)
            {
                size = (URT_txBufferWrite - URT_txBufferRead);
            }
            else
            {
                size = (URT_TXBUFFERSIZE - URT_txBufferRead) + URT_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(URT_TX_INTERRUPT_ENABLED)
                URT_EnableTxInt();
            #endif /* End URT_TX_INTERRUPT_ENABLED */

        #else /* URT_TXBUFFERSIZE > URT_FIFO_LENGTH */

            size = URT_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & URT_TX_STS_FIFO_FULL) != 0u)
            {
                size = URT_FIFO_LENGTH;
            }
            else if((size & URT_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End URT_TXBUFFERSIZE > URT_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: URT_ClearTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the TX RAM buffer by setting the read and write pointers both to zero.
    *  Clears the hardware TX FIFO.  Any data present in the FIFO will not be sent.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  URT_txBufferWrite - cleared to zero.
    *  URT_txBufferRead - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may have
    *  remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM buffer will be lost when overwritten.
    *
    *******************************************************************************/
    void URT_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        URT_TXDATA_AUX_CTL_REG |=  URT_TX_FIFO_CLR;
        URT_TXDATA_AUX_CTL_REG &= (uint8)~URT_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(URT_TXBUFFERSIZE > URT_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(URT_TX_INTERRUPT_ENABLED)
                URT_DisableTxInt();
            #endif /* End URT_TX_INTERRUPT_ENABLED */

            URT_txBufferRead = 0u;
            URT_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(URT_TX_INTERRUPT_ENABLED)
                URT_EnableTxInt();
            #endif /* End URT_TX_INTERRUPT_ENABLED */

        #endif /* End URT_TXBUFFERSIZE > URT_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: URT_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Write a Break command to the UART
    *
    * Parameters:
    *  uint8 retMode:  Wait mode,
    *   0 - Initialize registers for Break, sends the Break signal and return
    *       imediately.
    *   1 - Wait until Break sending is complete, reinitialize registers to normal
    *       transmission mode then return.
    *   2 - Reinitialize registers to normal transmission mode then return.
    *   3 - both steps: 0 and 1
    *       init registers for Break, send Break signal
    *       wait until Break sending is complete, reinit registers to normal
    *       transmission mode then return.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  URT_initVar - checked to identify that the component has been
    *     initialized.
    *  tx_period - static variable, used for keeping TX period configuration.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  SendBreak function initializes registers to send 13-bit break signal. It is
    *  important to return the registers configuration to normal for continue 8-bit
    *  operation.
    *  Trere are 3 variants for this API usage:
    *  1) SendBreak(3) - function will send the Break signal and take care on the
    *     configuration returning. Funcition will block CPU untill transmition
    *     complete.
    *  2) User may want to use bloking time if UART configured to the low speed
    *     operation
    *     Emample for this case:
    *     SendBreak(0);     - init Break signal transmition
    *         Add your code here to use CPU time
    *     SendBreak(1);     - complete Break operation
    *  3) Same to 2) but user may want to init and use the interrupt for complete
    *     break operation.
    *     Example for this case:
    *     Init TX interrupt whith "TX - On TX Complete" parameter
    *     SendBreak(0);     - init Break signal transmition
    *         Add your code here to use CPU time
    *     When interrupt appear with UART_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *   Uses static variable to keep registers configuration.
    *
    *******************************************************************************/
    void URT_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(URT_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(URT_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == URT_SEND_BREAK) ||
                    (retMode == URT_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    URT_WriteControlRegister(URT_ReadControlRegister() |
                                                          URT_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    URT_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = URT_TXSTATUS_REG;
                    }while((tmpStat & URT_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == URT_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == URT_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = URT_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & URT_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == URT_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == URT_REINIT) ||
                    (retMode == URT_SEND_WAIT_REINIT) )
                {
                    URT_WriteControlRegister(URT_ReadControlRegister() &
                                                  (uint8)~URT_CTRL_HD_SEND_BREAK);
                }

            #else /* URT_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == URT_SEND_BREAK) ||
                    (retMode == URT_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (URT_PARITY_TYPE != URT__B_UART__NONE_REVB) || \
                                        (URT_PARITY_TYPE_SW != 0u) )
                        URT_WriteControlRegister(URT_ReadControlRegister() |
                                                              URT_CTRL_HD_SEND_BREAK);
                    #endif /* End URT_PARITY_TYPE != URT__B_UART__NONE_REVB  */

                    #if(URT_TXCLKGEN_DP)
                        tx_period = URT_TXBITCLKTX_COMPLETE_REG;
                        URT_TXBITCLKTX_COMPLETE_REG = URT_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = URT_TXBITCTR_PERIOD_REG;
                        URT_TXBITCTR_PERIOD_REG = URT_TXBITCTR_BREAKBITS8X;
                    #endif /* End URT_TXCLKGEN_DP */

                    /* Send zeros*/
                    URT_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = URT_TXSTATUS_REG;
                    }while((tmpStat & URT_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == URT_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == URT_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = URT_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & URT_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == URT_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == URT_REINIT) ||
                    (retMode == URT_SEND_WAIT_REINIT) )
                {

                    #if(URT_TXCLKGEN_DP)
                        URT_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        URT_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End URT_TXCLKGEN_DP */

                    #if( (URT_PARITY_TYPE != URT__B_UART__NONE_REVB) || \
                         (URT_PARITY_TYPE_SW != 0u) )
                        URT_WriteControlRegister(URT_ReadControlRegister() &
                                                      (uint8)~URT_CTRL_HD_SEND_BREAK);
                    #endif /* End URT_PARITY_TYPE != NONE */
                }
            #endif    /* End URT_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: URT_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the transmit addressing mode
    *
    * Parameters:
    *  addressMode: 0 -> Space
    *               1 -> Mark
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void URT_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( URT_CONTROL_REG_REMOVED == 0u )
                URT_WriteControlRegister(URT_ReadControlRegister() |
                                                      URT_CTRL_MARK);
            #endif /* End URT_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( URT_CONTROL_REG_REMOVED == 0u )
                URT_WriteControlRegister(URT_ReadControlRegister() &
                                                    (uint8)~URT_CTRL_MARK);
            #endif /* End URT_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndURT_TX_ENABLED */

#if(URT_HD_ENABLED)


    /*******************************************************************************
    * Function Name: URT_LoadTxConfig
    ********************************************************************************
    *
    * Summary:
    *  Unloads the Rx configuration if required and loads the
    *  Tx configuration. It is the users responsibility to ensure that any
    *  transaction is complete and it is safe to unload the Tx
    *  configuration.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Valid only for half duplex UART.
    *
    * Side Effects:
    *  Disable RX interrupt mask, when software buffer has been used.
    *
    *******************************************************************************/
    void URT_LoadTxConfig(void) 
    {
        #if((URT_RX_INTERRUPT_ENABLED) && (URT_RXBUFFERSIZE > URT_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            URT_SetRxInterruptMode(0u);
        #endif /* URT_RX_INTERRUPT_ENABLED */

        URT_WriteControlRegister(URT_ReadControlRegister() | URT_CTRL_HD_SEND);
        URT_RXBITCTR_PERIOD_REG = URT_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(URT_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: URT_LoadRxConfig
    ********************************************************************************
    *
    * Summary:
    *  Unloads the Tx configuration if required and loads the
    *  Rx configuration. It is the users responsibility to ensure that any
    *  transaction is complete and it is safe to unload the Rx
    *  configuration.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Valid only for half duplex UART
    *
    * Side Effects:
    *  Set RX interrupt mask based on customizer settings, when software buffer
    *  has been used.
    *
    *******************************************************************************/
    void URT_LoadRxConfig(void) 
    {
        URT_WriteControlRegister(URT_ReadControlRegister() &
                                                (uint8)~URT_CTRL_HD_SEND);
        URT_RXBITCTR_PERIOD_REG = URT_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(URT_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((URT_RX_INTERRUPT_ENABLED) && (URT_RXBUFFERSIZE > URT_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            URT_SetRxInterruptMode(URT_INIT_RX_INTERRUPTS_MASK);
        #endif /* URT_RX_INTERRUPT_ENABLED */
    }

#endif  /* URT_HD_ENABLED */


/* [] END OF FILE */
