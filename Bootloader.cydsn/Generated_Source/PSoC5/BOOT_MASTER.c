/*******************************************************************************
* File Name: BOOT_MASTER.c
* Version 3.30
*
* Description:
*  This file provides the source code of APIs for the I2C component Master mode.
*
* Note:
*
*******************************************************************************
* Copyright 2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "BOOT_PVT.h"

#if(BOOT_MODE_MASTER_ENABLED)

/**********************************
*      System variables
**********************************/

volatile uint8 BOOT_mstrStatus;     /* Master Status byte  */
volatile uint8 BOOT_mstrControl;    /* Master Control byte */

/* Transmit buffer variables */
volatile uint8 * BOOT_mstrRdBufPtr;     /* Pointer to Master Read buffer */
volatile uint8   BOOT_mstrRdBufSize;    /* Master Read buffer size       */
volatile uint8   BOOT_mstrRdBufIndex;   /* Master Read buffer Index      */

/* Receive buffer variables */
volatile uint8 * BOOT_mstrWrBufPtr;     /* Pointer to Master Write buffer */
volatile uint8   BOOT_mstrWrBufSize;    /* Master Write buffer size       */
volatile uint8   BOOT_mstrWrBufIndex;   /* Master Write buffer Index      */


/*******************************************************************************
* Function Name: BOOT_MasterWriteBuf
********************************************************************************
*
* Summary:
*  Automatically writes an entire buffer of data to a slave device. Once the
*  data transfer is initiated by this function, further data transfer is handled
*  by the included ISR in byte by byte mode.
*
* Parameters:
*  slaveAddr: 7-bit slave address.
*  xferData:  Pointer to buffer of data to be sent.
*  cnt:       Size of buffer to send.
*  mode:      Transfer mode defines: start or restart condition generation at
*             begin of the transfer and complete the transfer or halt before
*             generating a stop.
*
* Return:
*  Status error - zero means no errors.
*
* Side Effects:
*  The included ISR will start transfer after start or restart condition will
*  be generated.
*
* Global variables:
*  BOOT_mstrStatus  - used to store current status of I2C Master.
*  BOOT_state       - used to store current state of software FSM.
*  BOOT_mstrControl - used to control master end of transaction with
*  or without the Stop generation.
*  BOOT_mstrWrBufPtr - used to store pointer to master write buffer.
*  BOOT_mstrWrBufIndex - used to current index within master write
*  buffer.
*  BOOT_mstrWrBufSize - used to store master write buffer size.
*
* Reentrant:
*  No
*
*******************************************************************************/
uint8 BOOT_MasterWriteBuf(uint8 slaveAddress, uint8 * wrData, uint8 cnt, uint8 mode)
      
{
    uint8 errStatus;

    errStatus = BOOT_MSTR_NOT_READY;

    if(NULL != wrData)
    {
        /* Check I2C state before transfer: valid are IDLE or HALT */
        if(BOOT_SM_IDLE == BOOT_state)
        {
            /* Check if free: Master is ready to transaction */
            if(BOOT_CHECK_BUS_FREE(BOOT_MCSR_REG))
            {
                errStatus = BOOT_MSTR_NO_ERROR;
            }
            else
            {
                errStatus = BOOT_MSTR_BUS_BUSY;
            }
        }
        else if(BOOT_SM_MSTR_HALT == BOOT_state)
        {
            errStatus = BOOT_MSTR_NO_ERROR;

            CyIntClearPending(BOOT_ISR_NUMBER);
            BOOT_mstrStatus &= ((uint8) ~BOOT_MSTAT_XFER_HALT);
        }
        else
        {
            /* errStatus = BOOT_MSTR_NOT_READY was send before */
        }

        if(BOOT_MSTR_NO_ERROR == errStatus)
        {
            BOOT_state    = BOOT_SM_MSTR_WR_ADDR;
            BOOT_DATA_REG = ((uint8) (slaveAddress << BOOT_SLAVE_ADDR_SHIFT));

            BOOT_mstrWrBufIndex = 0u;
            BOOT_mstrWrBufSize  = cnt;
            BOOT_mstrWrBufPtr   = (volatile uint8 *) wrData;

            BOOT_mstrControl = mode;    /* Save transaction mode */

            /* Generate a Start or ReStart depends on mode */
            if(BOOT_CHECK_RESTART(mode))
            {
                BOOT_GENERATE_RESTART;
            }
            else
            {
                BOOT_GENERATE_START;
            }

            BOOT_mstrStatus &= ((uint8) ~BOOT_MSTAT_WR_CMPLT);

            BOOT_EnableInt();   /* Enable intr to complete transfer */
        }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: BOOT_MasterReadBuf
********************************************************************************
*
* Summary:
*  Automatically writes an entire buffer of data to a slave device. Once the
*  data transfer is initiated by this function, further data transfer is handled
*  by the included ISR in byte by byte mode.
*
* Parameters:
*  slaveAddr: 7-bit slave address.
*  xferData:  Pointer to buffer where to put data from slave.
*  cnt:       Size of buffer to read.
*  mode:      Transfer mode defines: start or restart condition generation at
*             begin of the transfer and complete the transfer or halt before
*             generating a stop.
*
* Return:
*  Status error - zero means no errors.
*
* Side Effects:
*  The included ISR will start transfer after start or restart condition will
*  be generated.
*
* Global variables:
*  BOOT_mstrStatus  - used to store current status of I2C Master.
*  BOOT_state       - used to store current state of software FSM.
*  BOOT_mstrControl - used to control master end of transaction with
*  or without the Stop generation.
*  BOOT_mstrRdBufPtr - used to store pointer to master write buffer.
*  BOOT_mstrRdBufIndex - used to current index within master write
*  buffer.
*  BOOT_mstrRdBufSize - used to store master write buffer size.
*
* Reentrant:
*  No
*
*******************************************************************************/
uint8 BOOT_MasterReadBuf(uint8 slaveAddress, uint8 * rdData, uint8 cnt, uint8 mode)
      
{
    uint8 errStatus;

    errStatus = BOOT_MSTR_NOT_READY;

    if(NULL != rdData)
    {
        /* Check I2C state before transfer: valid are IDLE or HALT */
        if(BOOT_SM_IDLE == BOOT_state)
        {
            /* Check if free: Master is ready to transaction */
            if(BOOT_CHECK_BUS_FREE(BOOT_MCSR_REG))
            {
                errStatus = BOOT_MSTR_NO_ERROR;
            }
            else
            {
                errStatus = BOOT_MSTR_BUS_BUSY;
            }
        }
        else if(BOOT_SM_MSTR_HALT == BOOT_state)
        {
            errStatus = BOOT_MSTR_NO_ERROR;

            CyIntClearPending(BOOT_ISR_NUMBER);
            BOOT_mstrStatus &= ((uint8) ~BOOT_MSTAT_XFER_HALT);
        }
        else
        {
            /* errStatus = BOOT_MSTR_NOT_READY was send before */
        }

        if(BOOT_MSTR_NO_ERROR == errStatus)
        {
            BOOT_state    = BOOT_SM_MSTR_RD_ADDR;
            BOOT_DATA_REG = (((uint8) (slaveAddress << BOOT_SLAVE_ADDR_SHIFT)) |
                                                   BOOT_READ_FLAG);

            BOOT_mstrRdBufIndex  = 0u;
            BOOT_mstrRdBufSize   = cnt;
            BOOT_mstrRdBufPtr    = (volatile uint8 *) rdData;

            BOOT_mstrControl = mode;    /* Save transaction mode */

            /* Generate a Start or ReStart depends on mode */
            if(BOOT_CHECK_RESTART(mode))
            {
                BOOT_GENERATE_RESTART;
            }
            else
            {
                BOOT_GENERATE_START;
            }

            BOOT_mstrStatus &= ((uint8) ~BOOT_MSTAT_RD_CMPLT);

            BOOT_EnableInt();   /* Enable intr to complete transfer */
        }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: BOOT_MasterSendStart
********************************************************************************
*
* Summary:
*  Generates Start condition and sends slave address with read/write bit.
*
* Parameters:
*  slaveAddress:  7-bit slave address.
*  R_nW:          Zero, send write command, non-zero send read command.
*
* Return:
*  Status error - zero means no errors.
*
* Side Effects:
*  This function is entered without a 'byte complete' bit set in the I2C_CSR
*  register. It does not exit until it will be set.
*
* Global variables:
*  BOOT_state - used to store current state of software FSM.
*
* Reentrant:
*  No
*
*******************************************************************************/
uint8 BOOT_MasterSendStart(uint8 slaveAddress, uint8 R_nW)
      
{
    uint8 errStatus;

    errStatus = BOOT_MSTR_NOT_READY;

    /* If IDLE, check if bus is free */
    if(BOOT_SM_IDLE == BOOT_state)
    {
        /* If bus is free, generate Start condition */
        if(BOOT_CHECK_BUS_FREE(BOOT_MCSR_REG))
        {
            BOOT_DisableInt();  /* Disable ISR for Manual functions */

            slaveAddress = ((uint8) (slaveAddress << BOOT_SLAVE_ADDR_SHIFT)); /* Set Address */
            if(0u != R_nW)                                      /* Set the Read/Write flag */
            {
                slaveAddress |= BOOT_READ_FLAG;
                BOOT_state = BOOT_SM_MSTR_RD_ADDR;
            }
            else
            {
                BOOT_state = BOOT_SM_MSTR_WR_ADDR;
            }
            BOOT_DATA_REG = slaveAddress;   /* Write address to data reg */


            BOOT_GENERATE_START;
            while(BOOT_WAIT_BYTE_COMPLETE(BOOT_CSR_REG))
            {
                ; /* Wait for the address to be transfered */
            }

            #if(BOOT_MODE_MULTI_MASTER_SLAVE_ENABLED)
                if(BOOT_CHECK_START_GEN(BOOT_MCSR_REG))
                {
                    BOOT_CLEAR_START_GEN;

                    /* Start condition was not generated: reset FSM to IDLE */
                    BOOT_state = BOOT_SM_IDLE;
                    errStatus = BOOT_MSTR_ERR_ABORT_START_GEN;
                }
                else
            #endif /* (BOOT_MODE_MULTI_MASTER_SLAVE_ENABLED) */

            #if(BOOT_MODE_MULTI_MASTER_ENABLED)

                if(BOOT_CHECK_LOST_ARB(BOOT_CSR_REG))
                {
                    BOOT_BUS_RELEASE;

                    /* Master lost arbitrage: reset FSM to IDLE */
                    BOOT_state = BOOT_SM_IDLE;
                    errStatus = BOOT_MSTR_ERR_ARB_LOST;
                }
                else
            #endif /* (BOOT_MODE_MULTI_MASTER_ENABLED) */

                if(BOOT_CHECK_ADDR_NAK(BOOT_CSR_REG))
                {
                    /* Address has been NACKed: reset FSM to IDLE */
                    BOOT_state = BOOT_SM_IDLE;
                    errStatus = BOOT_MSTR_ERR_LB_NAK;
                }
                else
                {
                    /* Start was sent witout errors */
                    errStatus = BOOT_MSTR_NO_ERROR;
                }
        }
        else
        {
            errStatus = BOOT_MSTR_BUS_BUSY; /* Bus is busy */
        }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: BOOT_MasterSendRestart
********************************************************************************
*
* Summary:
*  Generates ReStart condition and sends slave address with read/write bit.
*
* Parameters:
*  slaveAddress:  7-bit slave address.
*  R_nW:          Zero, send write command, non-zero send read command.
*
* Return:
*  Status error - zero means no errors.
*
* Side Effects:
*  This function is entered without a 'byte complete' bit set in the I2C_CSR
*  register. It does not exit until it will be set.
*
* Global variables:
*  BOOT_state - used to store current state of software FSM.
*
* Reentrant:
*  No
*
*******************************************************************************/
uint8 BOOT_MasterSendRestart(uint8 slaveAddress, uint8 R_nW)
      
{
    uint8 errStatus;

    errStatus = BOOT_MSTR_NOT_READY;

    /* Check if START condition was generated */
    if(BOOT_CHECK_MASTER_MODE(BOOT_MCSR_REG))
    {
        slaveAddress = ((uint8) (slaveAddress << BOOT_SLAVE_ADDR_SHIFT)); /* Set Address */
        if(0u != R_nW)  /* Set the Read/Write flag */
        {
            slaveAddress |= BOOT_READ_FLAG;
            BOOT_state = BOOT_SM_MSTR_RD_ADDR;
        }
        else
        {
            BOOT_state = BOOT_SM_MSTR_WR_ADDR;
        }
        BOOT_DATA_REG = slaveAddress;    /* Write address to data reg */


        BOOT_GENERATE_RESTART_MANUAL;
        while(BOOT_WAIT_BYTE_COMPLETE(BOOT_CSR_REG))
        {
            ; /* Wait for the address to be transfered */
        }

        #if(BOOT_MODE_MULTI_MASTER_ENABLED)
            if(BOOT_CHECK_LOST_ARB(BOOT_CSR_REG))
            {
                BOOT_BUS_RELEASE;

                /* Master lost arbitrage: reset FSM to IDLE */
                BOOT_state = BOOT_SM_IDLE;
                errStatus = BOOT_MSTR_ERR_ARB_LOST;
            }
            else
        #endif /* (BOOT_MODE_MULTI_MASTER_ENABLED) */

            if(BOOT_CHECK_ADDR_NAK(BOOT_CSR_REG))
            {
                /* Address has been NACKed: reset FSM to IDLE */
                BOOT_state = BOOT_SM_IDLE;
                errStatus = BOOT_MSTR_ERR_LB_NAK;
            }
            else
            {
                /* ReStart was sent witout errors */
                errStatus = BOOT_MSTR_NO_ERROR;
            }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: BOOT_MasterSendStop
********************************************************************************
*
* Summary:
*  Generates I2C Stop condition on bus. Function do nothing if Start or Restart
*  condition was failed before call this function.
*
* Parameters:
*  None
*
* Return:
*  Status error - zero means no errors.
*
* Side Effects:
*  The Stop generation is required to complete transaction.
*  This function does not wait whileStop condition will be generated.
*
* Global variables:
*  BOOT_state - used to store current state of software FSM.
*
* Reentrant:
*  No
*
*******************************************************************************/
uint8 BOOT_MasterSendStop(void) 
{
    uint8 errStatus;

    errStatus = BOOT_MSTR_NOT_READY;

    /* Check if START condition was generated */
    if(BOOT_CHECK_MASTER_MODE(BOOT_MCSR_REG))
    {
        BOOT_GENERATE_STOP_MANUAL;              /* Generate STOP */
        BOOT_state = BOOT_SM_IDLE;  /* Reset state to IDLE */

        while(BOOT_WAIT_STOP_COMPLETE(BOOT_CSR_REG))
        {
            ; /* Wait for Stop to be generated */
        }

        errStatus = BOOT_MSTR_NO_ERROR;
        #if(BOOT_MODE_MULTI_MASTER_ENABLED)
            if(BOOT_CHECK_LOST_ARB(BOOT_CSR_REG))
            {
                BOOT_BUS_RELEASE;

                /* NACK was generated by enother instead Stop */
                errStatus = BOOT_MSTR_ERR_ARB_LOST;
            }
        #endif /* (BOOT_MODE_MULTI_MASTER_ENABLED) */
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: BOOT_MasterWriteByte
********************************************************************************
*
* Summary:
*  Sends one byte to a slave. A valid Start or ReStart condition must be
*  generated before this call this function. Function do nothing if Start or
*  Restart condition was failed before call this function.
*
* Parameters:
*  data:  The data byte to send to the slave.
*
* Return:
*  Status error - zero means no errors.
*
* Side Effects:
*  This function is entered without a 'byte complete' bit set in the I2C_CSR
*  register. It does not exit until it will be set.
*
* Global variables:
*  BOOT_state - used to store current state of software FSM.
*
*******************************************************************************/
uint8 BOOT_MasterWriteByte(uint8 theByte) 
{
    uint8 errStatus;

    errStatus = BOOT_MSTR_NOT_READY;

    /* Check if START condition was generated */
    if(BOOT_CHECK_MASTER_MODE(BOOT_MCSR_REG))
    {
        BOOT_DATA_REG = theByte;                        /* Write DATA register */
        BOOT_TRANSMIT_DATA_MANUAL;                      /* Set transmit mode */
        BOOT_state = BOOT_SM_MSTR_WR_DATA;  /* Set state WR_DATA */

        /* Make sure the last byte has been transfered first */
        while(BOOT_WAIT_BYTE_COMPLETE(BOOT_CSR_REG))
        {
            ; /* Wait for byte to be written */
        }

        #if(BOOT_MODE_MULTI_MASTER_ENABLED)
            if(BOOT_CHECK_LOST_ARB(BOOT_CSR_REG))
            {
                BOOT_BUS_RELEASE;

                /* Master lost arbitrage: reset FSM to IDLE */
                BOOT_state = BOOT_SM_IDLE;
                errStatus = BOOT_MSTR_ERR_ARB_LOST;
            }
            /* Check LRB bit */
            else
        #endif /* (BOOT_MODE_MULTI_MASTER_ENABLED) */

            if(BOOT_CHECK_DATA_ACK(BOOT_CSR_REG))
            {
                BOOT_state = BOOT_SM_MSTR_HALT;     /* Set state to HALT */
                errStatus = BOOT_MSTR_NO_ERROR;                 /* The LRB was ACKed */
            }
            else
            {
                BOOT_state = BOOT_SM_MSTR_HALT;     /* Set state to HALT */
                errStatus = BOOT_MSTR_ERR_LB_NAK;               /* The LRB was NACKed */
            }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: BOOT_MasterReadByte
********************************************************************************
*
* Summary:
*  Reads one byte from a slave and ACK or NACK the transfer. A valid Start or
*  ReStart condition must be generated before this call this function. Function
*  do nothing if Start or Restart condition was failed before call this
*  function.
*
* Parameters:
*  acknNack:  Zero, response with NACK, if non-zero response with ACK.
*
* Return:
*  Byte read from slave.
*
* Side Effects:
*  This function is entered without a 'byte complete' bit set in the I2C_CSR
*  register. It does not exit until it will be set.
*
* Global variables:
*  BOOT_state - used to store current state of software FSM.
*
* Reentrant:
*  No
*
*******************************************************************************/
uint8 BOOT_MasterReadByte(uint8 acknNak) 
{
    uint8 theByte;

    theByte = 0u;

    /* Check if START condition was generated */
    if(BOOT_CHECK_MASTER_MODE(BOOT_MCSR_REG))
    {
        /* When address phase need release the bus and receive the byte, then decide ACK or NACK */
        if(BOOT_SM_MSTR_RD_ADDR == BOOT_state)
        {
            BOOT_state = BOOT_SM_MSTR_RD_DATA;
            BOOT_READY_TO_READ_MANUAL;
        }

        while(BOOT_WAIT_BYTE_COMPLETE(BOOT_CSR_REG))
        {
            ; /* Wait for byte to be read */
        }

        theByte = BOOT_DATA_REG;

        /* Now if the ACK flag was set, ACK the data which will release the bus and
           start the next byte in otherwise do NOTHING to the CSR reg.
           This will allow the calling routine to generate a repeat start or
           stop depending on it's preference. */
        if(acknNak != 0u)   /* Do ACK */
        {
            BOOT_ACK_AND_RECEIVE_MANUAL;
        }
        else                /* Do NACK */
        {
            /* Do nothing to be able work with ReStart */
            BOOT_state = BOOT_SM_MSTR_HALT;
        }
    }

    return(theByte);
}


/*******************************************************************************
* Function Name: BOOT_MasterStatus
********************************************************************************
*
* Summary:
*  Returns the master's communication status.
*
* Parameters:
*  None
*
* Return:
*  Current status of I2C master.
*
* Global variables:
*  BOOT_mstrStatus - used to store current status of I2C Master.
*
*******************************************************************************/
uint8 BOOT_MasterStatus(void) 
{
    uint8 status;

    status = BOOT_mstrStatus;

    /* When in Master state only transaction is in progress */
    if(BOOT_CHECK_SM_MASTER)
    {
        /* Add transaction in progress activity to master status */
        status |= BOOT_MSTAT_XFER_INP;
    }
    else
    {
        /* Current master status is valid */
    }

    return(status);
}


/*******************************************************************************
* Function Name: BOOT_MasterClearStatus
********************************************************************************
*
* Summary:
*  Clears all status flags and returns the master status.
*
* Parameters:
*  None
*
* Return:
*  Current status of I2C master.
*
* Global variables:
*  BOOT_mstrStatus - used to store current status of I2C Master.
*
* Reentrant:
*  No
*
*******************************************************************************/
uint8 BOOT_MasterClearStatus(void) 
{
    /* Current master status */
    uint8 status;

    /* Read and clear master status */
    status = BOOT_mstrStatus;
    BOOT_mstrStatus = BOOT_MSTAT_CLEAR;

    return(status);
}


/*******************************************************************************
* Function Name: BOOT_MasterGetReadBufSize
********************************************************************************
*
* Summary:
*  Returns the amount of bytes that has been transferred with an
*  I2C_MasterReadBuf command.
*
* Parameters:
*  None
*
* Return:
*  Byte count of transfer. If the transfer is not yet complete, it will return
*  the byte count transferred so far.
*
* Global variables:
*  BOOT_mstrRdBufIndex - used to current index within master read
*  buffer.
*
*******************************************************************************/
uint8 BOOT_MasterGetReadBufSize(void) 
{
    return(BOOT_mstrRdBufIndex);
}


/*******************************************************************************
* Function Name: BOOT_MasterGetWriteBufSize
********************************************************************************
*
* Summary:
*  Returns the amount of bytes that has been transferred with an
*  I2C_MasterWriteBuf command.
*
* Parameters:
*  None
*
* Return:
*  Byte count of transfer. If the transfer is not yet complete, it will return
*  the byte count transferred so far.
*
* Global variables:
*  BOOT_mstrWrBufIndex - used to current index within master write
*  buffer.
*
*******************************************************************************/
uint8 BOOT_MasterGetWriteBufSize(void) 
{
    return(BOOT_mstrWrBufIndex);
}


/*******************************************************************************
* Function Name: BOOT_MasterClearReadBuf
********************************************************************************
*
* Summary:
*  Resets the read buffer pointer back to the first byte in the buffer.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  BOOT_mstrRdBufIndex - used to current index within master read
*   buffer.
*  BOOT_mstrStatus - used to store current status of I2C Master.
*
* Reentrant:
*  No
*
*******************************************************************************/
void BOOT_MasterClearReadBuf(void) 
{
    BOOT_mstrRdBufIndex = 0u;
    BOOT_mstrStatus    &= ((uint8) ~BOOT_MSTAT_RD_CMPLT);
}


/*******************************************************************************
* Function Name: BOOT_MasterClearWriteBuf
********************************************************************************
*
* Summary:
*  Resets the write buffer pointer back to the first byte in the buffer.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  BOOT_mstrRdBufIndex - used to current index within master read
*   buffer.
*  BOOT_mstrStatus - used to store current status of I2C Master.
*
* Reentrant:
*  No
*
*******************************************************************************/
void BOOT_MasterClearWriteBuf(void) 
{
    BOOT_mstrWrBufIndex = 0u;
    BOOT_mstrStatus    &= ((uint8) ~BOOT_MSTAT_WR_CMPLT);
}


/*******************************************************************************
* Function Name: BOOT_Workaround
********************************************************************************
*
* Summary:
*  Do nothing. This fake fuction use as workaround for CDT 78083.
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
void BOOT_Workaround(void) 
{
    /* Does nothing */
}

#endif /* (BOOT_MODE_MASTER_ENABLED) */


/* [] END OF FILE */
