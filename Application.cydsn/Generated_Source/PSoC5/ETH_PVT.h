/*******************************************************************************
* File Name: .h
* Version 2.40
*
* Description:
*  This private header file contains internal definitions for the SPIM
*  component. Do not use these definitions directly in your application.
*
* Note:
*
********************************************************************************
* Copyright 2012, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SPIM_PVT_ETH_H)
#define CY_SPIM_PVT_ETH_H

#include "ETH.h"


/**********************************
*   Functions with external linkage
**********************************/


/**********************************
*   Variables with external linkage
**********************************/

extern volatile uint8 ETH_swStatusTx;
extern volatile uint8 ETH_swStatusRx;

#if(ETH_TX_SOFTWARE_BUF_ENABLED)
    extern volatile uint8 ETH_txBuffer[ETH_TX_BUFFER_SIZE];
    extern volatile uint8 ETH_txBufferRead;
    extern volatile uint8 ETH_txBufferWrite;
    extern volatile uint8 ETH_txBufferFull;
#endif /* (ETH_TX_SOFTWARE_BUF_ENABLED) */

#if(ETH_RX_SOFTWARE_BUF_ENABLED)
    extern volatile uint8 ETH_rxBuffer[ETH_RX_BUFFER_SIZE];
    extern volatile uint8 ETH_rxBufferRead;
    extern volatile uint8 ETH_rxBufferWrite;
    extern volatile uint8 ETH_rxBufferFull;
#endif /* (ETH_RX_SOFTWARE_BUF_ENABLED) */

#endif /* CY_SPIM_PVT_ETH_H */


/* [] END OF FILE */
