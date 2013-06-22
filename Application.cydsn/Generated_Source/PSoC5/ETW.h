/*******************************************************************************
* File Name: ETW.h
* Version 1.70
*
*  Description:
*   Provides the function definitions for the Interrupt Controller.
*
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/
#if !defined(CY_ISR_ETW_H)
#define CY_ISR_ETW_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void ETW_Start(void);
void ETW_StartEx(cyisraddress address);
void ETW_Stop(void);

CY_ISR_PROTO(ETW_Interrupt);

void ETW_SetVector(cyisraddress address);
cyisraddress ETW_GetVector(void);

void ETW_SetPriority(uint8 priority);
uint8 ETW_GetPriority(void);

void ETW_Enable(void);
uint8 ETW_GetState(void);
void ETW_Disable(void);

void ETW_SetPending(void);
void ETW_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the ETW ISR. */
#define ETW_INTC_VECTOR            ((reg32 *) ETW__INTC_VECT)

/* Address of the ETW ISR priority. */
#define ETW_INTC_PRIOR             ((reg8 *) ETW__INTC_PRIOR_REG)

/* Priority of the ETW interrupt. */
#define ETW_INTC_PRIOR_NUMBER      ETW__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable ETW interrupt. */
#define ETW_INTC_SET_EN            ((reg32 *) ETW__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the ETW interrupt. */
#define ETW_INTC_CLR_EN            ((reg32 *) ETW__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the ETW interrupt state to pending. */
#define ETW_INTC_SET_PD            ((reg32 *) ETW__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the ETW interrupt. */
#define ETW_INTC_CLR_PD            ((reg32 *) ETW__INTC_CLR_PD_REG)


#endif /* CY_ISR_ETW_H */


/* [] END OF FILE */
