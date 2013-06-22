/*******************************************************************************
* File Name: EINT.c  
* Version 1.90
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "EINT.h"

/* APIs are not generated for P15[7:6] on PSoC 5 */
#if !(CY_PSOC5A &&\
	 EINT__PORT == 15 && ((EINT__MASK & 0xC0) != 0))


/*******************************************************************************
* Function Name: EINT_Write
********************************************************************************
*
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  None
*  
*******************************************************************************/
void EINT_Write(uint8 value) 
{
    uint8 staticBits = (EINT_DR & (uint8)(~EINT_MASK));
    EINT_DR = staticBits | ((uint8)(value << EINT_SHIFT) & EINT_MASK);
}


/*******************************************************************************
* Function Name: EINT_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to this drive mode.
*
* Return: 
*  None
*
*******************************************************************************/
void EINT_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(EINT_0, mode);
}


/*******************************************************************************
* Function Name: EINT_Read
********************************************************************************
*
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  None
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro EINT_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 EINT_Read(void) 
{
    return (EINT_PS & EINT_MASK) >> EINT_SHIFT;
}


/*******************************************************************************
* Function Name: EINT_ReadDataReg
********************************************************************************
*
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 EINT_ReadDataReg(void) 
{
    return (EINT_DR & EINT_MASK) >> EINT_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(EINT_INTSTAT) 

    /*******************************************************************************
    * Function Name: EINT_ClearInterrupt
    ********************************************************************************
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 EINT_ClearInterrupt(void) 
    {
        return (EINT_INTSTAT & EINT_MASK) >> EINT_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif /* CY_PSOC5A... */

    
/* [] END OF FILE */
