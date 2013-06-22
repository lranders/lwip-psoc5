/*******************************************************************************
* File Name: ECS.h  
* Version 1.90
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_ECS_H) /* Pins ECS_H */
#define CY_PINS_ECS_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "ECS_aliases.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component cy_pins_v1_90 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 ECS__PORT == 15 && ((ECS__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

void    ECS_Write(uint8 value) ;
void    ECS_SetDriveMode(uint8 mode) ;
uint8   ECS_ReadDataReg(void) ;
uint8   ECS_Read(void) ;
uint8   ECS_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define ECS_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define ECS_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define ECS_DM_RES_UP          PIN_DM_RES_UP
#define ECS_DM_RES_DWN         PIN_DM_RES_DWN
#define ECS_DM_OD_LO           PIN_DM_OD_LO
#define ECS_DM_OD_HI           PIN_DM_OD_HI
#define ECS_DM_STRONG          PIN_DM_STRONG
#define ECS_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define ECS_MASK               ECS__MASK
#define ECS_SHIFT              ECS__SHIFT
#define ECS_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define ECS_PS                     (* (reg8 *) ECS__PS)
/* Data Register */
#define ECS_DR                     (* (reg8 *) ECS__DR)
/* Port Number */
#define ECS_PRT_NUM                (* (reg8 *) ECS__PRT) 
/* Connect to Analog Globals */                                                  
#define ECS_AG                     (* (reg8 *) ECS__AG)                       
/* Analog MUX bux enable */
#define ECS_AMUX                   (* (reg8 *) ECS__AMUX) 
/* Bidirectional Enable */                                                        
#define ECS_BIE                    (* (reg8 *) ECS__BIE)
/* Bit-mask for Aliased Register Access */
#define ECS_BIT_MASK               (* (reg8 *) ECS__BIT_MASK)
/* Bypass Enable */
#define ECS_BYP                    (* (reg8 *) ECS__BYP)
/* Port wide control signals */                                                   
#define ECS_CTL                    (* (reg8 *) ECS__CTL)
/* Drive Modes */
#define ECS_DM0                    (* (reg8 *) ECS__DM0) 
#define ECS_DM1                    (* (reg8 *) ECS__DM1)
#define ECS_DM2                    (* (reg8 *) ECS__DM2) 
/* Input Buffer Disable Override */
#define ECS_INP_DIS                (* (reg8 *) ECS__INP_DIS)
/* LCD Common or Segment Drive */
#define ECS_LCD_COM_SEG            (* (reg8 *) ECS__LCD_COM_SEG)
/* Enable Segment LCD */
#define ECS_LCD_EN                 (* (reg8 *) ECS__LCD_EN)
/* Slew Rate Control */
#define ECS_SLW                    (* (reg8 *) ECS__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define ECS_PRTDSI__CAPS_SEL       (* (reg8 *) ECS__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define ECS_PRTDSI__DBL_SYNC_IN    (* (reg8 *) ECS__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define ECS_PRTDSI__OE_SEL0        (* (reg8 *) ECS__PRTDSI__OE_SEL0) 
#define ECS_PRTDSI__OE_SEL1        (* (reg8 *) ECS__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define ECS_PRTDSI__OUT_SEL0       (* (reg8 *) ECS__PRTDSI__OUT_SEL0) 
#define ECS_PRTDSI__OUT_SEL1       (* (reg8 *) ECS__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define ECS_PRTDSI__SYNC_OUT       (* (reg8 *) ECS__PRTDSI__SYNC_OUT) 


#if defined(ECS__INTSTAT)  /* Interrupt Registers */

    #define ECS_INTSTAT                (* (reg8 *) ECS__INTSTAT)
    #define ECS_SNAP                   (* (reg8 *) ECS__SNAP)

#endif /* Interrupt Registers */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_ECS_H */


/* [] END OF FILE */
