/*******************************************************************************
* File Name: TEST.h  
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

#if !defined(CY_PINS_TEST_H) /* Pins TEST_H */
#define CY_PINS_TEST_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "TEST_aliases.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component cy_pins_v1_90 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 TEST__PORT == 15 && ((TEST__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

void    TEST_Write(uint8 value) ;
void    TEST_SetDriveMode(uint8 mode) ;
uint8   TEST_ReadDataReg(void) ;
uint8   TEST_Read(void) ;
uint8   TEST_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define TEST_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define TEST_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define TEST_DM_RES_UP          PIN_DM_RES_UP
#define TEST_DM_RES_DWN         PIN_DM_RES_DWN
#define TEST_DM_OD_LO           PIN_DM_OD_LO
#define TEST_DM_OD_HI           PIN_DM_OD_HI
#define TEST_DM_STRONG          PIN_DM_STRONG
#define TEST_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define TEST_MASK               TEST__MASK
#define TEST_SHIFT              TEST__SHIFT
#define TEST_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define TEST_PS                     (* (reg8 *) TEST__PS)
/* Data Register */
#define TEST_DR                     (* (reg8 *) TEST__DR)
/* Port Number */
#define TEST_PRT_NUM                (* (reg8 *) TEST__PRT) 
/* Connect to Analog Globals */                                                  
#define TEST_AG                     (* (reg8 *) TEST__AG)                       
/* Analog MUX bux enable */
#define TEST_AMUX                   (* (reg8 *) TEST__AMUX) 
/* Bidirectional Enable */                                                        
#define TEST_BIE                    (* (reg8 *) TEST__BIE)
/* Bit-mask for Aliased Register Access */
#define TEST_BIT_MASK               (* (reg8 *) TEST__BIT_MASK)
/* Bypass Enable */
#define TEST_BYP                    (* (reg8 *) TEST__BYP)
/* Port wide control signals */                                                   
#define TEST_CTL                    (* (reg8 *) TEST__CTL)
/* Drive Modes */
#define TEST_DM0                    (* (reg8 *) TEST__DM0) 
#define TEST_DM1                    (* (reg8 *) TEST__DM1)
#define TEST_DM2                    (* (reg8 *) TEST__DM2) 
/* Input Buffer Disable Override */
#define TEST_INP_DIS                (* (reg8 *) TEST__INP_DIS)
/* LCD Common or Segment Drive */
#define TEST_LCD_COM_SEG            (* (reg8 *) TEST__LCD_COM_SEG)
/* Enable Segment LCD */
#define TEST_LCD_EN                 (* (reg8 *) TEST__LCD_EN)
/* Slew Rate Control */
#define TEST_SLW                    (* (reg8 *) TEST__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define TEST_PRTDSI__CAPS_SEL       (* (reg8 *) TEST__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define TEST_PRTDSI__DBL_SYNC_IN    (* (reg8 *) TEST__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define TEST_PRTDSI__OE_SEL0        (* (reg8 *) TEST__PRTDSI__OE_SEL0) 
#define TEST_PRTDSI__OE_SEL1        (* (reg8 *) TEST__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define TEST_PRTDSI__OUT_SEL0       (* (reg8 *) TEST__PRTDSI__OUT_SEL0) 
#define TEST_PRTDSI__OUT_SEL1       (* (reg8 *) TEST__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define TEST_PRTDSI__SYNC_OUT       (* (reg8 *) TEST__PRTDSI__SYNC_OUT) 


#if defined(TEST__INTSTAT)  /* Interrupt Registers */

    #define TEST_INTSTAT                (* (reg8 *) TEST__INTSTAT)
    #define TEST_SNAP                   (* (reg8 *) TEST__SNAP)

#endif /* Interrupt Registers */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_TEST_H */


/* [] END OF FILE */
