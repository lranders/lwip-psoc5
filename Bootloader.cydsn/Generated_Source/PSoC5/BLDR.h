/*******************************************************************************
* File Name: BLDR.h
* Version 1.10
*
*  Description:
*   Provides an API for the Bootloader. The API includes functions for starting
*   boot loading operations, validating the application and jumping to the
*   application.
*
********************************************************************************
* Copyright 2008-2013, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_BOOTLOADER_BLDR_H)
#define CY_BOOTLOADER_BLDR_H

#include "device.h"
#include <string.h>


/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component Bootloader_v1_10 requires cy_boot v3.0 or later
#endif /* (CY_ PSOC5X) */

/* Map existing call to the instance specific one */
#define CyBtldr_Start   BLDR_Start

#define BLDR_DUAL_APP_BOOTLOADER        (0)
#define BLDR_BOOTLOADER_APP_VERSION     (0)
#define BLDR_FAST_APP_VALIDATION        (0)
#define BLDR_PACKET_CHECKSUM_CRC        (0)
#define BLDR_WAIT_FOR_COMMAND           (1)
#define BLDR_WAIT_FOR_COMMAND_TIME      (255)
#define BLDR_BOOTLOADER_APP_VALIDATION  (1)

#define BLDR_CMD_GET_FLASH_SIZE_AVAIL   (1)
#define BLDR_CMD_ERASE_ROW_AVAIL        (1)
#define BLDR_CMD_VERIFY_ROW_AVAIL       (1)
#define BLDR_CMD_SYNC_BOOTLOADER_AVAIL  (1)
#define BLDR_CMD_SEND_DATA_AVAIL        (1)

#if(0u != BLDR_DUAL_APP_BOOTLOADER)
    #define BLDR_CMD_GET_APP_STATUS_AVAIL   (1)
#endif  /* (0u != BLDR_DUAL_APP_BOOTLOADER) */

#if (CY_PSOC4)
    #define BLDR_GET_REG16(x)   ((uint16)(                                                                   \
                                                (( uint16 )(( uint16 )CY_GET_XTND_REG8((x)     )       ))   |   \
                                                (( uint16 )(( uint16 )CY_GET_XTND_REG8((x) + 1u) <<  8u))       \
                                            ))


    #define BLDR_GET_REG32(x)   (                                                                    \
                                                (( uint32 )(( uint32 ) CY_GET_XTND_REG8((x)     )       ))   |   \
                                                (( uint32 )(( uint32 ) CY_GET_XTND_REG8((x) + 1u) <<  8u))   |   \
                                                (( uint32 )(( uint32 ) CY_GET_XTND_REG8((x) + 2u) << 16u))   |   \
                                                (( uint32 )(( uint32 ) CY_GET_XTND_REG8((x) + 3u) << 24u))       \
                                            )
#endif  /* (CY_PSOC4) */

/*******************************************************************************
* A common code for the Bootloader and Bootloadable
*******************************************************************************/

/* Mask used to indicate starting application */
#define BLDR_START_APP                      0x80

 /* Mask used to indicate starting bootloader */
#define BLDR_START_BTLDR                    0x40


#ifndef CYDEV_FLASH_BASE
    #define CYDEV_FLASH_BASE                            CYDEV_FLS_BASE
    #define CYDEV_FLASH_SIZE                            CYDEV_FLS_SIZE
#endif /* CYDEV_FLASH_BASE */


#define BLDR_META_DATA_SIZE                 64
#define BLDR_META_APP_CHECKSUM_OFFSET       0


#if(CY_PSOC3)

    #define BLDR_APP_ADDRESS                uint16
    #define BLDR_GET_CODE_DATA(idx)         *((uint8  CYCODE *) idx)
    #define BLDR_GET_CODE_WORD(idx)         *((uint32 CYCODE *) idx)

    /* Offset by 2 from 32 bit start because only need 16 bits */
    #define BLDR_META_APP_ADDR_OFFSET           3       /* 2 bytes */
    #define BLDR_META_APP_BL_LAST_ROW_OFFSET    7       /* 4 bytes */
    #define BLDR_META_APP_BYTE_LEN_OFFSET       11      /* 4 bytes */
    #define BLDR_META_APP_RUN_TYPE_OFFSET       15      /* 1 byte  */

#else

    #define BLDR_APP_ADDRESS                uint32
    #define BLDR_GET_CODE_DATA(idx)         *((uint8  *)(CYDEV_FLASH_BASE + (idx)))
    #define BLDR_GET_CODE_WORD(idx)         *((uint32 *)(CYDEV_FLASH_BASE + (idx)))

    #define BLDR_META_APP_ADDR_OFFSET           1   /* 4 bytes */
    #define BLDR_META_APP_BL_LAST_ROW_OFFSET    5   /* 4 bytes */
    #define BLDR_META_APP_BYTE_LEN_OFFSET       9   /* 4 bytes */
    #define BLDR_META_APP_RUN_TYPE_OFFSET       13  /* 1 byte  */

#endif /* (CY_PSOC3) */

#define BLDR_META_APP_ACTIVE_OFFSET             16  /* 1 byte  */
#define BLDR_META_APP_VERIFIED_OFFSET           17  /* 1 byte  */

#define BLDR_META_APP_BL_BUILD_VER_OFFSET       18  /* 2 bytes */
#define BLDR_META_APP_ID_OFFSET                 20  /* 2 bytes */
#define BLDR_META_APP_VER_OFFSET                22  /* 2 bytes */
#define BLDR_META_APP_CUST_ID_OFFSET            24  /* 4 bytes */


#if (CY_PSOC4)
    #if (__ARMCC_VERSION)
       extern volatile uint32 __attribute__ ((section(".bootloaderruntype"), zero_init))  cyBtldrRunType;
    #else   /* (__GNUC__) */
       extern volatile uint32 __attribute__ ((section(".bootloaderruntype")))             cyBtldrRunType;
    #endif  /* (__ARMCC_VERSION) */
#endif  /* (CY_PSOC4) */


#if(CY_PSOC4)

    extern uint8 appRunType;

    #define BLDR_SOFTWARE_RESET CY_SET_REG32(CYREG_CM0_AIRCR, 0x05FA0004)

    #define BLDR_RES_CAUSE_RESET_SOFT   (0x10u)
    #define BLDR_GET_RUN_TYPE           \
                        (((CY_GET_REG32(CYREG_RES_CAUSE) & BLDR_RES_CAUSE_RESET_SOFT) > 0u) \
                            ? (cyBtldrRunType) \
                            : 0u)


    #define BLDR_SET_RUN_TYPE(x)                (cyBtldrRunType = (x))

#else

    #define BLDR_SOFTWARE_RESET CY_SET_REG8(CYREG_RESET_CR2, 0x01)

    #define BLDR_GET_RUN_TYPE   (CY_GET_REG8(CYREG_RESET_SR0) & \
                                            (BLDR_START_BTLDR | BLDR_START_APP))

    #if defined(WORKAROUND_OPT_XRES)
        #define BLDR_SET_RUN_TYPE(x)     BLDR_SetFlashByte(BLDR_MD_APP_RUN_ADDR(BLDR_activeApp), (x))
    #else
        #define BLDR_SET_RUN_TYPE(x)     CY_SET_REG8(CYREG_RESET_SR0, (x))
    #endif  /* defined(WORKAROUND_OPT_XRES) */

#endif  /* (CY_PSOC4) */


#define BLDR_SetFlashRunType(runType)        BLDR_SetFlashByte(BLDR_MD_APP_RUN_ADDR(0), (runType))


void BLDR_SetFlashByte(uint32 address, uint8 value) ;


/*******************************************************************************
* Metadata addresses and pointer defines
*******************************************************************************/

#define BLDR_META_BASE(x)                 (CYDEV_FLASH_BASE + (CYDEV_FLASH_SIZE - ((x) * CYDEV_FLS_ROW_SIZE) - BLDR_META_DATA_SIZE))
#define BLDR_META_ARRAY                   (BLDR_FLASH_NUMBER_SECTORS - 1)

#define BLDR_META_APP_ENTRY_POINT_ADDR(x) (BLDR_META_BASE(x) + BLDR_META_APP_ADDR_OFFSET)
#define BLDR_META_APP_BYTE_LEN(x)         (BLDR_META_BASE(x) + BLDR_META_APP_BYTE_LEN_OFFSET)
#define BLDR_META_APP_RUN_ADDR(x)         (BLDR_META_BASE(x) + BLDR_META_APP_RUN_TYPE_OFFSET)
#define BLDR_META_APP_ACTIVE_ADDR(x)      (BLDR_META_BASE(x) + BLDR_META_APP_ACTIVE_OFFSET)
#define BLDR_META_APP_VERIFIED_ADDR(x)    (BLDR_META_BASE(x) + BLDR_META_APP_VERIFIED_OFFSET)
#define BLDR_META_APP_BLDBL_VER_ADDR(x)   (BLDR_META_BASE(x) + BLDR_META_APP_BL_BUILD_VER_OFFSET)
#define BLDR_META_APP_VER_ADDR(x)         (BLDR_META_BASE(x) + BLDR_META_APP_VER_OFFSET)
#define BLDR_META_APP_ID_ADDR(x)          (BLDR_META_BASE(x) + BLDR_META_APP_ID_OFFSET)
#define BLDR_META_APP_CUST_ID_ADDR(x)     (BLDR_META_BASE(x) + BLDR_META_APP_CUST_ID_OFFSET)
#define BLDR_META_LAST_BLDR_ROW_ADDR(x)   (BLDR_META_BASE(x) + BLDR_META_APP_BL_LAST_ROW_OFFSET)
#define BLDR_META_CHECKSUM_ADDR(x)        (BLDR_META_BASE(x) + BLDR_META_APP_CHECKSUM_OFFSET)


#if(0u == BLDR_DUAL_APP_BOOTLOADER)

    #define BLDR_MD_BASE                    BLDR_META_BASE(0)
    #define BLDR_MD_ROW                     ((CY_FLASH_NUMBER_ROWS / BLDR_FLASH_NUMBER_SECTORS) - 1)
    #define BLDR_MD_CHECKSUM_ADDR           BLDR_META_CHECKSUM_ADDR(0)
    #define BLDR_MD_LAST_BLDR_ROW_ADDR      BLDR_META_LAST_BLDR_ROW_ADDR(0)
    #define BLDR_MD_APP_BYTE_LEN            BLDR_META_APP_BYTE_LEN(0)
    #define BLDR_MD_APP_VERIFIED_ADDR       BLDR_META_APP_VERIFIED_ADDR(0)
    #define BLDR_MD_APP_ENTRY_POINT_ADDR    BLDR_META_APP_ENTRY_POINT_ADDR(0)
    #define BLDR_MD_APP_RUN_ADDR            BLDR_META_APP_RUN_ADDR(0)

#else

    #define BLDR_MD_ROW(x)                  ((CY_FLASH_NUMBER_ROWS / BLDR_FLASH_NUMBER_SECTORS) - 1 - (x))
    #define BLDR_MD_CHECKSUM_ADDR           BLDR_META_CHECKSUM_ADDR(appId)
    #define BLDR_MD_LAST_BLDR_ROW_ADDR      BLDR_META_LAST_BLDR_ROW_ADDR(appId)
    #define BLDR_MD_APP_BYTE_LEN            BLDR_META_APP_BYTE_LEN(appId)
    #define BLDR_MD_APP_VERIFIED_ADDR       BLDR_META_APP_VERIFIED_ADDR(appId)
    #define BLDR_MD_APP_ENTRY_POINT_ADDR    BLDR_META_APP_ENTRY_POINT_ADDR(BLDR_activeApp)
    #define BLDR_MD_APP_RUN_ADDR            BLDR_META_APP_RUN_ADDR(BLDR_activeApp)

#endif  /* (0u == BLDR_DUAL_APP_BOOTLOADER) */

/* Pointers to the Bootloader metadata items */
#define BLDR_P_APP_ACTIVE(x)             ((uint8 CYCODE *)        BLDR_META_APP_ACTIVE_ADDR(x))

#define BLDR_MD_PTR_CHECKSUM                 ((uint8  CYCODE *)       BLDR_MD_CHECKSUM_ADDR)
#define BLDR_MD_PTR_APP_ENTRY_POINT          ((BLDR_APP_ADDRESS CYCODE *) BLDR_MD_APP_ENTRY_POINT_ADDR)
#define BLDR_MD_PTR_LAST_BLDR_ROW            ((uint16 CYCODE *)       BLDR_MD_LAST_BLDR_ROW_ADDR)
#define BLDR_MD_PTR_APP_BYTE_LEN             ((BLDR_APP_ADDRESS CYCODE *) BLDR_MD_APP_BYTE_LEN)
#define BLDR_MD_PTR_APP_RUN_ADDR             ((uint8  CYCODE *)        BLDR_MD_APP_RUN_ADDR)
#define BLDR_MD_PTR_APP_VERIFIED             ((uint8  CYCODE *)        BLDR_MD_APP_VERIFIED_ADDR)
#define BLDR_MD_PTR_APP_BLD_BL_VER           ((uint16 CYCODE *)       BLDR_MD_APP_BLDBL_VER_ADDR)
#define BLDR_MD_PTR_APP_VER                  ((uint16 CYCODE *)       BLDR_MD_APP_VER_ADDR)
#define BLDR_MD_PTR_APP_ID                   ((uint16 CYCODE *)       BLDR_MD_APP_ID_ADDR)
#define BLDR_MD_PTR_APP_CUST_ID              ((uint32 CYCODE *)       BLDR_MD_APP_CUST_ID_ADDR)


/*******************************************************************************
* External References
*******************************************************************************/
extern void LaunchApp(uint32 addr);
extern void BLDR_Start(void) CYSMALL ;
extern void BLDR_LaunchApplication(void) CYSMALL ;
extern void BLDR_HostLink(uint8 TimeOut) ;

#if(1u == BLDR_DUAL_APP_BOOTLOADER)

    extern cystatus BLDR_ValidateApp(uint8 appId) CYSMALL ;
    #define BLDR_ValidateApplication()   BLDR_ValidateApp(0)

#else

    extern cystatus BLDR_ValidateApplication(void) CYSMALL ;
    #define BLDR_ValidateApp(x) BLDR_ValidateApplication()

#endif  /* (1u == BLDR_DUAL_APP_BOOTLOADER) */


/* If using custom interface as the IO Component, user must provide these functions */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && (CYDEV_BOOTLOADER_IO_COMP == BLDR_Custom_Interface)

    extern void CyBtldrCommStart(void);
    extern void CyBtldrCommStop (void);
    extern void CyBtldrCommReset(void);
    extern cystatus CyBtldrCommWrite(uint8* buffer, uint16 size, uint16* count, uint8 timeOut);
    extern cystatus CyBtldrCommRead (uint8* buffer, uint16 size, uint16* count, uint8 timeOut);

#endif  /* defined(CYDEV_BOOTLOADER_IO_COMP) && (CYDEV_BOOTLOADER_IO_COMP == BLDR_Custom_Interface) */


/*******************************************************************************
* Following code are OBSOLETE and must not be used starting from version 1.10
*******************************************************************************/
#define BLDR_BOOTLOADABLE_APP_VALID     (BLDR_BOOTLOADER_APP_VALIDATION)

#endif /* CY_BOOTLOADER_BLDR_H */


/* [] END OF FILE */
