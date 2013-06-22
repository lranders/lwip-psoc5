/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#ifndef DMA_H
#define DMA_H

/* DMA Configuration for DMA_TX */
#define DMA_TX_BYTES_PER_BURST       (1u)
#define DMA_TX_REQUEST_PER_BURST     (1u)
#define DMA_TX_SRC_BASE              (CYDEV_SRAM_BASE)
#define DMA_TX_DST_BASE              (CYDEV_PERIPH_BASE)

/* DMA Configuration for DMA_RX */
#define DMA_RX_BYTES_PER_BURST       (1u)
#define DMA_RX_REQUEST_PER_BURST     (1u)
#define DMA_RX_SRC_BASE              (CYDEV_PERIPH_BASE)
#define DMA_RX_DST_BASE              (CYDEV_SRAM_BASE)

/* Variable declarations for DMA_Tx */
extern uint8 TXChannel;
extern uint8 TX_TD[2];

/* Variable declarations for DMA_Rx */
extern uint8 RXChannel;
extern uint8 RX_TD[2];

#endif

//[] END OF FILE
