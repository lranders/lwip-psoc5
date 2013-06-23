/*
 * Copyright (c) 2013. All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * Author: Lars Randers <lranders@mail.dk>
 *
 * Credits go to: Paul Fleischer <https://github.com/xpgdk/stellaris-enc28j60-booster.git>
 *                and
 *                Kartik M <kmmankad@gmail.com>
 *
 * who both provided me with an excellent starting point for this driver.
 *
 */
#include <device.h>
#include <ETH.h>

#include "enc28j60.h"
#include "enc28j60reg.h"

#include <stdio.h>
#include <dma.h>

#include <netif/etharp.h>

#define RX_START	0x0000
#define RX_END		0x19ff
#define TX_START	0x1a00
#define TX_END		0x1fff

#define BUF	((struct eth_hdr *)buf)
#define ENC_FDX		0

static uint8_t enc_current_bank;
static uint16_t enc_next_packet;

/* Internal low-level register access functions*/
static uint8_t enc_rcr(uint8_t reg);
static void enc_wcr(uint8_t reg, uint8_t val);
static uint8_t enc_rcr_m(uint8_t reg);
static void enc_rbm(uint8_t *buf, uint16_t count, uint8_t offset);
static void enc_wbm(const uint8_t *buf, uint16_t count);
static void enc_bfs(uint8_t reg, uint8_t mask);
static void enc_bfc(uint8_t reg, uint8_t mask);
static void enc_switch_bank(uint8_t new_bank);

/* Internal high-level register access functions*/
static uint8_t enc_read_reg(uint8_t reg, uint8_t bank);
static void enc_write_reg(uint8_t reg, uint8_t bank, uint8_t value);
static uint8_t enc_read_mreg(uint8_t reg, uint8_t bank);
static void enc_set_bits(uint8_t reg, uint8_t bank, uint8_t mask);
static void enc_clear_bits(uint8_t reg, uint8_t bank, uint8_t mask);

/* Macros for accessing registers.
 * These macros should be used instead of calling the functions directly.
 * They simply pass the register's bank as an argument, so the caller
 * doesn't have to deal with that.
 */
#define READ_REG(reg) enc_read_reg(reg, reg ## _BANK)
#define WRITE_REG(reg, value) enc_write_reg(reg, reg ## _BANK, value)
#define READ_MREG(reg) enc_read_mreg(reg, reg ## _BANK)
#define SET_REG_BITS(reg, mask) enc_set_bits(reg, reg ## _BANK, mask)
#define CLEAR_REG_BITS(reg, mask) enc_clear_bits(reg, reg ## _BANK, mask)

#define ASSERT_CS ECS_Write(0)
#define DEASSERT_CS ECS_Write(1)

static uint16_t enc_phy_read(uint8_t addr);
#if 0
static void enc_set_rx_area(uint16_t start, uint16_t end);
static void enc_set_mac_addr(const uint8_t *mac_addr);
#endif

//static void enc_receive_packet(void);

//DMA globals
uint8_t TXChannel = DMA_INVALID_CHANNEL;
uint8_t TX_TD[2];
uint8_t RXChannel = DMA_INVALID_CHANNEL;
uint8_t RX_TD[2];

void enc_reset(void)
{
	ASSERT_CS;
	ETH_ClearRxBuffer();
	ETH_WriteTxData(RESET_OP);
	while((ETH_ReadTxStatus() & ETH_STS_SPI_DONE) == 0);//wait
	ETH_ReadRxData();
	DEASSERT_CS;
}

/**
 * Read Control Register (RCR)
 */
uint8_t enc_rcr(uint8_t reg)
{
	uint8_t result;

	ASSERT_CS;
	ETH_ClearRxBuffer();
	ETH_WriteTxData(reg & 0x1f);
	ETH_WriteTxData(0xff);
	while((ETH_ReadTxStatus() & ETH_STS_SPI_DONE) == 0);//wait
	ETH_ReadRxData();
	result = ETH_ReadRxData();
	DEASSERT_CS;
	return(result);
}

/**
 * Write Control Register (WCR)
 */
void enc_wcr(uint8_t reg, uint8_t val)
{
	ASSERT_CS;
	ETH_WriteTxData(WCR_OP | (reg & 0x1f));
	ETH_WriteTxData(val);
	while((ETH_ReadTxStatus() & ETH_STS_SPI_DONE) == 0);//wait
	DEASSERT_CS;
}

/**
 * Read Control Register for MAC and MII registers.
 * Reading MAC and MII registers produces an initial dummy
 * byte. Presumably because it takes longer to fetch the values
 * of those registers.
 */
uint8_t enc_rcr_m(uint8_t reg)
{
	uint8_t result;

	ASSERT_CS;
	ETH_ClearRxBuffer();
	ETH_WriteTxData(reg);
	ETH_WriteTxData(0xff); //clock in the dummy byte
	ETH_WriteTxData(0xff); //clock in the real byte
	while((ETH_ReadTxStatus() & ETH_STS_SPI_DONE) == 0);//wait
	ETH_ReadRxData();
	ETH_ReadRxData();
	result = ETH_ReadRxData();
	DEASSERT_CS;
	return(result);
}


/**
 * Read Buffer Memory.
 */
void enc_rbm(uint8_t *buf, uint16_t count, uint8_t offset)
{
	uint8_t opCode[2] = {RBM_OP, 0xff};

	if(count > 0) {
		ETH_ClearRxBuffer();
		CyDmaTdSetConfiguration(TX_TD[0], 1, TX_TD[1], 0);
		CyDmaTdSetConfiguration(TX_TD[1], count, CY_DMA_DISABLE_TD, 0);
		CyDmaTdSetAddress(TX_TD[0], LO16((uint32)&opCode[0]), LO16((uint32)ETH_BSPIM_sR8_Dp_u0__F0_REG));
		CyDmaTdSetAddress(TX_TD[1], LO16((uint32)&opCode[1]), LO16((uint32)ETH_BSPIM_sR8_Dp_u0__F0_REG));
		CyDmaChSetInitialTd(TXChannel, TX_TD[0]); 

		CyDmaTdSetConfiguration(RX_TD[0], offset, RX_TD[1], 0);
		CyDmaTdSetConfiguration(RX_TD[1], count, CY_DMA_DISABLE_TD, TD_INC_DST_ADR);
		CyDmaTdSetAddress(RX_TD[0], LO16((uint32)ETH_BSPIM_sR8_Dp_u0__F1_REG), LO16((uint32)buf));
		CyDmaTdSetAddress(RX_TD[1], LO16((uint32)ETH_BSPIM_sR8_Dp_u0__F1_REG), LO16((uint32)buf));
		CyDmaChSetInitialTd(RXChannel, RX_TD[0]); 

		ASSERT_CS;
		CyDmaChEnable(RXChannel, 1);
		CyDmaChEnable(TXChannel, 1);
		CyDmaChSetRequest(TXChannel, CPU_REQ);

		while((ETH_ReadTxStatus() & ETH_STS_SPI_DONE) == 0);//wait
		DEASSERT_CS;

//		UUART_PutString("RBM:");
//		for(len = 0; len < byt_length; len++) { outHx(wb[len]); UUART_PutChar(' '); }
//		UUART_PutCRLF(0);
	}
}

/**
 * Write Buffer Memory.
 */
void enc_wbm(const uint8_t *buf, uint16_t count)
{
	char8 opCode = WBM_OP;

	if(count > 0) {
		ASSERT_CS;
		CyDmaTdSetConfiguration(TX_TD[0], 1, TX_TD[1], 0);
		CyDmaTdSetConfiguration(TX_TD[1], count, CY_DMA_DISABLE_TD, TD_INC_SRC_ADR);
		CyDmaTdSetAddress(TX_TD[0], LO16((uint32)&opCode), LO16((uint32)ETH_BSPIM_sR8_Dp_u0__F0_REG));
		CyDmaTdSetAddress(TX_TD[1], LO16((uint32)buf), LO16((uint32)ETH_BSPIM_sR8_Dp_u0__F0_REG));
		CyDmaChSetInitialTd(TXChannel, TX_TD[0]); 
		CyDmaChEnable(TXChannel, 1);
		CyDmaChSetRequest(TXChannel, CPU_REQ);

		while((ETH_ReadTxStatus() & ETH_STS_SPI_DONE) == 0);//wait
		DEASSERT_CS;
	}
}

/**
 * Bit Field Set.
 * Set the bits of argument 'mask' in the register 'reg'.
 * Not valid for MAC and MII registers.
 */
void enc_bfs(uint8_t reg, uint8_t mask)
{
	if(reg > 0x1f) return;

	reg |= BFS_OP;//Set the opcode.
	ASSERT_CS;
	ETH_WriteTxData(reg);//Send the opcode and address.
	ETH_WriteTxData(mask);//Send the data.
	while((ETH_ReadTxStatus() & ETH_STS_SPI_DONE) == 0);//wait
	DEASSERT_CS;
}

/**
 * Bit Field Clear.
 * Clear the bits of argument 'mask' in the register 'reg'.
 * Not valid for MAC and MII registers.
 */
void enc_bfc(uint8_t reg, uint8_t mask)
{
	if(reg > 0x1f) return;

	reg |= BFC_OP;//Set the opcode.
	ASSERT_CS;
	ETH_WriteTxData(reg);//Send the opcode and address.
	ETH_WriteTxData(mask);//Send the data.
	while((ETH_ReadTxStatus() & ETH_STS_SPI_DONE) == 0);//wait
	DEASSERT_CS;
}

/**
 * Switch memory bank to 'new_bank'
 */
void enc_switch_bank(uint8_t new_bank)
{
	if (new_bank == enc_current_bank || new_bank == ANY_BANK) {
		return;
	}
	uint8_t econ1 = enc_rcr(ENC_ECON1);

	econ1 &= ~ENC_ECON1_BSEL_MASK;
	econ1 |= (new_bank & ENC_ECON1_BSEL_MASK) << ENC_ECON1_BSEL_SHIFT;
	enc_wcr(ENC_ECON1, econ1);
	enc_current_bank = new_bank;
}


/**
 * High level register read. Switches bank as appropriate.
 */
uint8_t enc_read_reg(uint8_t reg, uint8_t bank)
{
	if (bank != enc_current_bank) {
		enc_switch_bank(bank);
	}

	return enc_rcr(reg);
}

/**
 * High level bit field set. Switches bank as appropriate.
 */
void enc_set_bits(uint8_t reg, uint8_t bank, uint8_t mask)
{
	if (bank != enc_current_bank) {
		enc_switch_bank(bank);
	}

	enc_bfs(reg, mask);
}

/**
 * High level bit field clear. Switches bank as appropriate.
 */
void enc_clear_bits(uint8_t reg, uint8_t bank, uint8_t mask)
{
	if (bank != enc_current_bank) {
		enc_switch_bank(bank);
	}

	enc_bfc(reg, mask);
}

/**
 * High level MAC/MII register read. Switches bank as appropriate.
 */
uint8_t enc_read_mreg(uint8_t reg, uint8_t bank)
{
	if (bank != enc_current_bank) {
		enc_switch_bank(bank);
	}

	return enc_rcr_m(reg);
}

/**
 * High level register write. Switches bank as appropriate.
 */
void enc_write_reg(uint8_t reg, uint8_t bank, uint8_t value)
{
	if (bank != enc_current_bank) {
		enc_switch_bank(bank);
	}

	enc_wcr(reg, value);
}

/**
 * Read value from PHY address.
 * Reading procedure is described in ENC28J60 datasheet
 * section 3.3.
 */
uint16_t enc_phy_read(uint8_t addr)
{
	/*
	 1. Write the address of the PHY register to read
	 from into the MIREGADR register.*/
	WRITE_REG(ENC_MIREGADR, addr);

	/*2. Set the MICMD.MIIRD bit. The read operation
	 begins and the MISTAT.BUSY bit is set.*/
	WRITE_REG(ENC_MICMD, 0x1);

	/*3. Wait 10.24 μs. Poll the MISTAT.BUSY bit to be
	 certain that the operation is complete. While
	 busy, the host controller should not start any
	 MIISCAN operations or write to the MIWRH
	 register.
	 When the MAC has obtained the register
	 contents, the BUSY bit will clear itself.*/

	CyDelayUs(12);

	uint8_t stat;
	do {
		stat = READ_MREG(ENC_MISTAT);
	} while (stat & ENC_MISTAT_BUSY);

	/*4. Clear the MICMD.MIIRD bit.*/
	WRITE_REG(ENC_MICMD, 0x00);

	/*5. Read the desired data from the MIRDL and
	 MIRDH registers. The order that these bytes are
	 accessed is unimportant.
	 */
	uint16_t ret;
	ret = READ_MREG(ENC_MIRDL) & 0xFF;
	ret |= READ_MREG(ENC_MIRDH) << 8;

	return ret;
}

/**
 * Write value to PHY address.
 * Reading procedure is described in ENC28J60 datasheet
 * section 3.3.
 */
void enc_phy_write(uint8_t addr, uint16_t value)
{
	WRITE_REG(ENC_MIREGADR, addr);
	WRITE_REG(ENC_MIWRL, value & 0xFF);
	WRITE_REG(ENC_MIWRH, value >> 8);

	CyDelayUs(12);

	uint8_t stat;
	do {
		stat = READ_MREG(ENC_MISTAT);
	} while (stat & ENC_MISTAT_BUSY);
}

/**
 * Set the memory area to use for receiving packets.
 */
#if 0
void enc_set_rx_area(uint16_t start, uint16_t end)
{
	WRITE_REG(ENC_ERXSTL, start & 0xFF);
	WRITE_REG(ENC_ERXSTH, (start >> 8) & 0xFFF);

	WRITE_REG(ENC_ERXNDL, end & 0xFF);
	WRITE_REG(ENC_ERXNDH, (end >> 8) & 0xFFF);

	WRITE_REG(ENC_ERXRDPTL, start & 0xFF);
	WRITE_REG(ENC_ERXRDPTH, (start >> 8) & 0xFFF);
}

/**
 * Set the MAC address.
 */
void enc_set_mac_addr(const uint8_t *mac_addr)
{
	WRITE_REG(ENC_MAADR1, mac_addr[0]);
	WRITE_REG(ENC_MAADR2, mac_addr[1]);
	WRITE_REG(ENC_MAADR3, mac_addr[2]);
	WRITE_REG(ENC_MAADR4, mac_addr[3]);
	WRITE_REG(ENC_MAADR5, mac_addr[4]);
	WRITE_REG(ENC_MAADR6, mac_addr[5]);
}
#endif

/**
 * Read the MAC address.
 */
void enc_get_mac_addr(uint8_t *mac_addr)
{
  mac_addr[0] = READ_MREG(ENC_MAADR1);
  mac_addr[1] = READ_MREG(ENC_MAADR2);
  mac_addr[2] = READ_MREG(ENC_MAADR3);
  mac_addr[3] = READ_MREG(ENC_MAADR4);
  mac_addr[4] = READ_MREG(ENC_MAADR5);
  mac_addr[5] = READ_MREG(ENC_MAADR6);
}

static err_t
enc_low_level_output(struct netif *netif, struct pbuf *p)
{
  uint8_t frame[1514];
  uint8_t *frame_ptr = &frame[0];
  struct pbuf *b;

  for(b = p; b != NULL; b = b->next) {
    //printf("Copying %d bytes from %p to %p\n", b->len, b->payload, frame_ptr);
    memcpy(frame_ptr, b->payload, b->len);
    frame_ptr += b->len;
  }

  enc_send_packet(frame, p->tot_len);

  return ERR_OK;
}

static struct pbuf*
enc_low_level_input(struct netif *netif)
{
  struct pbuf *p = NULL, *q;
  /* Receive a single packet */
  uint8_t header[6];
  uint8_t *status = header + 2;
  
  WRITE_REG(ENC_ERDPTL, enc_next_packet & 0xFF);
  WRITE_REG(ENC_ERDPTH, (enc_next_packet >> 8) & 0xFF);
  enc_rbm(header, 6, 2);

  /* Update next packet pointer */
  enc_next_packet = header[0] | (header[1] << 8);
  
  uint16_t data_count = status[0] | (status[1] << 8);
  if (status[2] & (1 << 7)) {
    uint8_t frame[1514];
    uint8_t *frame_ptr = &frame[0];
    enc_rbm(frame, data_count, 1);

    p = pbuf_alloc(PBUF_LINK, data_count, PBUF_POOL);
    //printf("p: %p\n", p);
    if( p != NULL ) {
      for(q = p; q != NULL; q = q->next) {
        memcpy(q->payload, frame_ptr, q->len);
        frame_ptr += q->len;
      }
    } else {
      printf("pbuf_alloc: out of buffer memory\n");
    }
  }

  uint16_t erxst = READ_REG(ENC_ERXSTL) | (READ_REG(ENC_ERXSTH) << 8);

  /* Mark packet as read */
  if (enc_next_packet == erxst) {
    WRITE_REG(ENC_ERXRDPTL, READ_REG(ENC_ERXNDL));
    WRITE_REG(ENC_ERXRDPTH, READ_REG(ENC_ERXNDH));
  } else {
    WRITE_REG(ENC_ERXRDPTL, (enc_next_packet-1) & 0xFF);
    WRITE_REG(ENC_ERXRDPTH, ((enc_next_packet-1) >> 8) & 0xFF);
  }
  SET_REG_BITS(ENC_ECON2, ENC_ECON2_PKTDEC);
  return p;
}

#ifdef LWIP_NETIF_STATUS_CALLBACK
void
enc28j60_status_callback(struct netif *netif)
{
  if( netif->flags & NETIF_FLAG_UP ) {
    printf("netif_up: Got IP address: %d.%d.%d.%d\n", ip4_addr1_16(&netif->ip_addr), ip4_addr2_16(&netif->ip_addr), ip4_addr3_16(&netif->ip_addr), ip4_addr4_16(&netif->ip_addr));
  }
}
#endif

extern const unsigned char myMAC[6];

err_t enc28j60_init(struct netif *netif)
{
  netif->state = NULL;
  netif->hwaddr_len = 6;
  netif->name[0] = 'e';
  netif->name[1] = 'n';
  netif->output = etharp_output;
  netif->linkoutput = enc_low_level_output;
  netif->mtu = 1500;
  netif->flags = NETIF_FLAG_ETHERNET | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
#ifdef LWIP_NETIF_STATUS_CALLBACK
  netif->status_callback = enc28j60_status_callback;
#endif
  enc_get_mac_addr(netif->hwaddr);
#if 1
  printf("enc28j60_init: mac set to %02x:%02x:%02x:%02x:%02x:%02x\n", netif->hwaddr[0], netif->hwaddr[1], netif->hwaddr[2], netif->hwaddr[3], netif->hwaddr[4], netif->hwaddr[5]);
#endif
  return ERR_OK;
}

/**
 * Initialize the ENC28J60 with the given MAC-address
 */
void enc_init(const uint8_t *mac)
{
	enc_next_packet = 0u;

	if(TXChannel == DMA_INVALID_CHANNEL) {
		// Init DMA, 1 byte bursts, each burst requires a request
		TXChannel = DMA_TX_DmaInitialize(DMA_TX_BYTES_PER_BURST, DMA_TX_REQUEST_PER_BURST, HI16(DMA_TX_SRC_BASE), HI16(DMA_TX_DST_BASE));

		TX_TD[0] = CyDmaTdAllocate();
		TX_TD[1] = CyDmaTdAllocate();

		// Init DMA, 1 byte bursts, each burst requires a request
		RXChannel = DMA_RX_DmaInitialize(DMA_RX_BYTES_PER_BURST, DMA_RX_REQUEST_PER_BURST, HI16(DMA_RX_SRC_BASE), HI16(DMA_RX_DST_BASE));

		RX_TD[0] = CyDmaTdAllocate();
		RX_TD[1] = CyDmaTdAllocate();
		
#if 1
		printf("enc_init: dma initialized (TX=%d, RX=%d)\n", TXChannel, RXChannel);
#endif
	}

	enc_reset();

	uint8_t reg;
	do {
		CyDelay(200);
		reg = READ_REG(ENC_ESTAT);
		printf("ENC_ESTAT: %x\n", reg);
	} while ((reg & ENC_ESTAT_CLKRDY) == 0);


	enc_switch_bank(0);

	printf("ECON1: %x\n", READ_REG(ENC_ECON1));

#if 1
	printf("Silicon Revision: %d\n", READ_REG(ENC_EREVID));
#endif

	SET_REG_BITS(ENC_ECON1, ENC_ECON1_TXRST);
	CLEAR_REG_BITS(ENC_ECON1, ENC_ECON1_RXEN);

	SET_REG_BITS(ENC_ECON2, ENC_ECON2_AUTOINC);

	//enc_set_rx_area(0u, RX_END);
	WRITE_REG(ENC_ERXSTL, ( RX_START & 0x00ff));
	WRITE_REG(ENC_ERXSTH, ((RX_START & 0xff00) >> 8));

	WRITE_REG(ENC_ERXNDL, ( RX_END & 0x00ff));
	WRITE_REG(ENC_ERXNDH, ((RX_END & 0xff00) >> 8));

	WRITE_REG(ENC_ERXRDPTL, ( RX_START & 0x00ff));
	WRITE_REG(ENC_ERXRDPTH, ((RX_START & 0xff00) >> 8));

	//questionable: trying to set a read-only bit here...
	uint16_t phyreg = enc_phy_read(ENC_PHSTAT2);
	phyreg &= ~ENC_PHSTAT2_DPXSTAT;
	enc_phy_write(ENC_PHSTAT2, phyreg);

	//set to half duplex here
	phyreg = enc_phy_read(ENC_PHCON1);
#if ENC_FDX
	phyreg |= ENC_PHCON_PDPXMD;
#else
	phyreg &= ~ENC_PHCON_PDPXMD;
#endif
	enc_phy_write(ENC_PHCON1, phyreg);

	/* Setup receive filter to receive
	 * broadcast, multicast and unicast to the given MAC */
#if 1
	printf("Setting MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif

	WRITE_REG(ENC_MAADR1, mac[0]);
	WRITE_REG(ENC_MAADR2, mac[1]);
	WRITE_REG(ENC_MAADR3, mac[2]);
	WRITE_REG(ENC_MAADR4, mac[3]);
	WRITE_REG(ENC_MAADR5, mac[4]);
	WRITE_REG(ENC_MAADR6, mac[5]);
	//enc_set_mac_addr(mac);

	//WRITE_REG(ENC_ERXFCON, ENC_ERXFCON_UCEN | ENC_ERXFCON_CRCEN | ENC_ERXFCON_BCEN | ENC_ERXFCON_MCEN);
	WRITE_REG(ENC_ERXFCON, ENC_ERXFCON_UCEN | ENC_ERXFCON_CRCEN | ENC_ERXFCON_PMEN);
	WRITE_REG(ENC_EPMM0, 0x3f);
	WRITE_REG(ENC_EPMM1, 0x30);
	WRITE_REG(ENC_EPMCSL, 0x39);
	WRITE_REG(ENC_EPMCSH, 0xf7);

	/* Initialize MAC */
	WRITE_REG(ENC_MACON1, ENC_MACON1_TXPAUS | ENC_MACON1_RXPAUS | ENC_MACON1_MARXEN);

	WRITE_REG(ENC_MACON3, (0x1 << ENC_MACON3_PADCFG_SHIFT) | ENC_MACON3_TXRCEN | /*ENC_MACON3_FULDPX |*/ENC_MACON3_FRMLNEN);

	/* Set max. frame length */
	WRITE_REG(ENC_MAMXFLL, ( MAXFRAMELEN & 0x00ff));
	WRITE_REG(ENC_MAMXFLH, ((MAXFRAMELEN & 0xff00) >> 8));

#if ENC_FDX
#else
	WRITE_REG(ENC_MABBIPG, 0x12); /* back to back interpacket gap. set as per data sheet */
	WRITE_REG(ENC_MAIPGL, 0x12);  /* non back to back interpacket gap. set as per data sheet */
	WRITE_REG(ENC_MAIPGH, 0x0c);
#endif

	WRITE_REG(ENC_ECOCON, 0x00); /* Turn off the clock output pin; less noise */

	enc_phy_write(ENC_PHLCON, 0x347a); //LED control

	READ_REG(ENC_EIR); /* Clear pending interrupts */
	SET_REG_BITS(ENC_EIE, ENC_EIE_INTIE | ENC_EIE_PKTIE);

	CLEAR_REG_BITS(ENC_ECON1, ENC_ECON1_TXRST | ENC_ECON1_RXRST);
	SET_REG_BITS(ENC_ECON1, ENC_ECON1_RXEN);

#if 1
	uint8_t mc[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	enc_get_mac_addr(mc);
	printf("Mac addr set to: %02x:%02x:%02x:%02x:%02x:%02x\n", mc[0], mc[1], mc[2], mc[3], mc[4], mc[5]);
#endif
}


/**
 * Handle events from the ENC28J60.
 *
 * Note we can't use the EIR_PKTIF bit because it's unreliable (errata #6)
 */
void enc_action(struct netif *netif)
{
	struct pbuf* p;
//	uint8_t reg = READ_REG(ENC_EIR);

//	if (reg & ENC_EIR_PKTIF) {
		while (READ_REG(ENC_EPKTCNT) > 0) {
			p = enc_low_level_input(netif);
			if( p != NULL ) netif->input(p, netif);
		}
//	}
}

/**
 * Send an ethernet packet. Function will block until
 * transmission has completed.
 * TODO: Return if the transmission was successful or not
 */
void enc_send_packet(const uint8_t *buf, uint16_t count)
{
  uint8_t control = 0;

  WRITE_REG(ENC_ETXSTL, ( TX_START & 0x00ff));
  WRITE_REG(ENC_ETXSTH, ((TX_START & 0xff00) >> 8));

  WRITE_REG(ENC_EWRPTL, ( TX_START & 0x00ff));
  WRITE_REG(ENC_EWRPTH, ((TX_START & 0xff00) >> 8));

#if 0
  printf("enc_send_packet: len = %d\n", count);
  printf("dest: %02x:%02x:%02x:%02x:%02x:%02x\n", BUF->dest.addr[0], BUF->dest.addr[1], BUF->dest.addr[2],  BUF->dest.addr[3], BUF->dest.addr[4], BUF->dest.addr[5]);
  printf("src : %02x:%02x:%02x:%02x:%02x:%02x\n", BUF->src.addr[0], BUF->src.addr[1], BUF->src.addr[2], BUF->src.addr[3], BUF->src.addr[4], BUF->src.addr[5]);
  printf("Type: 0x%04x\n", htons(BUF->type));
#endif

  /*Write the Per Packet Control Byte
  See FIGURE 7-1: FORMAT FOR PER PACKET CONTROL BYTES
  on Page 41 of the datasheet */
  enc_wbm(&control, 1);

  /*Write the packet into the ENC's buffer*/
  enc_wbm(buf, count);

  WRITE_REG(ENC_ETXNDL, ( (count + TX_START) & 0x00ff));
  WRITE_REG(ENC_ETXNDH, (((count + TX_START) & 0xff00) >> 8));

  /* Errata 12 */
  SET_REG_BITS(ENC_ECON1, ENC_ECON1_TXRST);
  CLEAR_REG_BITS(ENC_ECON1, ENC_ECON1_TXRST);
  CLEAR_REG_BITS(ENC_EIR, ENC_EIR_TXERIF | ENC_EIR_TXIF);

  /* Send the packet */
  SET_REG_BITS(ENC_ECON1, ENC_ECON1_TXRTS);

  /* Busy wait for the transmission to complete */
  while (true) {
    uint8_t r = READ_REG(ENC_ECON1);
    if ((r & ENC_ECON1_TXRTS) == 0)
      break;
  }

  /* Clear TXRTS at the end of transmission */
  CLEAR_REG_BITS(ENC_ECON1, ENC_ECON1_TXRTS);

  /* Read status bits */
  uint8_t status[7];
  count++;

  WRITE_REG(ENC_ERDPTL, ( (count + TX_START) & 0x00ff));
  WRITE_REG(ENC_ERDPTH, (((count + TX_START) & 0xff00) >> 8));
  TEST_Write(1);
  enc_rbm(status, 7, 2);
  TEST_Write(0);

#if 0
  //    0           1           2           3           4           5           6
  //0000 0000 : 1111 1100 : 2222 1111 : 3322 2222 : 3333 3333 : 4444 4444 : 5555 5544
  //7654 3210 : 5432 1098 : 3210 9876 : 1098 7654 : 9876 5432 : 7654 3210 : 5432 1098
  printf("transmission vector: %02x %02x %02x %02x %02x %02x %02x\n", status[0], status[1], status[2], status[3], status[4], status[5], status[6]);
  printf("transmit byte count: %d\n", status[0] + (status[1] << 8));
  printf("collision count: %d\n", status[2] & 0x0f);
  printf("crc error: ");
  if(status[2] & 0x10) {printf("yes\n");} else printf("no\n");
  printf("length check error: ");
  if(status[2] & 0x20) {printf("yes\n");} else printf("no\n");
  printf("out of range: ");
  if(status[2] & 0x40) {printf("yes\n");} else printf("no\n");
  printf("done: ");
  if(status[2] & 0x80) {printf("yes\n");} else printf("no\n");
  printf("was multicast: ");
  if(status[3] & 0x01) {printf("yes\n");} else printf("no\n");
  printf("was broadcast: ");
  if(status[3] & 0x02) {printf("yes\n");} else printf("no\n");
  printf("deferred: ");
  if(status[3] & 0x04) {printf("yes\n");} else printf("no\n");
  printf("excessive defer: ");
  if(status[3] & 0x08) {printf("yes\n");} else printf("no\n");
  printf("excessive collision: ");
  if(status[3] & 0x10) {printf("yes\n");} else printf("no\n");
  printf("late collision: ");
  if(status[3] & 0x20) {printf("yes\n");} else printf("no\n");
  printf("giant: ");
  if(status[3] & 0x40) {printf("yes\n");} else printf("no\n");
  printf("underrun: ");
  if(status[3] & 0x80) {printf("yes\n");} else printf("no\n");
  printf("total bytes transmitted: %d\n", status[4] + (status[5] << 8));
  printf("was control frame: ");
  if(status[6] & 0x01) {printf("yes\n");} else printf("no\n");
  printf("pause control frame: ");
  if(status[6] & 0x02) {printf("yes\n");} else printf("no\n");
  printf("backpressure: ");
  if(status[6] & 0x04) {printf("yes\n");} else printf("no\n");
  printf("vlan tagged: ");
  if(status[6] & 0x08) {printf("yes\n");} else printf("no\n");
#endif
//  uint16_t transmit_count = status[0] | (status[1] << 8);

//  if (status[2] & 0x80) {
    /* Transmit OK*/
    //    printf("Transmit OK\n");
//  }
}
/* [] END OF FILE */
