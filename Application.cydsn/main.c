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
#include <device.h>
//#include <dma.h>
//#include <types.h>
#include <stdio.h>
#include <enc28j60.h>
#include <lwip/init.h>
#include <lwip/netif.h>
#include <lwip/dhcp.h>
#include <lwip/tcp_impl.h>
#include <netif/etharp.h>
#include <crc16.h>
#include <eeprom.h>

volatile unsigned char g_ucFlags = 0;
volatile unsigned long g_ulTickCounter = 0;
volatile unsigned long g_ulIntCounter = 0;

#define FLAG_SYSTICK 0
#define FLAG_RXPKT 1
#define FLAG_TXPKT 2
#define FLAG_RXPKTPEND 3
#define FLAG_ENC_INT 4
#define FLAG_USE_DHCP 5

#define TICK_MS 50

const epmLayout_t epmSaneValues = {
	.macAddress = {0x00,0x0d,0x65,0x2a,0x84,0x80},
	.ipAddress = {192,168,36,92},
	.netMask = {255,255,255,0},
	.gateWay = {192,168,36,88},
	.u.bits.useDhcp = 1
};

#define HB_RESET 186000
uint32_t hb_counter = HB_RESET;

void heartbeat(void)
{
	if(--hb_counter == 0) {
		HB_Write(~HB_Read());
		hb_counter = HB_RESET;
	}
}


char8 *cnvdat[] = { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B", "C", "D", "E", "F" };
void outHx(uint8 dat)
{
	URT_PutChar(cnvdat[(dat & 0xf0) >> 4][0]);
	URT_PutChar(cnvdat[dat & 0x0f][0]);
}

uint8_t epmWork[32];

#define EPM(mbr) ((epmLayout_t *)epm)->mbr

void main()
{
	struct netif netif;
	ip_addr_t ipaddr, netmask, gw;
    uint8_t *epm = (uint8_t*) CYDEV_EE_BASE;
	epmLayout_t *ptrEpmWork = (epmLayout_t *) &epmWork;

	URT_Start();
	printf("main: start\n");

	EPM_Start();
	printf("EPM layout len = %d, EPM size = %d\n", sizeof(epmLayout_t), EPM_EEPROM_SIZE);
	memset(ptrEpmWork, 0, sizeof(epmWork));
	memcpy(ptrEpmWork, epm, sizeof(epmLayout_t));

	uint8_t i;
	for(i=0; i<32; i++) {
		if((i>0) && (i%4==0)) URT_PutChar(' ');
		outHx(epm[i]);
	}
	printf("\n");

	uint16_t epmck = CalcChecksum16((uint8_t*) ptrEpmWork, sizeof(epmLayout_t)-2);
	uint16_t epmsv = CalcChecksum16((uint8_t*) &epmSaneValues, sizeof(epmLayout_t)-2);
	printf("EPM cksm: %d, calculated: %d\n", ptrEpmWork->checkSum, epmck);
	printf("EPM sane values checksum: %d\n", epmsv);
	if(ptrEpmWork->checkSum != epmck) {
		printf("main: epm checksum failed. using sane values\n");
		memcpy(ptrEpmWork, &epmSaneValues, sizeof(epmLayout_t));
		ptrEpmWork->checkSum = epmsv;

		if(CySetTemp() == CYRET_SUCCESS) {
			epm = (uint8_t*) ptrEpmWork;
//			printf("0x%08lx\n", (uint32_t) epm);
			EPM_Write(epm, 0u);
			epm += CY_EEPROM_SIZEOF_ROW;
//			printf("0x%08lx\n", (uint32_t) epm);
			EPM_Write(epm, 1u);
			epm -= CY_EEPROM_SIZEOF_ROW;
		}
//		epm = (uint8_t*) CYDEV_EE_BASE;
//		memcpy(ptrEpmWork, epm, sizeof(epmLayout_t));
	}
	printf("EPM mac: %02x:%02x:%02x:%02x:%02x:%02x\n",
		EPM(macAddress)[0], EPM(macAddress)[1], EPM(macAddress)[2],
		EPM(macAddress)[3], EPM(macAddress)[4], EPM(macAddress)[5]);

	LDON_CR_Write(1);
	ETH_Start(); //ethernet SPI link
	ETW_Start(); //ethernet time wheel

	enc_init(EPM(macAddress));
	while(EINT_Read() == 0) {
		printf("EINT stuck at zero\n");
		CyDelay(1000);
		enc_init(EPM(macAddress));
	}
	EINT_ClearInterrupt();
	ETE_Start(); //ethernet interrupt

	if(EPM(u).bits.useDhcp) {
		printf("main: DHCP mode selected\n");
		g_ucFlags |= (1<<FLAG_USE_DHCP);
	}

	LCD_Start();
	LCD_Position(0,0);
	LCD_PrintString("Hello all");
	CyGlobalIntEnable;

	lwip_init();

	if((g_ucFlags & (1<<FLAG_USE_DHCP)) == 0) {
		IP4_ADDR(&ipaddr, EPM(ipAddress)[0], EPM(ipAddress)[1], EPM(ipAddress)[2], EPM(ipAddress)[3]);
		IP4_ADDR(&netmask,  EPM(netMask)[0],   EPM(netMask)[1],   EPM(netMask)[2],   EPM(netMask)[3]);
		IP4_ADDR(&gw,       EPM(gateWay)[0],   EPM(gateWay)[1],   EPM(gateWay)[2],   EPM(gateWay)[3]);
	} else {
		IP4_ADDR(&ipaddr,  0, 0, 0, 0);
		IP4_ADDR(&netmask, 0, 0, 0, 0);
		IP4_ADDR(&gw,      0, 0, 0, 0);
	}

	EPM_Stop(); // No further access to EEPROM possible below this line!

	netif_add(&netif, &ipaddr, &netmask, &gw, NULL, enc28j60_init, ethernet_input);
	netif_set_default(&netif);

	if(g_ucFlags & (1<<FLAG_USE_DHCP)) {
		dhcp_start(&netif);
	} else {
		netif_set_up(&netif);
	}

	unsigned long lastArpTime = 0, lastTcpTime = 0, lastDhcpCoarseTime = 0, lastDhcpFineTime = 0;

	while(1)
	{
		if((g_ulTickCounter - lastArpTime) >= (ARP_TMR_INTERVAL / TICK_MS)) {
			etharp_tmr();
			lastArpTime = g_ulTickCounter;
//			printf("lastArpTime = %ld\n", lastArpTime);
//			printf("intCounter = %ld\n", g_ulIntCounter);
		}
		if((g_ulTickCounter - lastTcpTime) >= (TCP_TMR_INTERVAL / TICK_MS)) {
			tcp_tmr();
			lastTcpTime = g_ulTickCounter;
		}
		if((g_ucFlags & (1<<FLAG_USE_DHCP)) && (g_ulTickCounter - lastDhcpCoarseTime) >= (DHCP_COARSE_TIMER_MSECS / TICK_MS)) {
			dhcp_coarse_tmr();
			lastDhcpCoarseTime = g_ulTickCounter;
		}
		if((g_ucFlags & (1<<FLAG_USE_DHCP)) && (g_ulTickCounter - lastDhcpFineTime) >= (DHCP_FINE_TIMER_MSECS / TICK_MS)) {
			dhcp_fine_tmr();
			lastDhcpFineTime = g_ulTickCounter;
		}
		if((g_ucFlags & (1<<FLAG_ENC_INT)) != 0) {
			enc_action(&netif);
			g_ucFlags &= ~(1<<FLAG_ENC_INT);
		}
		heartbeat();
	}
}

/* [] END OF FILE */
