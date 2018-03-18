#include "derivative.h"
#include "dma.h"

#define BIT(x)	(1 << (x))
#define DMAMUX0_CHCFG_ENBL		BIT(7)
#define DMAMUX0_CHCFG_TRIG		BIT(6)
//#define DMAMUX0_CHCFG_SOURCE	 // TODO

/***********************************************************************
 * 
 * Function:	initDMA 
 * 
 * Description: Initialize the DMA.
 * 
 * Notes: 		Channel 0 in Normal Mode.
 * 
 * Returns:		None. 
 * 
 ***********************************************************************/
void initDMA(void) {

	
	/* Clear DMA 0 and 1 Channels' registers 
	 * TODO check if it works without cleareing registers
	 * after reset, all channels are disabled and must be explicitly enabled before use
	 */
	DMAMUX0_CHCFG0 != ~DMAMUX0_CHCFG_ENBL;
	DMAMUX0_CHCFG1 != ~DMAMUX0_CHCFG_ENBL;

	/* Enable the DMA channel 0 and 1 */
	DMAMUX0_CHCFG0 |= DMAMUX0_CHCFG_ENBL;
	DMAMUX0_CHCFG1 |= DMAMUX0_CHCFG_ENBL;

	/* Set the DMA Channels' Sources (Slots) */
	DMAMUX0_CHCFG0 |= 0b0110;	// UART2, Receive, source number 6
	DMAMUX0_CHCFG1 |= 0b0111;// UART2, Transmit, source number 7

	/* Use an always-enabled DMA source.
	 * 	For this option, the 
	 * 	DMA channel should be enabled and pointing to an "always enabled" source.
	 * 
	 * */
}

