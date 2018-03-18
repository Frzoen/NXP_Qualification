#include "derivative.h"
#include "uart.h"
#include "gpio.h"

/**************************
 * Defines
 **************************/
#define BIT(x)	(1 << (x))
//#define TIMER_STATUS	BIT(22)	example
#define UART2_C2_TIE_MASK	0x80
#define UART2_C2_RIE_MASK	0x20
#define UART2_S1_TDRE_MASK	0x80
#define UART2_S1_RDRF_MASK	0x20
#define BUFFER_LENGTH		255

/**************************
 * Variables definitions
 **************************/
static volatile uint8_t byteReceived;
static volatile uint8_t readyToTransmit = 1;
static volatile uint8_t flagIsReceived;
static volatile uint8_t bufferTransmit[BUFFER_LENGTH];

static volatile uint8_t queueHead = 0;
static volatile uint8_t queueTail;



/***********************************************************************
 * 
 * Function:	initUART2
 * 
 * Description: Initialize the serial port UART2.
 * 
 * Notes: 		
 * 
 * Returns:		None. 
 * 
 ***********************************************************************/
void initUART2(void) {
		
	/*	Set UART 2 GPIO */	
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;						// 
	PORTE_PCR22 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK;		// PTE22 UART2_TX
	PORTE_PCR23 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK;		// PTE23 UART2_RX
	
	/*	Enable UART2 clock */
	SIM_SCGC4 = SIM_SCGC4_UART2_MASK;	
	
	/*	Set UART2 baud rate
	 * 	The baud rate equals (UART Module Clock) / (SBR[12:0] x 16)
	 * 	Module Clock equals bus clock. Bus clock is set in mcg.c for 24 MHz.
	 * 	(SBR[12:0] = (UART Module Clock) / ((baud rate) x 16)
	 * 	SBR[12:0] = 13
	 */
	UART2_BDL = UART_BDL_SBR(13);	// 156 - 9600 bps
	UART2_BDH = UART_BDH_SBR(0);
	
	/*	Set Hardware interrupt requested when TDRE flag is 1 */
	//UART2_C2 |= UART2_C2_TIE_MASK;
	
	/*	Receiver Interrupt Enable for RDRF
	 * 		1 Hardware interrupt requested when RDRF flag is 1.
	 */
	UART2_C2 |= UART2_C2_RIE_MASK;
	
	/* Set the ICPR and ISER registers accordingly */
	NVIC_ICPR = 1 << ((INT_UART2 -16)%32);
	NVIC_ISER = 1 << ((INT_UART2 -16)%32);
	
	/*	Enable Transmitter */
	UART2_C2 |= UART_C2_TE_MASK;
	
	/*	Enable Receiver */
	UART2_C2 |= UART_C2_RE_MASK;
	
}

/***********************************************************************
 * 
 * Function:	transmitByte 
 * 
 * Description: Send a byte via the serial port.
 * 
 * Notes: 		Firstly, the function checks if the serial port
 * 				is ready to transmit. If so, ready to transmit flag
 * 				is cleared, transmitter interrupt is set and the given 
 * 				byte is send to the data register. If the readyToTransmit
 * 				flag is cleared at the beginning of a function, it will
 * 				skip the transmitting.
 * 
 * Returns:		None. 
 * 
 ***********************************************************************/ 
void transmitByte(uint8_t byte) {
	/*if (isReadyToTransmit) {
		readyToTransmit = 0;
		UART2_C2 |= UART2_C2_TIE_MASK;
		UART2_D = byte;
	}	*/	
}

/***********************************************************************
 * 
 * Function:	isReadyToTransmit 
 * 
 * Description: API function.
 * 
 * Notes: 		
 * 
 * Returns:		1 - if the data register is ready to get a new data
 * 				0 - if the data register isn't ready to get a new data 
 * 
 ***********************************************************************/
int isReadyToTransmit(void) {
	return readyToTransmit;
}

/***********************************************************************
 * 
 * Function:	isReceived
 * 
 * Description: Check if there is a new received byte.
 * 
 * Notes: 		API function.
 * 
 * Returns:		1 - 
 * 				0 - 
 * 
 ***********************************************************************/
int isReceived(void) {
	return flagIsReceived;
}

/***********************************************************************
 * 
 * Function:	getReceived
 * 
 * Description: Writes last received byte to a given variable
 * 
 * Notes: 		API function.
 * 
 * Returns:		Last received byte. 
 * 
 ***********************************************************************/
char getReceived(void) {	
	flagIsReceived = 0;
	return byteReceived;
}

/***********************************************************************
 * 
 * Function:	printt
 * 
 * Description:	
 * 
 * Notes:		
 * 
 * Returns:		
 ***********************************************************************/
void printt(char buffer[]) {
	
	/* Count the number of chars */ 
	int length = 0;
	while (buffer[length] != 0) {
		bufferTransmit[(queueTail + length) % BUFFER_LENGTH] = buffer[length];
		length++;
	}

	/* Compute a new queueTail value */
	queueTail = ((queueTail + length) % BUFFER_LENGTH);
		
	/* Enable interrupts for TDRE*/
	UART2_C2 |= UART2_C2_TIE_MASK;
}

/***********************************************************************
 * 
 * Function:	UART2_IRQHandler
 * 
 * Description: UART interrupt hander.
 * 
 * Notes: 		
 * 
 * Returns:		None. 
 * 
 ***********************************************************************/
void UART2_IRQHandler() {
	
	/*	Interrupt from RX pin */
	if (UART2_S1 & UART2_S1_RDRF_MASK) {
		byteReceived = UART2_D;
		flagIsReceived = 1;
	}
	/* Interrupt from Transmit Data Register Empty Flag */
	else if (UART2_S1 & UART2_S1_TDRE_MASK) {				
		
		/* Disable interrupts if everything sent */
		if (queueTail == queueHead) {
			UART2_C2 &= ~(UART2_C2_TIE_MASK);
		}
		else {
			UART2_D = bufferTransmit[queueHead];
			queueHead = ((queueHead + 1) % BUFFER_LENGTH);
		}
	}
}

