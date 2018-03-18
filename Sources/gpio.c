#include "derivative.h"
#include "gpio.h"

void InitGPIO (void) {
	
	// turn on all port clocks
	SIM_SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
	
	// turn on all UART clocks - not needed in this project
	SIM_SCGC4 = SIM_SCGC4_UART0_MASK | SIM_SCGC4_UART1_MASK | SIM_SCGC4_UART2_MASK;
	
	// turn on all TPM clocks
	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM2_MASK;

	// turn on ADC0 clock
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;

	// set ADC inputs to ADC inputs
	PORTC_PCR2  |= PORT_PCR_MUX(0);		// Camera 1 PTC2 ADC0_SE11
	
	
	// set GPIOs to GPIO function
	PORTC_PCR0  |= PORT_PCR_MUX(1);	// PTC0 Push Button S2
	PORTB_PCR18 |= PORT_PCR_MUX(1);	// PTB18 LED red
	PORTB_PCR19 |= PORT_PCR_MUX(1);	// PTB19 LED green
	PORTD_PCR1  |= PORT_PCR_MUX(1);	// PTD1  LED blue
	PORTB_PCR8  |= PORT_PCR_MUX(1);	// PTB8 Camera SI
	PORTB_PCR9  |= PORT_PCR_MUX(1);	// PTB9 Camera Clock
	
	PORTA_PCR4  = 0;				// Set PTA4 Pin Control Register to Zero
	PORTA_PCR4  |= PORT_PCR_MUX(3);	// PTA4 Motor 1 In 1 (speed) PTA4 TPM0_CH1
	PORTA_PCR5  |= PORT_PCR_MUX(1);	// PTA5 Motor 1 In 2 (direction)
	PORTC_PCR8  |= PORT_PCR_MUX(1);	// PTC8 Motor 2 In 1 (direction)
	PORTC_PCR9  |= PORT_PCR_MUX(3);	// PTC9 Motor 2 In 2 (speed) PTC8 TPM0_CH5
	
	// set GPIO outputs to outputs
	GPIOB_PDDR |= (1<<18);			// PTB18 LED red
	GPIOB_PDDR |= (1<<19);			// PTB19 LED green
	GPIOD_PDDR |= (1<<1);			// PTD1  LED blue
	GPIOB_PDDR |= (1<<8);			// PTB8 Camera SI
	GPIOB_PDDR |= (1<<9);			// PTB9 Camera Clock
	GPIOA_PDDR |= (1<<4);			// PTA4 Motor 1 In 1
	GPIOA_PDDR |= (1<<5);			// PTA5 Motor 1 In 2 (direction)
	GPIOC_PDDR |= (1<<8);			// PTC9 Motor 2 In 1 (direction)
	GPIOC_PDDR |= (1<<9);			// PTC9 Motor 2 In 2 
	
	// all LEDs off
	GPIOB_PDOR |= GPIO_PDOR_PDO(1<<18);   // red LED off
	GPIOB_PDOR |= GPIO_PDOR_PDO(1<<19);   // green LED off
	GPIOD_PDOR |= GPIO_PDOR_PDO(1<<1);    // blue LED off	
	
	// GPIOB_PDOR &= ~GPIO_PDOR_PDO(1<<18);  // red LED on
	// GPIOB_PDOR &= ~GPIO_PDOR_PDO(1<<19);  // green LED on
	// GPIOD_PDOR &= ~GPIO_PDOR_PDO(1<<1);   // blue LED on	
			
	// set GPIO input to input
	GPIOC_PDDR &= ~ (1<<0);			// PTC0 Push Button S2
	
	// set PWM outputs
	PORTA_PCR12 |= PORT_PCR_MUX(3);	// Servo Motor 1 Out  PTA12 TPM1_CH0
}
