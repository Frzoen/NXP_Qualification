/*
 * FRDM-KL25 Freedom Board NXP Cup Minimal Program
 * Version 0.9, 16.03.2018
 *(c) MKNMiR Synergia @ PWr
 */

#include "derivative.h" /* include peripheral declarations */
#include <stdio.h>

// Define threshold for Camera Black Line recognition
#define THRESHOLD				100			// Difference between black and white
// Defines for Direction PD Servo Control Loop
#define KP						50			// Proportional coefficient
#define KDP						25			// Differential coefficient
// Defines for LineScan Camera
#define TAOS_DELAY				asm ("nop")				// minimal delay time
#define	TAOS_SI_HIGH			GPIOB_PDOR |= (1<<8)	// SI on PTB8
#define	TAOS_SI_LOW				GPIOB_PDOR &= ~(1<<8)	// SI on PTB8
#define	TAOS_CLK_HIGH			GPIOB_PDOR |= (1<<9)	// CLK on PTB9
#define	TAOS_CLK_LOW			GPIOB_PDOR &= ~(1<<9)	// CLK on PTB9
#define SERVO_OFFSET 200
#define SERVO_DEAFULT_REGISTER_VALUE 8200 + SERVO_OFFSET
#define SERVO_SAFE_ZONE_REGISTER_VALUE 900
#define SERVO_MINIMAL_REGISTER_VALUE SERVO_DEAFULT_REGISTER_VALUE - SERVO_SAFE_ZONE_REGISTER_VALUE
#define SERVO_MAXIMAL_REGISTER_VALUE SERVO_DEAFULT_REGISTER_VALUE + SERVO_SAFE_ZONE_REGISTER_VALUE

#define SERVO_MINIMAL_VALUE -255
#define SERVO_MAXIMAL_VALUE 255
#define SERVO_MIDDLE_VALUE 0

#define NOWA_POZYCJA_FLAG (1<<0)
#define START_FLAG (1<<1)
#define SRODEK_FLAG (1<<2)
#define PRAWO_FLAG (1<<4)
#define LEWO_FLAG (1<<3)
#define STOP_FLAG (1<<5)
#define DAJ_STAN_FLAG (1<<6)
#define ODDALANIE_FLAG (1<<7)
#define PRZYBLIZANIE_FLAG (1<<8)

#define STRAIGHT_SPEED 	150
#define CURVE_SPEED 	150

#define LINESCAN_TRESHOLD_MAIN 8
#define LINESCAN_TRESHOLD_EDGE 20

#define LINESCAN_MIDDLE 64

#define _KP 20
#define _KD 20
#define _KD_SPEED 5

// Function declaration
void
ImageCapture(void);
int
abs(int);

// Variable declaration
int i;										// counter for loops
int j;										// additional counter
volatile int numberOfDer;					// counter for pos
int temp_reg;								// temporary register
volatile int ImageData[128];				// array to store the LineScan image
volatile int ImageDataDifference[128];// array to store the LineScan pixel difference
int BlackLinePos[4];				// array to store the black line positions
int BlackLinePosOld[4];				// array to store the black line positions
int BlackLineRight;	// position of the black line on the right side   <- leave for backwards compability
int BlackLineLeft;// position of the black line on the left side	  <- leave for backwards compability
int RoadMiddle = 0;							// calculated middle of the road
int diff = 0;					// actual difference from line middle position
int diff_old = 0;				// previous difference from line middle position
int servo_position = 0;		// actual position of the servo relative to middle
int CompareData;						// set data for comparison to find max

volatile uint8_t state = 0x00;				// simple machine state

// functions declaration
void servoWrite(int _serwoValue); 	// not used
void calculateServoLeft(void);		// not used
void calculateServoRight(void);		// not used
void calculateServoNewPos(void);	// not used

// Main program
int main(void)
{

	// configure clock to 48 MHz from a 8 MHz crystal
	MCG_C2 = (MCG_C2_RANGE0(1) | MCG_C2_EREFS0_MASK); // configure the oscillator settings
	MCG_C1 = (MCG_C1_CLKS(2) | MCG_C1_FRDIV(3));	// divider for 8 MHz clock	
	for (i = 0; i < 24000; i++)
		;						// wait for OSCINIT to set
	// now in FBE mode
	MCG_C6 |= MCG_C6_CME0_MASK;		// enable the clock monitor
	MCG_C5 |= MCG_C5_PRDIV0(1); 	// set PLL ref divider to divide by 2
	temp_reg = MCG_C6; // store present C6 value (as CME0 bit was previously set)
	temp_reg &= ~MCG_C6_VDIV0_MASK; // clear VDIV settings
	temp_reg |= MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0); // write new VDIV and enable PLL
	MCG_C6 = temp_reg; 				// update MCG_C6		
	for (i = 0; i < 4000; i++)
		; 	// wait for PLLST status bit to set
	// now in PBE mode
	SIM_CLKDIV1 = (SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV4(1)); // core clock, bus clock div by 2	
	MCG_C1 &= ~MCG_C1_CLKS_MASK; // switch CLKS mux to select the PLL as MCGCLKOUT	
	for (i = 0; i < 2000; i++)
		;	// Wait for clock status bits to update
	// now in PEE mode, core and system clock 48 MHz, bus and flash clock 24 MHz

	// turn on all port clocks
	SIM_SCGC5 =
			SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK
					| SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

	// turn on all UART clocks - not needed in this project
	SIM_SCGC4 = SIM_SCGC4_UART0_MASK | SIM_SCGC4_UART1_MASK
			| SIM_SCGC4_UART2_MASK;

	// turn on all TPM clocks
	SIM_SCGC6 |=
			SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM2_MASK;

	// turn on ADC0 clock
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;

	// set ADC inputs to ADC inputs
	PORTC_PCR2 |= PORT_PCR_MUX(0);	// Camera 1 PTC2 ADC0_SE11

	// set GPIOs to GPIO function
	PORTC_PCR1 |= PORT_PCR_MUX(1);  // PTC1 Signal idicator
	PORTC_PCR0 |= PORT_PCR_MUX(1);	// PTC0 Push Button S2
	PORTB_PCR18 |= PORT_PCR_MUX(1);	// PTB18 LED red
	PORTB_PCR19 |= PORT_PCR_MUX(1);	// PTB19 LED green
	PORTD_PCR1 |= PORT_PCR_MUX(1);	// PTD1  LED blue
	PORTB_PCR8 |= PORT_PCR_MUX(1);	// PTB8 Camera SI
	PORTB_PCR9 |= PORT_PCR_MUX(1);	// PTB9 Camera Clock
	PORTA_PCR4 = 0;					// Set PTA4 Pin Control Register to Zero
	PORTA_PCR4 |= PORT_PCR_MUX(3);	// PTA4 Motor 1 In 1 (speed) PTA4 TPM0_CH1
	PORTA_PCR5 |= PORT_PCR_MUX(1);	// PTA5 Motor 1 In 2 (direction)
	PORTC_PCR8 |= PORT_PCR_MUX(1);	// PTC8 Motor 2 In 1 (direction)
	PORTC_PCR9 |= PORT_PCR_MUX(3);	// PTC9 Motor 2 In 2 (speed) PTC8 TPM0_CH5

	// set GPIO outputs to outputs
	GPIOB_PDDR |= (1 << 18);		// PTB18 LED red
	GPIOB_PDDR |= (1 << 19);		// PTB19 LED green
	GPIOD_PDDR |= (1 << 1);			// PTD1  LED blue
	GPIOC_PDDR |= (1 << 1);			// PTC9 Signal indicator  output
	GPIOB_PDDR |= (1 << 8);			// PTB8 Camera SI
	GPIOB_PDDR |= (1 << 9);			// PTB9 Camera Clock
	GPIOA_PDDR |= (1 << 4);			// PTA4 Motor 1 In 1
	GPIOA_PDDR |= (1 << 5);			// PTA5 Motor 1 In 2 (direction)
	GPIOC_PDDR |= (1 << 8);			// PTC9 Motor 2 In 1 (direction)
	GPIOC_PDDR |= (1 << 9);			// PTC9 Motor 2 In 2 

	// all LEDs off
	GPIOB_PDOR |= GPIO_PDOR_PDO(1<<18);  		// red LED off
	GPIOB_PDOR |= GPIO_PDOR_PDO(1<<19);   		// green LED off
	GPIOD_PDOR |= GPIO_PDOR_PDO(1<<1);    		// blue LED off	
	GPIOC_PDOR &= ~GPIO_PDOR_PDO(1<<1);    		// indicator off

	// GPIOB_PDOR &= ~GPIO_PDOR_PDO(1<<18);  	// red LED on
	// GPIOB_PDOR &= ~GPIO_PDOR_PDO(1<<19);  	// green LED on
	// GPIOD_PDOR &= ~GPIO_PDOR_PDO(1<<1);   	// blue LED on	
	// GPIOC_PDOR |= GPIO_PDOR_PDO(1<<1);    	// indicator on

	// set GPIO input to input
	GPIOC_PDDR &= ~(1 << 0);		// PTC0 Push Button S2

	// set PWM outputs
	PORTA_PCR12 |= PORT_PCR_MUX(3);			// Servo Motor 1 Out  PTA12 TPM1_CH0

	// configure TPM clock source to be 48 MHz
	SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1);		// MCGFLLCLK clock or MCGPLLCLK/2
	SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;// use MCGPLLCLK/2 clock when running from 8 MHz and PLL

	// set TPM prescaler before enabling the timer
	TPM0_SC |= 3;					// prescaler for TPM0 (Motor) is 8
	TPM1_SC |= 3;					// prescaler for TPM1 (Servo) is 8

	// TPM modulo register, set frequency
	TPM0_MOD = 600;			// modulo TPM0 (Motor), periode = 0.10 ms (10000 Hz)
	TPM1_MOD = 60000;			// modulo TPM0 (Servo), periode = 10 ms (100 Hz)

	// set TPM clock mode to enable timer
	TPM0_SC |= TPM_SC_CMOD(1);		// enable TPM0 (Motor)
	TPM1_SC |= TPM_SC_CMOD(1);		// enable TPM1 (Servo)

	// configure TPM channel outputs high-true pulses
	TPM0_C1SC = 0x28;				// TPM0 channel1 Motor 1 In 1 speed left
	TPM0_C5SC = 0x28;				// TPM0 channel5 Motor 2 In 2 speed right
	TPM1_C0SC = 0x28;				// TPM1 channel0 Servo 1

	// TPM channel value registers, sets duty cycle
	//TPM1_C0V = 9000;				// TPM1 channel0 Servo 1 ca. 1.5 ms (middle)

	// initial configuration of motors
	TPM0_C1V = 150;					// TPM0 channel1 left Motor 
	TPM0_C5V = 150;					// TPM0 channel5 right Motor 
	GPIOA_PDOR &= ~(1 << 5);		// Set PTA5 left Motor 1 In 2 forward
	GPIOC_PDOR &= ~(1 << 8);		// Set PTC8 right Motor 2 In 1 forward

	// configure interrupts in TPM0
	TPM1_SC |= TPM_SC_TOIE_MASK;// enable overflow interrupt in TPM1 (10 ms rate)

	// ADC0 clock configuration
	ADC0_CFG1 |= 0x01;				// clock is bus clock divided by 2 = 12 MHz

	// ADC0 resolution    
	ADC0_CFG1 |= 0x08;				// resolution 10 bit, max. value is 1023

	// ADC0 conversion mode
	ADC0_SC3 = 0x00;				// single conversion mode

	// enable interrupts 18 (TPM = FTM1)  in NVIC, no interrupt levels
	NVIC_ICPR |= (1 << 18);			// clear pending interrupt 18
	NVIC_ISER |= (1 << 18);			// enable interrupt 18

	// enable interrupts globally
	asm (" CPSIE i ");
	// enable interrupts on core level

	//status preparation
	state |= START_FLAG | SRODEK_FLAG;

	// Main loop
	for (;;)
	{						// endless loop
	}						// end of endless loop	
	return 0;
}

void FTM1_IRQHandler()									// TPM1 ISR
{
	TPM1_SC |= 0x80;									// clear TPM1 ISR flag
	//GPIOB_PDOR &= ~GPIO_PDOR_PDO(1<<18);				// red LED on
	GPIOC_PDOR |= GPIO_PDOR_PDO(1<<1);					// indicator on

	// Capture Line Scan Image
	//ImageCapture();// capture LineScan image <- code of basic function moved below

	unsigned char i;
	ADC0_CFG2 |= 0x10;								// select B side of the MUX
	TAOS_SI_HIGH;
	TAOS_DELAY;
	TAOS_CLK_HIGH;
	TAOS_DELAY;
	TAOS_SI_LOW;
	TAOS_DELAY;
	// inputs data from camera (first pixel)
	ADC0_SC1A = 11;									// set ADC0 channel 11
	while ((ADC0_SC1A & ADC_SC1_COCO_MASK) == 0);	// wait until ADC is ready
	ImageData[0] = ADC0_RA ;						// return value
	TAOS_CLK_LOW;

	for (i = 1; i < 128; i++)
	{
		TAOS_DELAY;
		TAOS_DELAY;
		TAOS_CLK_HIGH;
		TAOS_DELAY;
		TAOS_DELAY;
		// inputs data from camera (one pixel each time through loop)
		ADC0_SC1A = 11;									// set ADC0 channel 11
		while ((ADC0_SC1A & ADC_SC1_COCO_MASK) == 0);// wait until ADC is ready
		ImageData[i] = ADC0_RA ;						// return value
		TAOS_CLK_LOW;
	}

	TAOS_DELAY;
	TAOS_DELAY;
	TAOS_CLK_HIGH;
	TAOS_DELAY;
	TAOS_DELAY;
	TAOS_CLK_LOW;

	numberOfDer = 0;
	int lastI = 200;

	for (i = 0; i < 127; i++)	// calculate difference between the pixels
	{
		if (i >= 1)
		{
			ImageDataDifference[i] = abs(ImageData[i] - ImageData[i + 1]);
			if (THRESHOLD < ImageDataDifference[i])
			{
				if (abs(lastI - i) > LINESCAN_TRESHOLD_EDGE)
				{
					BlackLinePosOld[numberOfDer] = BlackLinePos[numberOfDer];
					BlackLinePos[numberOfDer++] = i;
					lastI = i;
				}
			}
		}
	}

	if (numberOfDer == 0)
	{
		GPIOB_PDOR |= GPIO_PDOR_PDO(1<<19);		// green LED off
		GPIOD_PDOR |= GPIO_PDOR_PDO(1<<1);		// blue LED off	
		GPIOB_PDOR |= GPIO_PDOR_PDO(1<<18);		// red LED off
		TPM1_C0V = SERVO_DEAFULT_REGISTER_VALUE;
		GPIOB_PDOR |= GPIO_PDOR_PDO(1<<19);
		// green LED on
		GPIOD_PDOR |= GPIO_PDOR_PDO(1<<1);
		// blue LED on	
		GPIOB_PDOR |= GPIO_PDOR_PDO(1<<18);
	}

	else if (numberOfDer >= 2)
	{

		state = START_FLAG | SRODEK_FLAG;
		TPM1_C0V = SERVO_DEAFULT_REGISTER_VALUE - (((BlackLinePos[1] + BlackLinePos[0])/2-64)*_KP) - (BlackLinePos[0]-BlackLinePosOld[0] + BlackLinePos[1]-BlackLinePosOld[1])*_KD;
		GPIOB_PDOR &= ~GPIO_PDOR_PDO(1<<19);	// green LED on
		GPIOD_PDOR &= ~GPIO_PDOR_PDO(1<<1);		// blue LED on	
		GPIOB_PDOR &= ~GPIO_PDOR_PDO(1<<18);	// red LED on

		TPM0_C1V = STRAIGHT_SPEED;	// TPM0 channel1 left Motor 
		TPM0_C5V = STRAIGHT_SPEED;	// TPM0 channel5 right Motor 

		// GPIOC_PDOR |= GPIO_PDOR_PDO(1<<1);    // indicator on
	}
	else if (numberOfDer < 2)
	{
		int servo_reg_val = SERVO_DEAFULT_REGISTER_VALUE;
		if (state & LEWO_FLAG)
		{

			TPM0_C1V = CURVE_SPEED - (BlackLinePos[0] - 1);	// TPM0 channel1 left Motor 
			TPM0_C5V = CURVE_SPEED - (BlackLinePos[0] - 1);// TPM0 channel5 right Motor 

			if ((BlackLinePos[0]) < 64)
			{
				servo_reg_val = SERVO_DEAFULT_REGISTER_VALUE
				- ((BlackLinePos[0] - 1) * _KP - (BlackLinePos[0] - BlackLinePosOld[0])*_KD);
				if (servo_reg_val > SERVO_MINIMAL_REGISTER_VALUE)
				{
					TPM1_C0V = servo_reg_val;
				}
				else
				TPM1_C0V = SERVO_MINIMAL_REGISTER_VALUE;
			}
			else
			TPM1_C0V = SERVO_MINIMAL_REGISTER_VALUE;

			// all LEDs off
			GPIOB_PDOR |= GPIO_PDOR_PDO(1<<19);// green LED off
			GPIOD_PDOR |= GPIO_PDOR_PDO(1<<1);// blue LED off	
			GPIOB_PDOR &= ~GPIO_PDOR_PDO(1<<18);// red LED on

		}
		else if (state & PRAWO_FLAG)
		{

			TPM0_C1V = CURVE_SPEED - (BlackLinePos[0] - BlackLinePosOld[0])*_KD_SPEED;// TPM0 channel1 left Motor 
			TPM0_C5V = CURVE_SPEED - (BlackLinePos[0] - BlackLinePosOld[0])*_KD_SPEED;// TPM0 channel5 right Motor

			if ((BlackLinePos[0]) >= 64)
			{
				servo_reg_val = SERVO_DEAFULT_REGISTER_VALUE
				+ ((64-(BlackLinePos[0] - 64)) * _KP + (BlackLinePos[0] - BlackLinePosOld[0])*_KD);
				if (servo_reg_val < SERVO_MAXIMAL_REGISTER_VALUE)
				{
					TPM1_C0V = servo_reg_val;
				}
				else
				TPM1_C0V = SERVO_MAXIMAL_REGISTER_VALUE;
			}
			else
			TPM1_C0V = SERVO_MAXIMAL_REGISTER_VALUE; // all LEDs off
			GPIOB_PDOR |= GPIO_PDOR_PDO(1<<18);// red LED off
			GPIOD_PDOR |= GPIO_PDOR_PDO(1<<1);// blue LED off	

			GPIOB_PDOR &= ~GPIO_PDOR_PDO(1<<19);// green LED on
		}
		else
		{
			if (BlackLinePos[0] < LINESCAN_MIDDLE)
			{

				TPM0_C1V = CURVE_SPEED - (BlackLinePos[0] - BlackLinePosOld[0])*_KD_SPEED;// TPM0 channel1 left Motor 
				TPM0_C5V = CURVE_SPEED - (BlackLinePos[0] - BlackLinePosOld[0])*_KD_SPEED;// TPM0 channel5 right Motor 

				state &= SRODEK_FLAG;
				state |= LEWO_FLAG;

				servoWrite((BlackLinePos[0] - 1) * 4);

			}
			else if (BlackLinePos[0] >= LINESCAN_MIDDLE)
			{

				TPM0_C1V = CURVE_SPEED - (BlackLinePos[0] - BlackLinePosOld[0])*_KD_SPEED;// TPM0 channel1 left Motor 
				TPM0_C5V = CURVE_SPEED - (BlackLinePos[0] - BlackLinePosOld[0])*_KD_SPEED;// TPM0 channel5 right Motor 

				state &= SRODEK_FLAG;
				state |= PRAWO_FLAG;
				servoWrite((BlackLinePos[0] - 64) * -4);
			}
			// all LEDs off
			GPIOB_PDOR |= GPIO_PDOR_PDO(1<<18);// red LED off
			GPIOB_PDOR |= GPIO_PDOR_PDO(1<<19);// green LED off

			GPIOD_PDOR &= ~GPIO_PDOR_PDO(1<<1);// blue LED on	
		}

	}

	GPIOC_PDOR &= ~GPIO_PDOR_PDO(1<<1);	// indicator off

}

// Capture LineScan Image
void ImageCapture(void)
{

	unsigned char i;
	ADC0_CFG2 |= 0x10;							// select B side of the MUX
	TAOS_SI_HIGH;
	TAOS_DELAY;
	TAOS_CLK_HIGH;
	TAOS_DELAY;
	TAOS_SI_LOW;
	TAOS_DELAY;
	// inputs data from camera (first pixel)
	ADC0_SC1A = 11;									// set ADC0 channel 11
	while ((ADC0_SC1A & ADC_SC1_COCO_MASK) == 0);	// wait until ADC is ready
	ImageData[0] = ADC0_RA ;						// store value
	TAOS_CLK_LOW;

	for (i = 1; i < 128; i++)
	{
		TAOS_DELAY;
		TAOS_DELAY;
		TAOS_CLK_HIGH;
		TAOS_DELAY;
		TAOS_DELAY;
		// inputs data from camera (one pixel each time through loop)
		ADC0_SC1A = 11;									// set ADC0 channel 11
		while ((ADC0_SC1A & ADC_SC1_COCO_MASK) == 0);// wait until ADC is ready
		ImageData[i] = ADC0_RA ;						// store value
		TAOS_CLK_LOW;
	}

	TAOS_DELAY;
	TAOS_DELAY;
	TAOS_CLK_HIGH;
	TAOS_DELAY;
	TAOS_DELAY;
	TAOS_CLK_LOW;
}

void calculateServoLeft(void)
{
	if (BlackLinePos[0] > LINESCAN_MIDDLE)
	{
		servoWrite(SERVO_MAXIMAL_VALUE);
	}
	else
	{
		servoWrite((float) ((float) BlackLinePos[0] / 64.0) * 255);
	}
}

void calculateServoRight(void)
{
	if (BlackLinePos[0] < LINESCAN_MIDDLE)
	{
		servoWrite(SERVO_MINIMAL_VALUE);
	}
	else
	{
		servoWrite(-(float) ((float) BlackLinePos[0] / 64.0) * 255);
	}
}

void servoWrite(int _servoValue) //-255 to 255
{
	// if servo outside boundary
	if (_servoValue > SERVO_MAXIMAL_VALUE)
	{
		TPM1_C0V = SERVO_MAXIMAL_REGISTER_VALUE;
	}
	else if (_servoValue < SERVO_MINIMAL_VALUE )
	{
		TPM1_C0V = SERVO_MINIMAL_REGISTER_VALUE;
	}
	else //if servo value is good
	{
		TPM1_C0V =(_servoValue - SERVO_MINIMAL_VALUE) * (SERVO_MAXIMAL_REGISTER_VALUE - SERVO_MINIMAL_REGISTER_VALUE) / (SERVO_MAXIMAL_VALUE - SERVO_MINIMAL_VALUE) + SERVO_MINIMAL_REGISTER_VALUE;
	}
}
