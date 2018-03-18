#include "derivative.h"
#include "mcg.h"

void InitClock (void) {
	
	int i;										// counter for loops
	int temp_reg;								// temporary register
	
	// configure clock to 48 MHz from a 8 MHz crystal
		MCG_C2 = (MCG_C2_RANGE0(1) | MCG_C2_EREFS0_MASK); 	// configure the oscillator settings
		MCG_C1 = (MCG_C1_CLKS(2) | MCG_C1_FRDIV(3));		// divider for 8 MHz clock	
		for (i = 0; i < 24000; i++)	{
			// wait for OSCINIT to set
		}
		
	// now in FBE mode
		MCG_C6 |= MCG_C6_CME0_MASK;		// enable the clock monitor
		MCG_C5 |= MCG_C5_PRDIV0(1); 	// set PLL ref divider to divide by 2
		temp_reg = MCG_C6; 				// store present C6 value (as CME0 bit was previously set)
		temp_reg &= ~MCG_C6_VDIV0_MASK; // clear VDIV settings
		temp_reg |= MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0); 	// write new VDIV and enable PLL
		MCG_C6 = temp_reg; 				// update MCG_C6		
		for (i = 0; i < 4000; i++) 	// wait for PLLST status bit to set
	
	// now in PBE mode
		SIM_CLKDIV1 = (SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV4(1));	// core clock, bus clock div by 2	
		MCG_C1 &= ~MCG_C1_CLKS_MASK;	// switch CLKS mux to select the PLL as MCGCLKOUT	
		for (i = 0; i < 2000; i++) {
			// Wait for clock status bits to update
		}
	// now in PEE mode, core and system clock 48 MHz, bus and flash clock 24 MHz

}
