//*************************************************************************
// Name of Group Members: Daniel Gress, Shane Duffy
// Creation Date:Thursday 4 February 2025
// Lab Section: Section 010
// Lab this program is associated with: Lab 1
// Lab due date: Friday 5 February 2025
//
// Hardware Inputs used: 
// Hardware Outputs used: None. 
//
// Additional files needed: inc folder, driverlib folder, stdin.h
//
// Date of last modification: 6 February 2025 @ 22:36
//*************************************************************************

#include <stdint.h>
#include "inc/tm4c123gh6pm"

/**
 * 
 * main.c
 *
 */

int main(void)
{
    
	SYSCTL_RCG_RCGCGPIO_R |= 0x20; //enable clock on port f

	GPIO_PORTF_DEN_R |= 0x40; //enable pin 1 on port f
    GPIO_PORTF_DIR_R |= 0x40; //denote pin 1 on port f as an output
	
    
    while(1) //execute {} ad aeternum
    {
    GPIO_PORTF_DATA_R |= 0x10; //set value of port f pin1 to high
    }

		
    
    
	return 0;
}





