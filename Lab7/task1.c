//******************ECE 132*************************
//Names: Daniel Gress, Shane Duffy
//Lab section: ECE 132, Section 010
//*************************************************
//Date Started: Tuesday, March 25, 2025
//Date of Last Modification: March 27, 2025
//Lab Assignment: Lab 7
//Lab Due Date: Tuesday, April 1, 2025
//*************************************************
//Purpose of program: Task 1, create a simple multimeter.
//Program Inputs: Analog voltage from ADC channel 0 (PE3)
//Program Outputs: Voltage displayed over UART terminal
//*************************************************

//File include Statements
#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"

// Variable Declarations
#define GPIO_PA0_U0RX           0x00000001
#define GPIO_PA1_U0TX           0x00000401

// Function Prototypes
bool UARTKeyPressed(void); // Helper function to detect a key press

// Main Program
// NOTE: any blank fills are notated in UPPERCASE for legibility

int main(void) {
    uint32_t INPUT;
    float voltage;
    unsigned long voltage_mV;
    int i;

    char message[] = "Voltage: ";
    int mess_len = sizeof(message)/sizeof(message[0]);

    char units[] = " V"; 				// char array for voltage units
    int un_len = sizeof(units)/sizeof(units[0]);

    char next[] = "Press any key to sample again\r\n";
    int next_len = sizeof(next)/sizeof(next[0]);

    // The following block of code is used to configure THE SYSTEM CLOCK AND UART0
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set system clock to 50MHz
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Enable UART0 module
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {} // Wait for UART0 to be ready
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable GPIO Port A
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {} // Wait for GPIOA to be ready
    GPIOPinConfigure(GPIO_PA0_U0RX); // Configure PA0 as UART RX line
    GPIOPinConfigure(GPIO_PA1_U0TX); // Configure PA1 as UART TX line
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Set PA0 and PA1 as UART pins
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); // Set UART to 8N1 at 115200bps

    // Send startup message to UART terminal for visual confirmation
    char init_msg[] = "Ready! Press any key to start sampling.\r\n";
    for (i = 0; i < sizeof(init_msg) - 1; i++) {
        UARTCharPut(UART0_BASE, init_msg[i]);
    }

    // Peripheral setup for ADC0
    // This will read the voltage off Port E Pin 3 (AIN0)
    // This ADC will have the ability to read a voltage between 0V and 3V3
    // The resolution of the measurement is ~0.8059 mV (3.3V / 4096 steps)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // Enable ADC0 module
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {} // Wait for ADC0 to be ready
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Enable GPIO Port E
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {} // Wait for GPIOE to be ready
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); // Enable PE3 as ADC input (AIN0)
    ADCSequenceDisable(ADC0_BASE, 0); // Disable ADC0 before configuration
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0); // The ADC0 is configured to use PROCESSOR TRIGGER and PRIORITY 0
    
    // The ADC will support up to 8 samples to be taken (WE ARE USING 1)
    // The ADC will be triggered to get a sample based on the condition: PROCESSOR TRIGGER
    // The ADC will have a priority that is HIGHER THAN OTHER SAMPLINGS (PRIORITY 0)

    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    // For SEQUENCER 0, the step 0 configuration is being setup
    // The step configuration is for CHANNEL 0 (AIN0) to be read and this is the sample that is LAST IN THE SEQUENCE READ
    // It is configured that the ADC INTERRUPT IS ALLOWED TO HAPPEN

    ADCSequenceEnable(ADC0_BASE, 0); //Enable ADC0 with sample sequence number 0

    while (1) {
        // waits for user key press to trigger ADC read
        while (!UARTKeyPressed()) {}

        // read and discard character to clear UART buffer
        UARTCharGet(UART0_BASE);

        // trigger ADC conversion
        ADCProcessorTrigger(ADC0_BASE, 0);
        while (!ADCIntStatus(ADC0_BASE, 0, false)) {} //wait for conversion to complete
        ADCIntClear(ADC0_BASE, 0); //clear ADC interrupt flag
        ADCSequenceDataGet(ADC0_BASE, 0, &INPUT); //get ADC result

        // convert ADC value to voltage (0–3.3V)
        voltage = (INPUT * 3.3f) / 4095.0f;
        voltage_mV = (unsigned long)(voltage * 1000); //convert to millivolts

	/* to print out the decimal, we must convert the number read from the pin to some kind of string
	* in this case, we are subdividing the floating point number to its whole component (thousands) and 
 	* its decimal component (remainder). then, we will output each one using UART. 
  	* Modulo is used to separate each 10ths place.     	
  	*/
	
        int thousands = voltage_mV / 1000;
        int remainder = voltage_mV % 1000;

        UARTCharPut(UART0_BASE, '\n');
        UARTCharPut(UART0_BASE, '\r');

        //output voltage prefix message
        for (i = 0; i < mess_len - 1; i++) {
            UARTCharPut(UART0_BASE, message[i]);
        }

        //output integer component
        UARTCharPut(UART0_BASE, '0' + thousands);
        UARTCharPut(UART0_BASE, '.');

        //output decimal component
        UARTCharPut(UART0_BASE, '0' + ((remainder / 100) % 10));
        UARTCharPut(UART0_BASE, '0' + ((remainder / 10) % 10));
        UARTCharPut(UART0_BASE, '0' + (remainder % 10));
	    
        //output units
        for (i = 0; i < un_len - 1; i++) {
            UARTCharPut(UART0_BASE, units[i]);
        }
        // end of message
	UARTCharPut(UART0_BASE, '\r');
        UARTCharPut(UART0_BASE, '\n');
        
	// output repeat message
        for (i = 0; i < next_len - 1; i++) {
            UARTCharPut(UART0_BASE, next[i]);
        }
    }
}

// checks if a character is available from UART
bool UARTKeyPressed(void) {
    return UARTCharsAvail(UART0_BASE);
}
