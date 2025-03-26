//******************ECE 132*************************
//Names: Daniel Gress, Shane Duffy
//Lab section: ECE 132, Section 010
//*************************************************
//Date Started: Tuesday, March 25, 2025
//Date of Last Modification: March 25, 2025
//Lab Assignment: Lab 7
//Lab Due Date: Tuesday, April 1, 2025
//*************************************************
//Purpose of program: Task 1, create a simple multimeter.
//Program Inputs:
//Program Outputs: 
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
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"


//Variable Declarations
#define GPIO_PA0_U0RX           0x00000001

#define GPIO_PA1_U0TX           0x00000401

//Function Prototypes

//Main Program
int main(void) {
    unsigned long INPUT;
    char message[]="Voltage: ";
    int i;
    int mess_len = sizeof(message)/sizeof(message[0]);
    char units[]= " V"; //Character array for the units of the voltage
    int un_len = sizeof(units)/sizeof(units[0]);
    char next[]="Press any Key to sample again";
    int next_len = sizeof(next)/sizeof(next[0]);
    long C;

    //The following block of code is used to configure ____________________
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set up Clock
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Enable UART hardware
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable Pin hardware
    GPIOPinConfigure(GPIO_PA0_U0RX); // Configure GPIO pin for UART RX line
    GPIOPinConfigure(GPIO_PA1_U0TX); // Configure GPIO Pin for UART TX line
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Set Pins for UART
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, // Configure UART to 8N1 at 115200bps
    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));



    //Peripheral setup for ADC0
    //This will read the voltage off Port ___ Pin ___
	  //This ADC will have the ability to read a voltage between 0mV and ___ mV
	  //The resolution of the measurement is __ mV
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR,0); //The ADC0 is configured
    //The ADC will support up to ___ samples to be taken
    //The ADC will be triggered to get a sample based on the condition _____________________
    //The ADC will have a priority that is __________ than other samplings

    ADCSequenceStepConfigure(_________,0,0,ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
    //For ____ the step configuration is being setup
    //The step configuration is for Channel ___ to be read and this to be the sample that is ____ in the sequence read
    //It is configured that the ADC interrupt is (allowed or not allowed) to happen

    ADCSequenceEnable(ADC0_BASE, 0); //Enable ADC___ with sample sequence number ___



while(1){ //All your code will go in this section
    //read in ADC:
    ADCProcessorTrigger(ADC0_BASE,0);        // trigger ADC convsersion
    ADCSequenceDataGet(ADC0_BASE,0,&INPUT);  // get result


    //display voltage on the UART:
  
        //step 1 - convert the value to a set of characters
        float voltage = (INPUT) / 4096;
        mess_len = snprint(message, "%.2f", voltage);
  
        //Step 2 - output to UART
        //reset position from last message
        UARTCharPut(UART0_BASE,'\n');
        UARTCharPut(UART0_BASE,'\r');
  
        //Voltage message
        for (i=0; i<mess_len; i++){
            UARTCharPut(UART0_BASE, message[i]);
        }

        // Voltage values
  
        // Voltage units
        UARTCharPut(UART0_BASE, 'V');

        // Interface to trigger new sample read
          // New line
          UARTCharPut(UART0_BASE, '\n');
  
          // Output message about how to get a new sample
          char sample_message[] = "Press button to get new sample";
          for (i = 0; i < sizeof(sample_message) - 1; i++)
          {
              UARTCharPut(UART0_BASE, sample_message[i]);
          }
  
          // Wait for button to be pushed to take next reading
          while (!button_is_pressed())
          {
              // Wait loop (implement button_is_pressed() to return true when the button is pressed)
          }

}
