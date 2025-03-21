//******************ECE 132*************************
//Name:
//Lab section:
//*************************************************
//Date Started:
//Date of Last Modification:
//Lab Assignment:
//Lab Due Date:
//*************************************************
//Purpose of program in Task 1 Part 1:
//Purpose of program in Task 1 Part 2:
//Program Hardware Inputs:
//Program Hardware Outputs:
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

//Variable Declarations
#define GPIO_PA0_U0RX           0x00000001
#define GPIO_PA1_U0TX           0x00000401

//Function Prototypes
void initUART(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Enable UART hardware
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable Pin hardware
    GPIOPinConfigure(GPIO_PA0_U0RX); // Configure GPIO pin for UART RX line
    GPIOPinConfigure(GPIO_PA1_U0TX ); // Configure GPIO Pin for UART TX line
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));                          //wait until uart0 is ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));                          //wait until gpio a is ready
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Set Pins for UART
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, // Configure UART to 8N1 at 115200bps
    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

//Main Program
int main(void) {
    int i;
    char C;
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    initUART();

    char names[] = "Group Members: Daniel Gress, Shane Duffy\r\n";
    int names_len = sizeof(names)/sizeof(names[0]);

    while(1) {
        // Prompt user for input
        char prompt[] = "Input a letter: ";
        int prompt_len = sizeof(prompt)/sizeof(prompt[0]);

        for(i = 0; i < names_len; i++) {
            if(names[i] != '\0'){
                UARTCharPut(UART0_BASE, names[i]);
            }
        }

        // Wait for and capture user input
        C = UARTCharGet(UART0_BASE);

        // Echo back the received character
        UARTCharPut(UART0_BASE, C);

        // Formatting: Move to new line after displaying input
        UARTCharPut(UART0_BASE, '\r');
        UARTCharPut(UART0_BASE, '\n');
    }
    return 0;
}
