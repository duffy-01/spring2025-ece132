//*************************************************************************
//Name of Group Members: Daniel Gress, Shane Duffy
//Creation Date: 17 March 2025
//Lab Section: Section 010
//Lab this program is associated with: Lab 6
//Lab due date: 25 March 2025
//
//Hardware Inputs used: Port F pin 4 (Left Button - SW1), Port F pin 0 (Right Button - SW2)
//Hardware Outputs used: UART0 (PA0 U0RX, PA1 U0TX)
//
//Additional files needed: stdint.h, stdbool.h, inc/hw_memmap.h, inc/hw_types.h,
//                          driverlib/sysctl.h, driverlib/gpio.h, driverlib/uart.h,
//                          driverlib/pin_map.h, driverlib/interrupt.h, inc/hw_ints.h
//
//Date of last modification: 17 March 2025
//*************************************************************************

//include statements for necessary libraries
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"

volatile uint32_t g_ui32LeftCount = 0;                                       //this variable keeps track of how many times the left button was pressed
volatile bool namesPrinted = false;                                          //this flag prevents the group names from being printed repeatedly

void GPIOPortF_Handler(void);                                                //forward declaration of the port f interrupt handler

//inituart: sets up uart0 on port a at 115200 baud, 8 data bits, 1 stop bit, no parity
void InitUART(void)
{
SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);                                 //enable uart0 peripheral
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);                                 //enable gpio port a for uart
while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));                          //wait until uart0 is ready
while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));                          //wait until gpio a is ready
GPIOPinConfigure(GPIO_PA0_U0RX);                                             //configure pa0 as uart rx
GPIOPinConfigure(GPIO_PA1_U0TX);                                             //configure pa1 as uart tx
GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);                   //set pa0 and pa1 as uart pins
UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); // configure uart settings
}

//initgpio: configures port f for button inputs on pf0 and pf4 and sets up interrupts
void InitGPIO(void)
{
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                                  //enable gpio port f
while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));                           //wait until gpio f is ready
HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;                         //unlock port f to allow modifications to pf0
HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;                                   //allow changes to pf0
GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);               //set pf0 and pf4 as inputs
GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); //enable weak pull-ups on pf0 and pf4
GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_FALLING_EDGE);  //set interrupts for falling edge
GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);                      //enable interrupts on pf0 and pf4
GPIOIntRegister(GPIO_PORTF_BASE, GPIOPortF_Handler);                          //register the interrupt handler for port f
IntEnable(INT_GPIOF);                                                         //enable the port f interrupt in the nvic
}

//
//gpioportf_handler: interrupt service routine for port f
//if pf4 (left button) is pressed, it increments the counter
//if pf0 (right button) is pressed, it prints the count over uart and resets the counter
void GPIOPortF_Handler(void)
{
uint32_t ui32Status;                                                   //get the interrupt status
int i;                                                                 //loop variable for printing characters
ui32Status = GPIOIntStatus(GPIO_PORTF_BASE, true);                     //read interrupt status for port f
GPIOIntClear(GPIO_PORTF_BASE, ui32Status);                             //clear the interrupt flags
if(ui32Status & GPIO_PIN_4)                                            //if left button (pf4) is pressed
{
    g_ui32LeftCount++;                                                 //increment the left press count
}
if(ui32Status & GPIO_PIN_0)                                            //if right button (pf0) is pressed
{
    char buffer[50];                                                   //buffer for output message
    sprintf(buffer, "Left presses: %d\r\n", g_ui32LeftCount);          //format the message with the count
    for(i = 0; buffer[i] != '\0'; i++)                                 //loop through the message characters
    {
        UARTCharPut(UART0_BASE, buffer[i]);                            //send each character over uart
    }
    g_ui32LeftCount = 0;                                               //reset the left press count
}
}

//
//polluartfornames: checks if uart input is available and prints the group names once per key press
void PollUARTForNames(void)
{
int i;                                                                 //loop variable for printing characters
if(UARTCharsAvail(UART0_BASE))                                         //if there is uart input available
{
    while(UARTCharsAvail(UART0_BASE))                                  //flush all incoming characters
    {
        (void)UARTCharGet(UART0_BASE);
    }
    if(!namesPrinted)                                                  //if the names have not been printed already
    {
        char names[] = "Group Members: Daniel Gress, Shane Duffy\r\n"; //group names message
        for(i = 0; names[i] != '\0'; i++)                              //loop through the message characters
        {
            UARTCharPut(UART0_BASE, names[i]);                         //send each character over uart
        }
        namesPrinted = true;                                           //set the flag to prevent repeated printing
    }
}
else
{
    namesPrinted = false;                                              //reset the flag when no uart input is detected
}
}


//main: sets up the system clock, initializes uart and gpio, enables interrupts, and continuously polls uart
int main(void)
{
SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //set system clock to 50mhz using pll and a 16mhz crystal
InitUART();                                                                             //initialize uart
InitGPIO();                                                                             //initialize gpio for buttons
IntMasterEnable();                                                                      //enable global interrupts
while(1)                                                                                //main loop
{
    PollUARTForNames();                                                                 //poll for uart input to print group names
    SysCtlDelay(SysCtlClockGet()/3000);                                                 //delay to control polling rate
}
return 0;
}
