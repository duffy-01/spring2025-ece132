//*************************************************************************
// Name of Group Members: Daniel Gress, Shane Duffy
// Creation Date: 25 February 2025
// Lab Section: Section 010
// Lab this program is associated with: Lab 4
// Lab due date: 3 February 2025
//
// Hardware Inputs used:    Port F pin 4.
// Hardware Outputs used:   SW1 on Port F pin 4.
//
// Additional files needed: stdint.h, stdbool.h, driverlib/systick.h, driverlib/sysctl.h, inc/tm4c123gh6pm.h
//
// Date of last modification: 25 February 2025
//*************************************************************************

//Include statements

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "inc/hw_types.h"

//Global variable declarations

//Prototype statements

void portF_output_setup(int output_pin);
void portF_input_setup(int input_pin);
void systick(int reload_val);
void blink_control(void);
void blink(void);

//main function

int main(){

    portF_output_setup(0x40);          //set pin 1 on port f as output
    portF_input_setup(0x10);           //set pin 4 on port f as input

    systick(0x00FFFFFF);               //set reload value for SysTick
    SysTickIntRegister(blink_control); //register SysTick interrupt handler
    IntMasterEnable();                 //enable interrupts

    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);     //set pin 4 to falling edge
    GPIOIntRegister(GPIO_PORTF_BASE, blink_control);                    //register interrupt handler
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);                         //enable interrupt on pin 4
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);                          //clear interrupt on pin 4

    while(1){                          //execute {} ad aeternum
        blink_control();               //call blink control function
    }
    SysTickIntRegister();              //clear SysTick interrupt register
    return 0;
}

//Function Definitions

void portF_output_setup(int output_pin){
    SYSCTL_RCGCGPIO_R |= 0x20;          //enable clock on port f
    GPIO_PORTF_DIR_R |= output_pin;     //set pin as output
    GPIO_PORTF_DEN_R |= output_pin;     //enable pin
}
void portF_input_setup(int input_pin){
    SYSCTL_RCGCGPIO_R |= 0x20;          //enable clock on port f
    GPIO_PORTF_DIR_R &= ~input_pin;     //set pin as input
    GPIO_PORTF_DEN_R |= input_pin;      //enable pin
}
void systick(int reload_val){
    NVIC_ST_CTRL_R = 0;                 //disable SysTick
    NVIC_ST_RELOAD_R = reload_val;      //set reload value
    NVIC_ST_CURRENT_R = 0;              //clear current value
    NVIC_ST_CTRL_R = 0x07;              //enable SysTick with interrupts
}
void blink_control(void){
    if((GPIO_PORTF_DATA_R & 0x10) == 0x10){ //if input pin is pressed
        blink();                            //call blink function
    }
}
void blink(void){
    GPIO_PORTF_DATA_R ^= 0x40;          //toggle output pin
    for(int i = 0; i < 1000000; i++);   //delay
    GPIO_PORTF_DATA_R ^= 0x40;          //toggle output pin
}

