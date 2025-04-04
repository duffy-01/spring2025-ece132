//*************************************************************************
// Name of Group Members: Takeru Hiura, Shane Duffy
// Creation Date: 4/3/25
// Lab Section: Section 010
// Lab this program is associated with: Project 1
// Lab due date: 4/15/25
//
// Hardware Inputs used:
// Hardware Outputs used:
//
// Additional files needed:
//
// Date of last modification: 4/3/25
//*************************************************************************

// include statements 
#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"

//****************************
//  STATE STUFF
//****************************

const int CAR_MAX = 5;

typedef enum {STATE_NS, STATE_EW, OVERRIDE}
trafficLightStates;

// default NS light green
void light_NS();
void light_EW();
void light_OVERRIDE();

//****************************
//  IR SENSOR PARAMETERS
//****************************

const uint8_t pinNS; 
const uint8_t pinEW;

// ACTIVE LOW!
void count_car(){}
void count_car(){

}


int carCount = 0; //global variable to count number of cars pass the sensor


// IR Pins
// IR Logic
// IR Prototypes

//****************************
//  SWITCH PARAMETERS
//****************************

// Switch Pins
// Switch Prototypes
// Switch Logic

void portF_input_setup(uint8_t pins); // input setup function prototype
void pedestrian_override(void); //pedestrian_override prototype
void car_count(void);

/*
 * main.c
 */

int main(void)
{
    portF_input_setup(0x10);            // call portF_input setup function to initialize PF4 (switch SW1) as input
    portD_input_setup(pinNS | pinEW);   // setup the pins for NS and EW on port D
   
    systick(0x00FFFFFF);                //set reload value for SysTick
    SysTickIntRegister(pedestrian_override);  //register SysTick interrupt handler
    IntMasterEnable();                  //enable interrupts
   
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);                      // clear the flag so the interrupt can happen again
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE); // set interrupt type to falling edge for PF4 (switch SW1, active low)
    GPIOIntRegister(GPIO_PORTF_BASE, pedestrian_override);          // register interrupt handler to pedestrian_override function
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);                     // enable the interrupt
    
    while (1)
    {
        if (carCount >= CAR_MAX)
        {
            lightState = fsm[lightState.next[input]];
            carCount = 0;
        };

        switch(lightState)
        {
            case STATE_NS:
            // ir counter here
            lightState = STATE_EW
            case STATE_EW:

            case OVERRIDE:
            SysCtlDelay(SysCtlClockGet() / 3 * 10);
            lightState = light_NS;
        }
    }
    SysTickIntRegister();              //clear SysTick interrupt register
	return 0;
}

// Function definitions
    // Start the function: portF_input_setup
    // It is type void since it doesn't return anything
    // It will have the following parameters: uint8_t pins, an 8 bit integer for the GPIO Port F pins we want to enable as input
    // Step 1 of configuring a GPIO pin as an input is to configure the clock
    // Step 2 of configuring a GPIO pin as an input is to unlock the switch
    // Step 3 of configuration for a GPIO pin is to configure the direction to be an input
    // Step 4 of the configuration for a GPIO pin to be input is to set up pull-up resistors for active low pins
    // Step 5 of the configuration for a GPIO pin to be an output is to configure the enable for the pin
    // end the function
void portF_input_setup(uint8_t pins){
    SYSCTL_RCGCGPIO_R |= 0x20; // configure the clock for Port F
    GPIO_PORTF_LOCK_R |= 0x4C4F434B; // unlock pins
    GPIO_PORTF_CR_R |= pins; // allow interaction with pins
    GPIO_PORTF_DIR_R &= ~pins; // set direction for pins to be input
    GPIO_PORTF_PUR_R |= pins; // set up pull-up resistors for active low pins
    GPIO_PORTF_DEN_R |= pins; // configure the enable
}



void portD_input_setup(uint8_t pins){
    SYSCTL_RCGCGPIO_R |= 0x08; // configure the clock for Port D
    GPIO_PORTD_CR_R |= pins; // allow interaction with pins
    GPIO_PORTD_DIR_R &= ~pins; // set direction for pins to be input
    GPIO_PORTD_PUR_R |= pins; // set up pull-up resistors for active low pins
    GPIO_PORTD_DEN_R |= pins; // configure the enable
}



void systick(int reload_val){
    NVIC_ST_CTRL_R = 0;                 //disable SysTick
    NVIC_ST_RELOAD_R = reload_val;      //set reload value
    NVIC_ST_CURRENT_R = 0;              //clear current value
    NVIC_ST_CTRL_R = 0x07;              //enable SysTick with interrupts
}

void pedestrian_override(void){

    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4); // clear the flag so the interrupt can happen again

    if(!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)){ // check if switch SW1 is pressed
        lightState = OVERRIDE;
        SysCtlDelay(SysCtlClockGet() * 20 / 3); //create a delay of 20 seconds by multiplying the number of clocks per second by 10 and dividing by 3 because of the 3 cycle delay produced by SysCtlDelay
    }
}

void car_count(void){
    if(GPIOPinREAD(GPIO_PORTD_BASE, GPIO_PIN_0) == 1){
        carCount++;
        while(GPIOPinREAD(GPIO_PORTD_BASE, GPIO_PIN_0) == 1);
    }
}