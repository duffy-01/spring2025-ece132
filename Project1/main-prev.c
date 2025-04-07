//*************************************************************************
// Name of Group Members: Takeru Hiura, Shane Duffy
// Creation Date: 4/3/25
// Lab Section: Section 010
// Lab this program is associated with: Project 1
// Lab due date: 4/15/25
//
// Hardware Inputs used: F0, F1 (IR sensors)
// Hardware Outputs used: F2, F3 (NS lights), D6, D7 (EW lights)
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

typedef enum {STATE_NS, STATE_EW, OVERRIDE} trafficLightStates;

trafficLightStates lightState = STATE_NS; // Initialize state to NS green

// default NS light green
void light_NS();
void light_EW();
void light_OVERRIDE();

//****************************
//  IR SENSOR PARAMETERS
//****************************

const uint8_t pinNS = GPIO_PIN_6; // D6
const uint8_t pinEW = GPIO_PIN_7; // D7

// ACTIVE LOW!
void count_car(){}

int carCount = 0; //global variable to count number of cars pass the sensor

//****************************
//  SWITCH PARAMETERS
//****************************

void portF_input_setup(uint8_t pins); // input setup function prototype
void portD_input_setup(uint8_t pins); // input setup function prototype
void pedestrian_override(void); //pedestrian_override prototype
void car_count(void);

/*
 * main.c
 */

int main(void)
{
    // Set up clock
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    portF_input_setup(GPIO_PIN_4 | GPIO_PIN_0 | GPIO_PIN_1);            // call portF_input setup function to initialize PF4, PF0 (IR sensors), and PF1 as input
    portD_input_setup(GPIO_PIN_6 | GPIO_PIN_7);   // setup the pins for NS and EW on port D
    
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
            lightState = (lightState == STATE_NS) ? STATE_EW : STATE_NS;
            carCount = 0;
        };

        switch(lightState)
        {
            case STATE_NS:
                light_NS();
                break;
            case STATE_EW:
                light_EW();
                break;
            case OVERRIDE:
                light_OVERRIDE();
                break;
        }
    }
    SysTickIntRegister(0);              //clear SysTick interrupt register
    return 0;
}

// Function definitions
void portF_input_setup(uint8_t pins){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // configure the clock for Port F
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // unlock pins
    GPIO_PORTF_CR_R |= pins; // allow interaction with pins
    GPIO_PORTF_DIR_R &= ~pins; // set direction for pins to be input
    GPIO_PORTF_PUR_R |= pins; // set up pull-up resistors for active low pins
    GPIO_PORTF_DEN_R |= pins; // configure the enable
}

void portD_input_setup(uint8_t pins){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; // configure the clock for Port D
    GPIO_PORTD_CR_R |= pins; // allow interaction with pins
    GPIO_PORTD_DIR_R &= ~pins; // set direction for pins to be input
    GPIO_PORTD_PUR_R |= pins; // set up pull-up resistors for active low pins
    GPIO_PORTD_DEN_R |= pins; // configure the enable
}

void systick(int reload_val){
    NVIC_ST_CTRL_R = 0;                 //disable SysTick
    NVIC_ST_RELOAD_R = reload_val;      //set reload value
    NVIC_ST_CURRENT_R = 0;              //clear current value
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_CLK_SRC; //enable SysTick with interrupts
}

void pedestrian_override(void){
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4); // clear the flag so the interrupt can happen again

    if(!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)){ // check if switch SW1 is pressed
        lightState = OVERRIDE;
        SysCtlDelay(SysCtlClockGet() * 20 / 3); //create a delay of 20 seconds by multiplying the number of clocks per second by 10 and dividing by 3
    }
}

void car_count(void){
    if(GPIOPinRead(GPIO_PORTD_BASE, pinNS) == 1 || GPIOPinRead(GPIO_PORTD_BASE, pinEW) == 1){
        carCount++;
        while(GPIOPinRead(GPIO_PORTD_BASE, pinNS) == 1 || GPIOPinRead(GPIO_PORTD_BASE, pinEW) == 1);
    }
}

void light_NS(){
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_2); // NS green, EW red
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, 0);
}

void light_EW(){
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0); // NS red, EW green
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_6);
}

void light_OVERRIDE(){
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_3); // NS red, EW red
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_7);
    SysCtlDelay(SysCtlClockGet() / 3 * 10); // delay for 10 seconds
    lightState = STATE_NS;
}
