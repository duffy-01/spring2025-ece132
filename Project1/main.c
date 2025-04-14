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
// Date of last modification: 4/8/25
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
#include "driverlib/systick.h"

//****************************
//  STATE STUFF
//****************************

const int CAR_MAX = 5;

typedef enum {STATE_NS, STATE_EW, OVERRIDE} trafficLightStates;

trafficLightStates lightState = STATE_EW; // Initialize state to NS green

// default NS light green
void light_NS();
void light_EW();
void light_OVERRIDE();

//****************************
//  IR SENSOR PARAMETERS
//****************************

const uint8_t pinNS = GPIO_PIN_6; // D6
const uint8_t pinEW = GPIO_PIN_7; // D7
void count_car(){}
int carCount = 0; //global variable to count number of cars pass the sensor

//****************************
//  SWITCH PARAMETERS
//****************************

void portF_input_setup(uint8_t pins); // input setup function prototype
void portD_input_setup(uint8_t pins); // input setup function prototype
void portF_output_setup(uint8_t pins); // input setup function prototype
void portD_output_setup(uint8_t pins); // input setup function prototype

void pedestrian_override(void); //pedestrian_override prototype
void car_count(uint8_t sensorPin);
void systick(int reload_val);

/*
 * main.c
 */

int main(void)
{
    // Set up clock
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    portF_input_setup(0x13);    //b10011 corresponds to SW1, N, and E respectively
    portF_output_setup(0x0C);       //b01100
    portD_output_setup(0x44);                 // setup the pins for NS and EW on port D

    systick(0xFFFFFF); //set reload value for SysTick

    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);                      // clear the flag so the interrupt can happen again
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE); // set interrupt type to falling edge for PF4 (switch SW1, active low)
    GPIOIntRegister(GPIO_PORTF_BASE, pedestrian_override);          // register interrupt handler to pedestrian_override function
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);                     // enable the interrupt

    SysTickIntRegister(pedestrian_override);
    IntMasterEnable();                                              //enable interrupts

    while (1)
    {
        switch(lightState)
        {
            case STATE_NS:
                light_NS();
                car_count(GPIO_PIN_0);
                lightState = STATE_EW;
                break;
            case STATE_EW:
                light_EW();
                car_count(GPIO_PIN_1);
                lightState = STATE_NS;
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

void portF_output_setup(uint8_t pins){
    SYSCTL_RCGCGPIO_R |= 0x20;          //enable clock on port f
    GPIO_PORTF_DIR_R |= pins;     //set pin as output
    GPIO_PORTF_DEN_R |= pins;     //enable pin
}

void portD_input_setup(uint8_t pins){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; // configure the clock for Port D
    GPIO_PORTD_CR_R |= pins; // allow interaction with pins
    GPIO_PORTD_DIR_R &= ~pins; // set direction for pins to be input
    GPIO_PORTD_PUR_R |= pins; // set up pull-up resistors for active low pins
    GPIO_PORTD_DEN_R |= pins; // configure the enable
}

void portD_output_setup(uint8_t pins){
    SYSCTL_RCGCGPIO_R |= 0x08;          //enable clock on port f
    GPIO_PORTD_DIR_R |= pins;     //set pin as output
    GPIO_PORTD_DEN_R |= pins;     //enable pin
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

    }
}

void car_count(uint8_t sensorPin){
    int count = 0;
    uint8_t prevState= 1; // sensor idle

    while (count <= CAR_MAX) {
        uint8_t currentState = (GPIO_PORTF_DATA_R & sensorPin) ? 1 : 0;
        if (prevState == 0 && currentState == 1){count++;}
        prevState = currentState;

        SysCtlDelay(SysCtlClockGet()/3000);  //delay to control polling rate

        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4); // clear the flag so the interrupt can happen again
            if(!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)){ // check if switch SW1 is pressed
                light_OVERRIDE();
                break;
            }
    }
}

void light_NS(){
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0); // NS green, EW red
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); // NS green, EW red

    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6,  GPIO_PIN_6);
}

void light_EW(){
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0); // NS green, EW red
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); // NS green, EW red

    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_2, 0); // NS green, EW red
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // NS green, EW red
}

void light_OVERRIDE(){
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_3); // NS red, EW red
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_2, GPIO_PIN_6);
    SysCtlDelay(SysCtlClockGet() * 1 / 3 * 20); // delay for 20 seconds
    lightState = STATE_NS;
}
