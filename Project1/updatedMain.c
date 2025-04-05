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

// Include statements
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
#include "driverlib/systick.h" // SysTick driver library



const int CAR_MAX = 5;

typedef enum {STATE_NS, STATE_EW, OVERRIDE}
trafficLightStates;
trafficLightStates lightState = STATE_NS;

// default NS light green
void light_NS();
void light_EW();
void light_OVERRIDE();



// ACTIVE LOW!

int count = 0; // global variable to count number of cars passing the sensor

// IR Pins
// IR Logic
// IR Prototypes



// Switch Pins
// Switch Prototypes
// Switch Logic

void portF_input_setup(uint8_t pins); // input setup function prototype
void portB_output_setup(uint8_t pins); // output setup function prototype for Port B
void pedestrian_override(void); // pedestrian_override prototype
void car_count(void);
void portD_input_setup(uint8_t pins);


#define NS_GREEN  GPIO_PIN_1   // PB1 for NS Green LED
#define NS_RED    GPIO_PIN_2   // PB2 for NS Red LED
#define EW_GREEN  GPIO_PIN_3   // PB3 for EW Green LED
#define EW_RED    GPIO_PIN_4   // PB4 for EW Red LED


/*
 * main.c
 */

 int main(void)
 {
     portF_input_setup(GPIO_PIN_4);  // Set PF4 (pedestrian override) as input
     portB_output_setup(NS_GREEN | NS_RED | EW_GREEN | EW_RED); // Set PB1, PB2, PB3, PB4 as output for LEDs
     portD_input_setup(GPIO_PIN_0 | GPIO_PIN_1);  // Set PD0, PD1 (IR sensors) as input
 
     IntMasterEnable();  // Enable interrupts
 
     GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);  // Clear flag
     GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE); // Set interrupt type to falling edge for PF4
     GPIOIntRegister(GPIO_PORTF_BASE, pedestrian_override);  // Register interrupt handler
     GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);  // Enable the interrupt
 
     while (1) {
         car_count();  // Call car_count to update carCount with sensor data
 
         if (count >= CAR_MAX) {  // If car count reaches 5
             // Switch traffic lights between NS and EW directions
             if (lightState == STATE_NS) {
                 lightState = STATE_EW;
                 light_EW();  // Switch to EW direction
             } else if (lightState == STATE_EW) {
                 lightState = STATE_NS;
                 light_NS();  // Switch to NS direction
             }
             count = 0;  // Reset car count
         }
 
         switch (lightState) {
             case STATE_NS:
                 light_NS();  // Set NS light to green and EW to red
                 break;
             case STATE_EW:
                 light_EW();  // Set EW light to green and NS to red
                 break;
             case OVERRIDE:
                 light_OVERRIDE();  // Set both sides to red during override
                 SysCtlDelay(SysCtlClockGet() * 20 / 3); // create a delay of 20 seconds
                 lightState = STATE_NS;  // Reset to NS state after override
                 break;
             default:
                 break;
         }
     }
     return 0;
 }

// Function definitions

// Function to configure Port F input (pedestrian override switch)
void portF_input_setup(uint8_t pins) {
    SYSCTL_RCGCGPIO_R |= 0x20; // configure the clock for Port F
    GPIO_PORTF_LOCK_R |= 0x4C4F434B; // unlock pins
    GPIO_PORTF_CR_R |= pins; // allow interaction with pins
    GPIO_PORTF_DIR_R &= ~pins; // set direction for pins to be input
    GPIO_PORTF_PUR_R |= pins; // set up pull-up resistors for active low pins
    GPIO_PORTF_DEN_R |= pins; // configure the enable
}

void portD_input_setup(uint8_t pins) {
    SYSCTL_RCGCGPIO_R |= 0x08;  // Enable clock for Port D 
    GPIO_PORTD_LOCK_R |= 0x4C4F434B;  // Unlock Port D pins
    GPIO_PORTD_CR_R |= pins;  // Allow modification of these pins
    GPIO_PORTD_DIR_R &= ~pins;  // Set direction to input
    GPIO_PORTD_DEN_R |= pins;  // Enable the pins (Digital function)
}

// Function to configure Port B for output (LEDs)
void portB_output_setup(uint8_t pins) {
    SYSCTL_RCGCGPIO_R |= 0x02;  // configure the clock for Port B 
    GPIO_PORTB_DIR_R |= pins;   // set direction to output
    GPIO_PORTB_DEN_R |= pins;   // enable the pins
}

// Function to control NS and EW lights using LEDs
void light_NS() {
    GPIO_PORTB_DATA_R = NS_GREEN | EW_RED;  
}

void light_EW() {
    GPIO_PORTB_DATA_R = NS_RED | EW_GREEN; 
}

void light_OVERRIDE() {
    GPIO_PORTB_DATA_R = NS_RED | EW_RED;  // Both NS and EW red PB2 and PB4
}

void car_count(void) {
    if (!GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0)) {
        count++;
        while (!GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0)); 
    }
    if (!GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1)) {
        count++;
        while (!GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1)); 
    }
}


void pedestrian_override(void) {
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4); // clear the flag so the interrupt can happen again
    if (!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)) { // check if switch SW1 is pressed
        lightState = OVERRIDE;
    }
}
