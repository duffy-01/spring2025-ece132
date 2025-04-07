//*************************************************************************
// Name of Group Members: Takeru Hiura, Shane Duffy
// Creation Date: 4/3/25
// Lab Section: Section 010
// Lab this program is associated with: Project 1
// Lab due date: 4/15/25
//
// Hardware Inputs used:    F0 & F1 for N & E Ir Sensors 
//                          F4 for SW1  
//                          
// Hardware Outputs used:   F2 & F3 for NS Red & Green
//                          D6 & D7 for WE Red & Green
//
// Additional files needed: see includes for exact header info 
//                          (driverlib and inc folders needed)
//
// Date of last modification: 4/6/25
//*************************************************************************

// include statements 
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
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

//*****************************************
//              Constants
//*****************************************

#define pinNS GPIO_PIN_0
#define pinEW GPIO_PIN_1
#define CAR_THRESHOLD 5

#define NS_GREEN GPIO_PIN_2   // PF2
#define NS_RED   GPIO_PIN_3   // PF3
#define EW_GREEN GPIO_PIN_6   // PD6
#define EW_RED   GPIO_PIN_7   // PD7

volatile int carCount = 0;

enum Input {
    SENSOR_NONE = 0,
    SENSOR_NS   = 1,
    SENSOR_EW   = 2,
    SENSOR_BOTH = 3
};

//*****************************************
//          State Machine Stuff
//*****************************************

struct state {
    unsigned long portF_out;  // North-South Lights (PF2, PF3)
    unsigned long portD_out;  // East-West Lights (PD6, PD7)
    const struct state *next[4];
};

typedef const struct state styp;

#define goN   &fsm[0]
#define waitN &fsm[1]
#define goE   &fsm[2]
#define waitE &fsm[3]
#define override &fsm[4]

styp fsm[5] = {
    {NS_GREEN,  EW_RED,   {goN, waitN, goN, waitN}},   // goN
    {NS_RED,    EW_RED,   {goE, goE, goE, goE}},       // waitN
    {NS_RED,    EW_GREEN, {goN, goE, waitE, waitE}},   // goE
    {NS_RED,    EW_RED,   {goN, goN, goN, goN}},       // waitE
    {NS_RED,    EW_RED,   {goN, goN, goN, goN}}        // override (all red)
};

styp *lightState = goN;

//*****************************************
//          Function Prototypes
//*****************************************

void portF_input_setup(uint8_t pins);
void portD_input_setup(uint8_t pins);
void systick(int reload_val);
void pedestrian_override(void);
void update_lights(unsigned long pf_out, unsigned long pd_out);

//*****************************************
//          Main Function
//*****************************************

int main(void) {
    uint8_t input;
    carCount = 0;
    lightState = goN;

    portF_input_setup(GPIO_PIN_4 | NS_GREEN | NS_RED);
    portD_input_setup(pinNS | pinEW | EW_GREEN | EW_RED);
    systick(0x00FFFFFF);
    SysTickIntRegister(pedestrian_override);
    IntMasterEnable();

    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    GPIOIntRegister(GPIO_PORTF_BASE, pedestrian_override);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);

    while (1) {
        input = 0;
        if ((GPIOPinRead(GPIO_PORTD_BASE, pinNS) & pinNS) == 0) {
            input |= 1;
        }
        if ((GPIOPinRead(GPIO_PORTD_BASE, pinEW) & pinEW) == 0) {
            input |= 2;
        }

        // Count cars for the current direction
        if ((input & 1) && (lightState == goN || lightState == waitN)) {
            carCount++;
            while ((GPIOPinRead(GPIO_PORTD_BASE, pinNS) & pinNS) == 0);
        } else if ((input & 2) && (lightState == goE || lightState == waitE)) {
            carCount++;
            while ((GPIOPinRead(GPIO_PORTD_BASE, pinEW) & pinEW) == 0);
        }

        // Transition if enough cars have passed
        if (carCount >= CAR_THRESHOLD) {
            lightState = lightState->next[input];
            update_lights(lightState->portF_out, lightState->portD_out);
            carCount = 0;
        }
    }

    return 0;
}

//*****************************************
//         Functions Definitions
//*****************************************

void portF_input_setup(uint8_t pins) {
    SYSCTL_RCGCGPIO_R |= 0x20;
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R |= pins;
    GPIO_PORTF_DIR_R |= (pins & ~GPIO_PIN_4); // outputs except PF4
    GPIO_PORTF_DIR_R &= ~GPIO_PIN_4; // PF4 is input
    GPIO_PORTF_PUR_R |= GPIO_PIN_4;
    GPIO_PORTF_DEN_R |= pins;
}

void portD_input_setup(uint8_t pins) {
    SYSCTL_RCGCGPIO_R |= 0x08;
    GPIO_PORTD_CR_R |= pins;
    GPIO_PORTD_DIR_R |= (pins & ~(pinNS | pinEW)); // outputs except IR inputs
    GPIO_PORTD_DIR_R &= ~(pinNS | pinEW);
    GPIO_PORTD_PUR_R |= (pinNS | pinEW);
    GPIO_PORTD_DEN_R |= pins;
}

void systick(int reload_val) {
    NVIC_ST_CTRL_R = 0;
    NVIC_ST_RELOAD_R = reload_val;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 0x07;
}

void pedestrian_override(void) {
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);
    if ((GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) & GPIO_PIN_4) == 0) {
        lightState = override;
        update_lights(lightState->portF_out, lightState->portD_out);
        SysCtlDelay(SysCtlClockGet() * 5 / 3);
        lightState = goN;
        carCount = 0;
    }
}

void update_lights(unsigned long pf_out, unsigned long pd_out) {
    GPIO_PORTF_DATA_R &= ~(NS_GREEN | NS_RED);
    GPIO_PORTF_DATA_R |= pf_out;

    GPIO_PORTD_DATA_R &= ~(EW_GREEN | EW_RED);
    GPIO_PORTD_DATA_R |= pd_out;
}


