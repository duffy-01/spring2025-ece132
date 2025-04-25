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
#include "driverlib/pwm.h"

//****************************
//  STATE STUFF
//****************************

uint32_t ulPeriod = 40000; // 2MHz / 50Hz = 40000 ticks for 20ms period

typedef enum {LOCK, UNLOCK, INTRUDER} lockStates;
lockStates lockState = LOCK; // Initialize state LOCK

void Servo_Init(uint32_t dutyCycle);
void intruder_alert();
void portF_input_setup(uint8_t pins);
void portF_output_setup(uint8_t pins);
void systick(int reload_val);

// Keypad Functions
void keypad_init();
int input();

// Password Variables
int password[4] = {1, 2, 3, 4};  // Correct sequence
int user_input[4];               // Store user input
int input_index = 0;             // Track input position

/*
 * main.c
 */

int main(void)
{
    portF_input_setup(GPIO_PIN_0);
    portF_output_setup(GPIO_PIN_2 | GPIO_PIN_3);
    systick(0xFFFFFF);
    keypad_init();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);
    GPIOIntRegister(GPIO_PORTF_BASE, intruder_alert);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
    SysTickIntRegister(intruder_alert);
    IntMasterEnable();

    while (1)
    {
        int key = input(); // check if key pressed
        if (key != -1) // if key pressed
        {
            user_input[input_index] = key;
            input_index++;

            if (input_index == 4) // Check when 4 digits are entered
            {
                bool correct = true;
                int i = 0;
                for (i = 0; i < 4; i++)
                {
                    if (user_input[i] != password[i])
                    {
                        correct = false;
                        break;
                    }
                }

                if (correct)
                {
                    lockState = UNLOCK; // correct password
                }
                else
                {
                    input_index = 0; // reset if wrong
                }
            }
        }

        switch(lockState)
        {
            case LOCK:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); // red LED
                Servo_Init(5000);
                break;
            case UNLOCK:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); // green LED
                Servo_Init(1000);
                break;
            case INTRUDER:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); // red LED
                Servo_Init(5000);
                break;
        }
    }

}

// Function definitions
void portF_input_setup(uint8_t pins){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R |= pins;
    GPIO_PORTF_DIR_R &= ~pins;
    GPIO_PORTF_PUR_R |= pins;
    GPIO_PORTF_DEN_R |= pins;
}

void portF_output_setup(uint8_t pins){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    GPIO_PORTF_DIR_R |= pins;
    GPIO_PORTF_DEN_R |= pins;
}

void Servo_Init(uint32_t dutyCycle) {
    SysCtlPWMClockSet(SYSCTL_PWMDIV_8);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ulPeriod);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, dutyCycle);
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
}

void systick(int reload_val){
    NVIC_ST_CTRL_R = 0;
    NVIC_ST_RELOAD_R = reload_val;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 0x07;
}

void intruder_alert(void){
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
    if(!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0)){
        lockState = INTRUDER;
        char messageI[]="INTRUDER ALERT";
        int i;
        int mess_len = sizeof(messageI)/sizeof(messageI[0]);
        for(i = 0; i < mess_len; i++){
            UARTCharPut(UART0_BASE, messageI[i]);
        }
    }
}

//*****************
// Keypad setup
//*****************
void keypad_init(){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R4;
    while((SYSCTL_RCGCGPIO_R & (SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R4))==0);

    GPIO_PORTB_DEN_R |= 0xFF;
    GPIO_PORTB_DIR_R |= 0xFF;

    GPIO_PORTC_DEN_R |= 0xF0;
    GPIO_PORTC_DIR_R &= ~0xF0;
    GPIO_PORTC_PDR_R |= 0xF0;

    GPIO_PORTE_DEN_R |= 0x0F;
    GPIO_PORTE_DIR_R |= 0x0F;
}

// Keypad scan function
int input(){
    int row, col;
    for(row = 0; row < 4; row++){
        GPIO_PORTE_DATA_R = (1 << row);
        for(col = 0; col < 4; col++){
            if((GPIO_PORTC_DATA_R & (1 << (col + 4))) != 0){
                // Now return the key number
                int key_map[4][4] = {
                    {1,2,3,10},
                    {4,5,6,11},
                    {7,8,9,12},
                    {14,0,15,13}
                };
                return key_map[row][col];
            }
        }
    }
    return -1; // no key pressed
}
