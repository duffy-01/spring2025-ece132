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
#include "driverlib/adc.h"
#include "driverlib/watchdog.h"


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


void WatchdogInit(void);
void WatchdogIntHandler(void);


volatile bool g_bWatchdogFeed = true;
volatile bool input_started = false;
volatile uint32_t last_input_time = 0;
volatile uint32_t current_time = 0;

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

    Servo_Init(5000);

    WatchdogInit();

    WatchdogIntClear(WATCHDOG0_BASE);
    while (1)
    {
        int key = input(); // check if key pressed
        if (key != -1) // if key pressed
        {

            if(input_index == 0){
                input_started = true;
                last_input_time = current_time;

                char startMsg[] = "Timer started!";
                int i;
                for (i = 0; i < sizeof(startMsg) - 1; i++) {
                    UARTCharPut(UART0_BASE, startMsg[i]);
                }
                UARTCharPut(UART0_BASE, '\n');
                UARTCharPut(UART0_BASE, '\r');
            }

            user_input[input_index] = key;
            input_index++;
            char messageE[]= "Entered digit: ";
            int i = 0;
            int mess_lenE = sizeof(messageE)/sizeof(messageE[0]);
            for(i = 0; i < mess_lenE; i++){
                UARTCharPut(UART0_BASE, messageE[i]);
            }
            UARTCharPut(UART0_BASE, '0' + input_index);
            UARTCharPut(UART0_BASE, '\n');
            UARTCharPut(UART0_BASE, '\r');
        }

            if (input_index > 3) // Check when 4 digits are entered
            {
                input_started = false;

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
                     user_input[0] = 0;
                     user_input[1] = 0;
                     user_input[2] = 0;
                     user_input[3] = 0;
                     correct = false;
                     input_index = 0;
                }
                else
                {
                    input_index = 0; // reset if wrong
                    char messageW[]= "Incorrect password.";
                    int i;
                    int mess_lenW = sizeof(messageW)/sizeof(messageW[0]);
                    for(i = 0; i < mess_lenW; i++){
                        UARTCharPut(UART0_BASE, messageW[i]);
                    }
                    UARTCharPut(UART0_BASE, '\n');
                    UARTCharPut(UART0_BASE, '\r');
                }
            }


        switch(lockState)
        {
            case LOCK:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); // red LED
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 5000);
                break;
            case UNLOCK:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); // green LED
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 1000);
                SysCtlDelay(SysCtlClockGet() * 1 / 3 * 5);
                lockState = LOCK;
                break;
            case INTRUDER:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); // red LED
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 5000);
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
        while(!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0));
        lockState = INTRUDER;
        char messageI[]="INTRUDER ALERT";
        int i = 0;
        int mess_len = sizeof(messageI)/sizeof(messageI[0]);
        for(i = 0; i < mess_len; i++){
            UARTCharPut(UART0_BASE, messageI[i]);
        }
        UARTCharPut(UART0_BASE, '\n');
        UARTCharPut(UART0_BASE, '\r');
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
                while(GPIO_PORTC_DATA_R);
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
void WatchdogIntHandler(void)
{
    // Clear the watchdog interrupt
    WatchdogIntClear(WATCHDOG0_BASE);

    // Increment time counter
    current_time++;

    // Check if input has been started but not completed
    if (input_started && input_index > 0 && input_index < 4)
    {
        if (current_time - last_input_time >= 10) // 10 seconds timeout
        {
            // Reset input
            input_index = 0;
            input_started = false;

            // Display timeout message
            char message[] = "Timeout! Please enter 4 digits within 10 seconds.";
            int i = 0;
            for (i = 0; i < sizeof(message) - 1; i++)
            {
                UARTCharPut(UART0_BASE, message[i]);
            }
            UARTCharPut(UART0_BASE, '\n');
            UARTCharPut(UART0_BASE, '\r');
        }
    }
}

// In WatchdogInit(), make sure the watchdog doesn't reset the system, we just want the interrupt
void WatchdogInit(void)
{
    // Enable watchdog peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WDOG0)) {}

    // Register and enable watchdog interrupt
    IntRegister(INT_WATCHDOG, WatchdogIntHandler);
    IntEnable(INT_WATCHDOG);

    // Unlock if locked
    if(WatchdogLockState(WATCHDOG0_BASE) == true)
    {
        WatchdogUnlock(WATCHDOG0_BASE);
    }

    // Configure watchdog
    WatchdogIntEnable(WATCHDOG0_BASE);
    WatchdogIntTypeSet(WATCHDOG0_BASE, WATCHDOG_INT_TYPE_INT);

    // Set reload value for 1 second intervals
    // SysCtlClockGet() gives ticks per second, divide by 1 for 1-second timeout
    WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet()/1);

    // IMPORTANT: We don't want the system to reset, just the interrupt
    WatchdogResetDisable(WATCHDOG0_BASE);

    // Enable watchdog
    WatchdogEnable(WATCHDOG0_BASE);
}
