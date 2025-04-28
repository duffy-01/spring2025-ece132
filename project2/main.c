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





typedef enum {LOCK, UNLOCK, INTRUDER} lockStates;
lockStates lockState = LOCK; // Initialize state LOCK


//function prototypes 
void Servo_Init(void);
void intruder_alert(void);
void portF_input_setup(uint8_t pins);
void portF_output_setup(uint8_t pins);
void systick(int reload_val);
void UART_Init(void);
void keypad_Init(void);
int input(void);
void Watchdog_Init(void);
void WatchdogIntHandler(void);



// Password Variables
int password[4] = {1, 2, 3, 4};  // Correct sequence
int user_input[4];               // Store user input
int input_index = 0;             // Track input position




volatile bool g_bWatchdogFeed = 1;

/*
 * main.c
 */

int main(void)
{
    unsigned long INPUT;

    portF_input_setup(GPIO_PIN_0);
    portF_output_setup(GPIO_PIN_2 | GPIO_PIN_3);
    systick(0xFFFFFF);

    keypad_Init();
    UART_Init();
    Servo_Init();
    Watchdog_Init();

    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);
    GPIOIntRegister(GPIO_PORTF_BASE, intruder_alert);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
    SysTickIntRegister(intruder_alert);
    IntMasterEnable();

    WatchdogIntClear(WATCHDOG0_BASE);



    //Peripheral setup for ADC0
    //This will read the voltage off Port E Pin 0
    //This ADC will have the ability to read a voltage between 0mV and 3300 mV
    //The resolution of the measurement is 0.8 mV
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR,0); //The ADC0 is configured
    //The ADC will support up to 8 samples to be taken
    //The ADC will be triggered to get a sample based on the condition of processor trigger
    //The ADC will have a priority that is higher than other samplings
    ADCSequenceStepConfigure(ADC0_BASE,0,0,ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH9); //channel 3 for PE0
    //For sequence 0 the step configuration is being setup
    //The step configuration is for Channel 3 to be read and this to be the sample that is first in the sequence read
    //It is configured that the ADC interrupt is (allowed or not allowed) to happen
    ADCSequenceEnable(ADC0_BASE, 0); //Enable ADC0 with sample sequence number 0

    while (1)
    {
        ADCProcessorTrigger(ADC0_BASE,0);//causes a processor trigger for a sample sequence
        ADCSequenceDataGet(ADC0_BASE,0,&INPUT);//gets ADC value and stores it in unsigned long variable INPUT
        int key = input(); // check if key pressed
        if (key != -1) // if key pressed
        {
            g_bWatchdogFeed = 1;

            user_input[input_index] = key;
            input_index++;
            char messageE[]= "Entered digit ";
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
                g_bWatchdogFeed = 1;
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

                if (correct && (INPUT > 1241))
                {
                    lockState = UNLOCK;
                     user_input[0] = 0;
                     user_input[1] = 0;
                     user_input[2] = 0;
                     user_input[3] = 0;
                     correct = false;
                     input_index = 0;
                }
                else
                {
                    input_index = 0;
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
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // configure the clock for Port F
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // unlock pins
    GPIO_PORTF_CR_R |= pins; // allow interaction with pins
    GPIO_PORTF_DIR_R &= ~pins; // set direction for pins to be input
    GPIO_PORTF_PUR_R |= pins; // set up pull-up resistors for active low pins
    GPIO_PORTF_DEN_R |= pins; // configure the enable
}


// Start the function: portF_output_setup
// It is type void since it doesn't return anything
// It will have the following parameters: uint8_t pins, an 8 bit integer for the GPIO Port F pins we want to enable as input
// Step 1 of configuring a GPIO pin as an output is to configure the clock
// Step 2 of configuration for a GPIO pin is to configure the direction to be an output
// Step 3 of configuration for a GPIO pin is to enable the pin
// end the function
void portF_output_setup(uint8_t pins){
    SYSCTL_RCGCGPIO_R |= 0x20;    //enable clock on port f
    GPIO_PORTF_DIR_R |= pins;     //set pin as output
    GPIO_PORTF_DEN_R |= pins;     //enable pin
}

void UART_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}


void Servo_Init(void) {
    SysCtlPWMClockSet(SYSCTL_PWMDIV_8);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 40000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 5000);
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
}


// Start the function: systick
// It is type void since it doesn't return anything
// It will have the following parameters: int reload_val
// Step 1 of setting the reload value of the timer is the initialization of the control register to 0 so we are not trying to count while we set things
// Step 2 of setting the reload value of the timer is to set the reload value
// Step 3 of setting the reload value of the timer is to reset the current value that is used for counting
// Step 4 of setting the reload value of the timer is to initialize the control register to allow counting to happen
// end the function
void systick(int reload_val){
    NVIC_ST_CTRL_R = 0;                 //disable SysTick
    NVIC_ST_RELOAD_R = reload_val;      //set reload value
    NVIC_ST_CURRENT_R = 0;              //clear current value
    NVIC_ST_CTRL_R = 0x07;              //enable SysTick with interrupts
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
void keypad_Init(){
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



int input(){
    int row, col;
    for(row = 0; row < 4; row++){
        GPIO_PORTE_DATA_R = (1 << row);
        for(col = 0; col < 4; col++){
            if((GPIO_PORTC_DATA_R & (1 << (col + 4))) != 0){
                while(GPIO_PORTC_DATA_R);
                int key_map[4][4] = {
                    {1,2,3,'A'},
                    {4,5,6,'B'},
                    {7,8,9,'C'},
                    {'#',0,'*','D'}
                };
                return key_map[row][col];
            }
        }
    }
    return -1;
}


void WatchdogIntHandler(void)
{
    WatchdogIntClear(WATCHDOG0_BASE);

    if (!g_bWatchdogFeed && input_index > 0 && input_index < 4)
    {
        input_index = 0;

        char message[] = "Input timeout. Please try again.";
        int i = 0;
        for (i = 0; i < sizeof(message) - 1; i++)
        {
            UARTCharPut(UART0_BASE, message[i]);
        }
        UARTCharPut(UART0_BASE, '\n');
        UARTCharPut(UART0_BASE, '\r');
        g_bWatchdogFeed = 1;
    }
    else{

        g_bWatchdogFeed = 0;
    }


}

void Watchdog_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WDOG0)) {}

    WatchdogIntRegister(INT_WATCHDOG, WatchdogIntHandler);
    IntEnable(INT_WATCHDOG);

    if(WatchdogLockState(WATCHDOG0_BASE) == true)
    {
        WatchdogUnlock(WATCHDOG0_BASE);
    }
    WatchdogIntEnable(WATCHDOG0_BASE);
    WatchdogIntTypeSet(WATCHDOG0_BASE, WATCHDOG_INT_TYPE_INT);
    WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet() * 10/3);
    WatchdogEnable(WATCHDOG0_BASE);
}
