//*************************************************************************
// Name of Group Members: Takeru Hiura, Shane Duffy
// Creation Date: 4/18/25
// Lab Section: Section 010
// Lab this program is associated with: Project 2
// Lab due date: 5/9/25
//
// Hardware Inputs used: PF0 (IR sensor), PE4 (potentiometer), PC4, PC5, PC6, PC7, PE0, PE1, PE2, for keypad inputs
// Hardware Outputs used: PF2, PF3 (LEDs), PF1 (servo motor)
//
// Additional files needed: driverlib and inc files
//
// Date of last modification: 4/28/25
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
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/watchdog.h"




//structure used for the FSM of the system
typedef enum {LOCK, UNLOCK, INTRUDER} lockStates;
lockStates lockState = LOCK; // Initialize state to LOCK


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



int password[4] = {1, 2, 3, 4}; // password sequence for lock device
int user_input[4];              // array used to get user input
int input_index = 0;            // variable used to keep track of user input




volatile bool g_bWatchdogFeed = 1; //intialize feed variable to feed for watchdog


int main(void){
    unsigned long adc_input; //used to store adc value

    portF_input_setup(GPIO_PIN_0); //enable PF0 for user input
    portF_output_setup(GPIO_PIN_2 | GPIO_PIN_3); //enable PF2 and PF3 for led output
    systick(0xFFFFFF);

    //functions to initialize peripherals and software
    keypad_Init();
    UART_Init();
    Servo_Init();
    Watchdog_Init();

    //setup the interrupt to PF0 for the IR sensor
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);                          // clear the flag so the interrupt can happen again
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);     // set interrupt type to falling edge for PF0 (IR sensor, active low)
    GPIOIntRegister(GPIO_PORTF_BASE, intruder_alert);                   // register interrupt handler to intruder_alert function
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);                         // register interrupt handler to intruder_alert function
    SysTickIntRegister(intruder_alert);                                 // registers intruder_alert as SysTick interrupt handler
    IntMasterEnable();                                                  // enable interrupts

    WatchdogIntClear(WATCHDOG0_BASE);                                   //initially have watchdog cleared




    //Peripheral setup for ADC0
    //This will read the voltage off Port E Pin 4
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR,0); //The ADC0 is configured
    //The ADC will support up to 8 samples to be taken
    //The ADC will be triggered to get a sample based on the condition of processor trigger
    //The ADC will have a priority that is higher than other samplings
    ADCSequenceStepConfigure(ADC0_BASE,0,0,ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH9); //Channel 9 for PE4
    //For sequence 0 the step configuration is being setup
    //The step configuration is for Channel 9 to be read and this to be the sample that is first in the sequence read
    //It is configured that the ADC interrupt is (allowed or not allowed) to happen
    ADCSequenceEnable(ADC0_BASE, 0); //Enable ADC0 with sample sequence number 0

    while (1) {
        ADCProcessorTrigger(ADC0_BASE,0);//causes a processor trigger for a sample sequence
        ADCSequenceDataGet(ADC0_BASE,0,&adc_input);//gets ADC value and stores it in unsigned long variable INPUT
        int key = input(); //retrieve input value from keypad
        if (key != -1) { //if statement to check if there is user input
            g_bWatchdogFeed = 1; //feed dog when there is user input

            user_input[input_index] = key; //store user input from keypad
            input_index++; //increment the user input index
            char messageE[]= "Entered digit "; //char array for message to display if user input was recognized
            int i = 0; // initialize variable i
            int mess_lenE = sizeof(messageE)/sizeof(messageE[0]); //calculate length of message
            for(i = 0; i < mess_lenE; i++){ //using UART to display message on terminal
                UARTCharPut(UART0_BASE, messageE[i]);
            }
            UARTCharPut(UART0_BASE, '0' + input_index); //print which input has been entered
            UARTCharPut(UART0_BASE, '\n'); //newline on terminal
            UARTCharPut(UART0_BASE, '\r'); //set cursor to start of newline
        }

            if (input_index > 3) { //checks if user entered all 4 keypad inputs
                g_bWatchdogFeed = 1; // feed the dog
                bool correct = true; //originally set the password check to true
                int i = 0; //intialize loop variable
                for (i = 0; i < 4; i++) { // go through each user input and compare to password
                    if (user_input[i] != password[i]){
                        correct = false; // set password check to false if any digits in the input does not match the password
                        break;
                    }
                }

                if (correct && (input_index > 1241)) { // check if the potentiometer is in the correct position and if the password entered is correct
                    lockState = UNLOCK; //set fsm state to unlock
                    // set all user inputs back to 0 for the next attempt
                     user_input[0] = 0;
                     user_input[1] = 0;
                     user_input[2] = 0;
                     user_input[3] = 0;
                     correct = false;
                     input_index = 0;
                }
                else {
                    input_index = 0;
                    char messageW[]= "Incorrect password."; //message when password combination is incorrect
                    int i; //initialize loop variable
                    int mess_lenW = sizeof(messageW)/sizeof(messageW[0]); //get the length of the message
                    for(i = 0; i < mess_lenW; i++){ //print each character of the message
                        UARTCharPut(UART0_BASE, messageW[i]);
                    }
                    UARTCharPut(UART0_BASE, '\n'); //make a newline after the message
                    UARTCharPut(UART0_BASE, '\r'); //set cursor to beginning of the newline
                }
            }


        //switch statement for each of the states in the FSM
        switch(lockState){
            case LOCK: //lock state
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); //make sure green LED is off
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); // turn on red LED
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 5000); //set PWM so servo motor is in lock state, -90 degrees
                break;
            case UNLOCK: //unlock state
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0); //make sure red LED is off
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); // turn on green LED
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 1000); // set PWM so servo motor is in unlock state, 90 degrees
                SysCtlDelay(SysCtlClockGet() * 1 / 3 * 5); //set a delay so system stays unlocked for approximately 5 seconds
                lockState = LOCK; //set state back to lock
                break;
            case INTRUDER: //intruder state
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); //make sure green LED is off
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); //turn on red LED
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 5000); //set PWM so servo motor is in lock state, -90 degrees
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

// Start the function: UART_Init
// It is type void since it doesn't return anything
// Step 1 is to enable the UART hardware
// Step 2 is to enable the GPIO A pin for the UART
// Step 3 is to configure the UART RX line
// Step 4 is to configure the UART TX line
// Step 5 is to configure UART to 8N1 at 115200 bps
void UART_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Enable UART hardware
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable Pin hardware
    GPIOPinConfigure(GPIO_PA0_U0RX); // Configure GPIO pin for UART RX line
    GPIOPinConfigure(GPIO_PA1_U0TX); // Configure GPIO Pin for UART TX line
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, // Configure UART to 8N1 at 115200bps
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

// Start the function: Servo_Init
// It is type void since it doesn't return anything
// Step 1 is to divide the system clock by 8 and set it to the PWM clock
// Step 2 is to enable port F for the PWM signal
// Step 3 is to enable module 1 for the PWM
// Step 4 is to configure PF1 pin as PWM
// Step 5 is to set up PF1 as the PWM pin
// Step 6 is to set up the PWM options
// Step 7 is to set up the period of the PWM signal
// Step 8 is to set up the duty cycle to 12.5%
// Step 9 is to enable the PWM generator
// Step 10 is to turn on the output pin
void Servo_Init(void) {
    SysCtlPWMClockSet(SYSCTL_PWMDIV_8); //divide the system clock by 8 to set up a 20 ms period
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // enable port F as the PWM signal
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); // enable module 1 which is used for the PWM signal
    GPIOPinConfigure(GPIO_PF1_M1PWM5); // configures pin PF1 for the PWM
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1); // this is used to set up the actual PF1 pin to use for PWM
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC); // set up for the options of the PWM
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 40000); // set the period of the PWM signal to 40000
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 5000); // set up the default duty cycle to be 5000/40000 = 12.5%
    PWMGenEnable(PWM1_BASE, PWM_GEN_2); // this is to set up the PWM generator
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true); //this is to turn on the output pin for the PWM signal
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

// Start the function: intruder_alert
// It is type void since it doesn't return anything
// Step 1 of setting the reload value of the timer is the initialization of the control register to 0 so we are not trying to count while we set things
// Step 2 of setting the reload value of the timer is to set the reload value
// Step 3 of setting the reload value of the timer is to reset the current value that is used for counting
// Step 4 of setting the reload value of the timer is to initialize the control register to allow counting to happen
// end the function
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


// Start the function: keypad_Init
// It is type void since it doesn't return anything
// ENTER COMMENTS
// end the function

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


// Start the function: input
// It is type void since it doesn't return anything
// ENTER COMMENTS
// end the function

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



// Start the function: WatchdogIntHandler
// It is type void since it doesn't return anything
// Step 1 is to clear the watchdog interrupt so it can happen
// Step 2 is to check if the watchdog has not been fed and if user has not entered all inputs if they began entering them
// Step 3 is to display a timeout message to UART
// Step 4 is to feed the watchdog after so the watchdog is on standby again
// end the function
void WatchdogIntHandler(void)
{
    WatchdogIntClear(WATCHDOG0_BASE); //clear watchdog interrupt so it can happen again

    if (!g_bWatchdogFeed && input_index > 0 && input_index < 4){ //if statement to check if watchdog has not been fed and if user has not entered all inputs
        input_index = 0; //set the input index back to 0 for the lock input to reset
        char message[] = "Input timeout. Please try again."; //message to display to user in terminal
        int i = 0; // variable for loop
        for (i = 0; i < sizeof(message) - 1; i++) { //go through each character in string to print to terminal
            UARTCharPut(UART0_BASE, message[i]); //prints each character
        }
        UARTCharPut(UART0_BASE, '\n'); //goes to newline after message
        UARTCharPut(UART0_BASE, '\r'); //goes to beginning of new line
        g_bWatchdogFeed = 1; // feed the watchdog
    }
    else{
        g_bWatchdogFeed = 0; // has the watchdog to not fed for future interrupts
    }


}

// Start the function: Watchdog_Init
// It is type void since it doesn't return anything
// Step 1 is to enable the watchdog peripheral
// Step 2 is to set the interrupt to be the WatchdogHandler function
// Step 3 is to enable the watchdog interrupt
// Step 4 is to check the register access and make sure it is unlocked
// Step 5 is to enable the actual watchdog interrupt
// Step 6 is to set up the period for the watchdog timer to 10 seconds
// Step 7 is to turn on the watchdog
// end the function
void Watchdog_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0); // enable watchdog peripheral
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WDOG0)) {} //wait for perhipheral to be ready

    WatchdogIntRegister(INT_WATCHDOG, WatchdogIntHandler); // set the WatchdogIntHandler method for the watchdog interrupt
    IntEnable(INT_WATCHDOG); // enable watchdog interrupt

    if(WatchdogLockState(WATCHDOG0_BASE) == true) //this is to make sure register access is unlocked for watchdog
    {
        WatchdogUnlock(WATCHDOG0_BASE);
    }
    WatchdogIntEnable(WATCHDOG0_BASE); // this is to enable the actual watchdog interrupt
    WatchdogIntTypeSet(WATCHDOG0_BASE, WATCHDOG_INT_TYPE_INT); // this is for enabling watchdog interrupt as well
    WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet() * 10/3); //set the watchdog timer to 10 seconds
    WatchdogEnable(WATCHDOG0_BASE); //this is to turn on watchdog interrupt
}
