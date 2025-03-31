//******************ECE 132*************************
//Names: Daniel Gress, Shane Duffy
//Lab section: ECE 132, Section 010
//*************************************************
//Date Started: Tuesday, March 25, 2025
//Date of Last Modification: March 27, 2025
//Lab Assignment: Lab 7
//Lab Due Date: Tuesday, April 1, 2025
//*************************************************
//Purpose of program: Task 2, create a multimeter that calculates voltage,
//resistance, and current from a potentiometer.
//Program Inputs: Analog voltage from ADC channel 0 (PE3)
//Program Outputs: Voltage (mV), resistance (Ohms), and current (uA)
//displayed over UART terminal every 10 seconds
//*************************************************

//file include statements
#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "inc/hw_types.h"

//constant definitions
#define MAX_RESISTANCE 10000.0f   // max pot resistance in ohms
#define SYSTEM_VOLTAGE 3.3f       // system reference voltage
#define ADC_MAX        4095.0f    // 12-bit ADC resolution

//main program
int main(void) {
    unsigned long adcValue;
    float voltage, resistance, current;
    unsigned long voltage_mV, resistance_ohm, current_uA;
    char output[100];
    int i;

    //set system clock to 50 MHz using PLL and 16 MHz crystal
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL |
                   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //initialize UART0 (PA0 = RX, PA1 = TX)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

    //print startup message
    char msg[] = "Multimeter - Task 2 Running\r\n";
    for (i = 0; i < sizeof(msg) - 1; i++) {
        UARTCharPut(UART0_BASE, msg[i]);
    }

    //enable ADC0 and configure PE3 as analog input (AIN0)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); // set PE3 as analog input

    //disable sequencer 0 before configuration
    ADCSequenceDisable(ADC0_BASE, 0);
    //configure ADC0 to use processor trigger and priority 0
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

    //configure step 0 to sample AIN0, trigger interrupt, and end sequence
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0,
                             ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

    //enable ADC0 sequencer 0
    ADCSequenceEnable(ADC0_BASE, 0);

    while (1) {
        //trigger ADC conversion
        ADCProcessorTrigger(ADC0_BASE, 0);
        //wait for ADC conversion to complete
        while (!ADCIntStatus(ADC0_BASE, 0, false)) {}
        ADCIntClear(ADC0_BASE, 0); // clear ADC interrupt
        ADCSequenceDataGet(ADC0_BASE, 0, &adcValue); // get ADC result

        //calculate voltage (in V)
        voltage = (adcValue * SYSTEM_VOLTAGE) / ADC_MAX;

        //calculate resistance based on linear drop from max resistance
        resistance = MAX_RESISTANCE * (1.0f - (voltage / SYSTEM_VOLTAGE));

        //calculate current (I = V / R), convert to microamps
        if (resistance > 0.0f)
            current = (voltage / resistance) * 1e6f;
        else
            current = 0.0f;

        //convert floating-point values to integers for display
        voltage_mV = (unsigned long)(voltage * 1000);
        resistance_ohm = (unsigned long)(resistance);
        current_uA = (unsigned long)(current);

        //format output message to send over UART
        int len = snprintf(output, sizeof(output),
            "Source Voltage: %lu mV\tResistance: %lu Ohms\tCurrent: %lu uA\r\n",
            voltage_mV, resistance_ohm, current_uA);

        //send output to terminal
        for (i = 0; i < len; i++) {
            UARTCharPut(UART0_BASE, output[i]);
        }

        //delay approximately 10 seconds
        SysCtlDelay(SysCtlClockGet() / 3 * 10);
    }
}
