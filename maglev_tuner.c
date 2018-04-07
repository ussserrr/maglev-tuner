#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/fpu.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"

#include "pid.h"

#define LED GPIO_PIN_0  // PN0
#define BUTTON GPIO_PIN_0  // PJ0
void ButtonDown(void);
uint32_t g_ui32SysClock;

// PWM
#define PWM_FREQUENCY 8000  // Hz (0.125ms period)
//#define PWM_FREQUENCY 2000  // magic
volatile uint32_t ui32PWMLoad;
volatile uint32_t ui32PWMClock;

// ADC
#define ADCSequencer 1  // Four measurements
uint32_t ui32ADCValues[4];
void TimerADC_Handler(void);
void adc_handler(void);

// PID
PIDdata PIDdataMagLev;
float feedback;
float valueToPWM;

char buffer[50];
float map(uint32_t value, uint32_t min_in, uint32_t max_in, float min_out, float max_out);


void main(void) {
    ROM_IntMasterDisable();

//    ROM_FPULazyStackingEnable();
    ROM_FPUEnable();

    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // LED (for any debug purposes)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, LED);
    ROM_GPIOPinWrite(GPIO_PORTN_BASE, LED, LED);
    ROM_GPIOPinWrite(GPIO_PORTN_BASE, LED, 0);

    // Button (PJ0)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, BUTTON);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, BUTTON, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    ROM_GPIOIntDisable(GPIO_PORTJ_BASE, BUTTON);
    ROM_GPIOIntClear(GPIO_PORTJ_BASE, BUTTON);
    GPIOIntRegister(GPIO_PORTJ_BASE, ButtonDown);
    ROM_GPIOIntTypeSet(GPIO_PORTJ_BASE, BUTTON, GPIO_FALLING_EDGE);
    ROM_GPIOIntEnable(GPIO_PORTJ_BASE, BUTTON);

    // UART configuration
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {}
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, g_ui32SysClock);
//    UARTprintf("DID0: %x\n", HWREG(SYSCTL_DID0));

    PID_Init(&PIDdataMagLev);
    PIDdataMagLev.setpoint = 3412.0f;  // 2.75V

    // PWM configuration (PG0) (electromagnet driver)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) {}
    ROM_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG)) {}
    ROM_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);
    ROM_GPIOPinConfigure(GPIO_PG0_M0PWM4);
    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_NO_SYNC);
    ROM_PWMOutputInvert(PWM0_BASE, PWM_OUT_4_BIT, true);
    ui32PWMClock = g_ui32SysClock/8;
    ui32PWMLoad = (ui32PWMClock/PWM_FREQUENCY) - 1;
    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, ui32PWMLoad);
    UARTprintf("set val: %d, true val: %d\n", ui32PWMLoad, ROM_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2));

    // Timer configuration
    uint32_t ui32TimerPeriod;
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ui32TimerPeriod = g_ui32SysClock / 1000;
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ui32TimerPeriod - 1);
    // Timer interrupt
    ROM_TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER0_BASE, TIMER_TIMA_TIMEOUT, TimerADC_Handler);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // ADC configuration (PE3: Hall sensor)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {}
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {}
    ROM_ADCHardwareOversampleConfigure(ADC0_BASE, 8);
    ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    ROM_ADCSequenceConfigure(ADC0_BASE, ADCSequencer, ADC_TRIGGER_PROCESSOR, 0);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, ADCSequencer, 0, ADC_CTL_CH3);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, ADCSequencer, 1, ADC_CTL_CH2);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, ADCSequencer, 2, ADC_CTL_CH1);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, ADCSequencer, 3, ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);  // PE3
    ROM_ADCIntEnable(ADC0_BASE, ADCSequencer);
    ADCIntRegister(ADC0_BASE, ADCSequencer, adc_handler);
    ROM_ADCSequenceEnable(ADC0_BASE, ADCSequencer);

    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
    ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
    ROM_IntMasterEnable();

    while (1) {}
}


void TimerADC_Handler(void) {
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ROM_ADCProcessorTrigger(ADC0_BASE, ADCSequencer);
//    ROM_GPIOPinWrite(GPIO_PORTN_BASE, LED, ROM_GPIOPinRead(GPIO_PORTN_BASE, LED)^1);
}


void adc_handler(void) {
    ROM_ADCIntClear(ADC0_BASE, ADCSequencer);
    ROM_ADCSequenceDataGet(ADC0_BASE, ADCSequencer, ui32ADCValues);

    PIDdataMagLev.Kp = map(ui32ADCValues[0], 20, 4095, 0.0f, 50.0f);
    PIDdataMagLev.Ki = map(ui32ADCValues[1], 20, 4095, 0.0f, 20.0f);
    PIDdataMagLev.Kd = map(ui32ADCValues[2], 20, 4095, 0.0f, 5.0f);
    feedback = (float)ui32ADCValues[3];

    valueToPWM = PID_Update(&PIDdataMagLev, feedback);
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, (uint16_t)valueToPWM);
}


void ButtonDown(void) {
    if (ROM_GPIOIntStatus(GPIO_PORTJ_BASE, false) & BUTTON) {
        ROM_GPIOIntClear(GPIO_PORTJ_BASE, BUTTON);
        ROM_SysCtlDelay(g_ui32SysClock/150);

        sprintf(buffer, "%f %f %f\n", PIDdataMagLev.Kp, PIDdataMagLev.Ki, PIDdataMagLev.Kd);
        UARTprintf(buffer);
    }
}


float map(uint32_t value, uint32_t min_in, uint32_t max_in, float min_out, float max_out) {
    if (value < min_in) return min_out;
    if (value > max_in) return max_out;
    return (value - min_in) * (max_out - min_out) / (max_in - min_in) + min_out;
}
