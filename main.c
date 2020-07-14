#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"
#include "driverlib/uart.h"
#include "uartstdio.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

void initConsole(void);
void initPWM(void);
void initQEI(void);
void init(void);
void setPWM(void);
void setDIR(uint8_t);
void initLoopControl(void);

void Timer0AIntHandler(void);
void PF6IntHandler(void);
void Timer0BInthandler(void);

volatile unsigned long Period;

volatile uint16_t uMax;
volatile uint16_t uMin;
volatile int16_t pos;

volatile int16_t pos_d;
volatile float error;
volatile float u,val;
volatile int Kp,Kd,Ki;
volatile float error_i,error_d,error_old;
volatile float dt;
volatile int antiWindup;

int main(void) {

    // Set the clocking to run directly from the crystal.
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);//50mhz
    Period = SysCtlClockGet()/20000; // pwm period = 20khz
    //TimerLoadSet(ui32Base, ui32Timer, ui32Value)

    init();

    //test print
    UARTprintf("hello \n");

}

void init(){
    //enable GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

     //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
     HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
     HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
     HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
     //Set Pins to be PHA0 and PHB0
     GPIOPinConfigure(GPIO_PD6_PHA0);
     GPIOPinConfigure(GPIO_PD7_PHB0);

     //GPIO pin interrupt configure PD6
     GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);
     GPIOIntRegister(GPIO_PORTD_BASE, PF6IntHandler);
     GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_6);


     IntMasterEnable();
     initPWM();
     initConsole();
     initQEI();
     initLoopControl();
}

void initConsole(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0|GPIO_PIN_1);
    UARTStdioConfig(0, 115200, 16000000);
}

void initPWM(void){

    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);

    GPIOPinTypePWM(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, Period-1);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, Period-1);

    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

}

void initQEI(){
    //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
     GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);

     //DISable peripheral and int before configuration
     QEIDisable(QEI0_BASE);
     QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

     // Configure quadrature encoder, use an arbitrary top limit of 1000
     QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 0xFFFFFFFF);

     // Enable the quadrature encoder.
     QEIEnable(QEI0_BASE);

     //Set position to a middle value so we can see if things are working
     QEIPositionSet(QEI0_BASE, 0);
     pos = 0;
}

void initLoopControl(){
    //init value
    u           = 0;// pid value calculate
    pos_d       = 0; // set point
    error       = 0;// pos error
    error_i     = 0;// integral error
    error_d     = 0;// derivative error
    error_old   = 0;// previous pos error

    Kp = 600;
    Ki = 125; //125
    Kd = 75; //15
    antiWindup = 1;
    dt = 0.001;
    //timer0A trigger enable
    TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet()*dt) -1);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void setDIR(uint8_t val){
   // UARTprintf("set direction \n");
    if(val == 0)
    {

        PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);
        PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);

    }
    else
    {
        PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
        PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false);


    }
}
// calibrate
void setPWM(void){

    //rescale to dutycycle %

    val = abs(u);
    val = (val*100.0)/65535.0;

    if(val < 10) val = 10;
    if(val > 95) val = 95;

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6 , val*Period/100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7 , val*Period/100);

//    UARTprintf("set PWM = %d", (val*Period/100));

}

void PF6IntHandler(){
    GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_6);
    pos = QEIPositionGet(QEI0_BASE);
    if(pos >  1320){
        pos = pos - 1320;
        UARTprintf("reset positive \n");
    }
    if(pos < -1320){
        pos = pos + 1320;
        UARTprintf("reset negative \n");
    }
    UARTprintf("%d \n" ,QEIPositionGet(QEI0_BASE));
    if(pos == 0)
    {
        UARTprintf("DONT MOVE\n");
    }
    else
    {
    }

}

void Timer0AIntHandler(){
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
   // UARTprintf("timer0A trigger \n");
    if(pos >= -2 && pos <= 2){
        PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);
        PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false);
    }
    else {
       error = pos_d - pos;
       error_d = (error - error_old)/dt;
       u = Kp*error + Ki*error_i + Kd*error_d;

       if(abs(u) >= 65535 && (((error >= 0) && (error_i >= 0)) || ((error <0) && (error_i < 0))))
       {
           if(antiWindup)
           {
               error_i = error_i;
           }
           else // if no ani windup
           {
               error_i = error_i + dt*1.0*error; // rectangular integration
           }
       }
       else
       {
           error_i = error_i + dt*1.0*error;
       }

       error_old = error; // store for next calculate
       int temp = abs(u);
       if(u>=0)
       {
//           UARTprintf("u value %d \n", u);
//           UARTprintf("u value abs %d \n", temp);
           setDIR(0);
           setPWM();
       }
       else
       {
//           UARTprintf("u value %d \n", u);
//           UARTprintf("u value abs %d \n", temp);
           setDIR(1);
           setPWM();
       }
    }
}
//if u>=0 set Direction turn on PF2 spin backward
//else set Direction of turn on PWM pin PF1 spin forward

void Timer0BIntHandler(void)
{

}
