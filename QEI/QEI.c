/*
 * QEI.c
 *
 *  Created on: 7 Jul 2020
 *      Author: user
 */

#include "QEI.h"

void initQEI(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0)){}
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    ROM_GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    ROM_GPIOPinConfigure(GPIO_PD6_PHA0);
    ROM_GPIOPinConfigure(GPIO_PD7_PHB0);

    ROM_QEIDisable(QEI0_BASE);
    ROM_QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    ROM_QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET
                    | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP, 10000);
//    ROM_QEIVelocityConfigure(QEI0_BASE,QEI_VELDIV_1,ROM_SysCtlClockGet()/2);
//    ROM_QEIVelocityEnable(QEI0_BASE);
//    ROM_QEIIntEnable(QEI0_BASE,QEI_INTINDEX|QEI_INTTIMER);
//    QEIIntRegister(QEI0_BASE, &QEI_ISR);
    ROM_QEIEnable(QEI0_BASE);
}

