/*
 * LabPwm.cpp
 *
 *  Created on: Dec 8, 2018
 *      Author: nickm
 */

#include <stdio.h>
#include "io.hpp"
#include "uart0_min.h"
#include "LPC17xx.h"
#include <stdio.h>
#include "utilities.h"
#include <stdint.h>
#include "LabPwm.h"
//PWMDriver* PWMDriver::instance = NULL;

uint32_t LabPwm::mCT = 0;

LabPwm::LabPwm()
{
    //
}
void LabPwm::PwmSelectAllPins()
{
    LPC_PINCON->PINSEL4 &= ~(0xFFF << 0);
    LPC_PINCON->PINSEL4 |= (0x01 << 0); // k2_0
    LPC_PINCON->PINSEL4 |= (0x01 << 2); // k2_1
    LPC_PINCON->PINSEL4 |= (0x01 << 4); // k2_2
    LPC_PINCON->PINSEL4 |= (0x01 << 6); // k2_3
    LPC_PINCON->PINSEL4 |= (0x01 << 8); // k2_4
    LPC_PINCON->PINSEL4 |= (0x01 << 10); // k2_5
    //LPC_PINCON->PINSEL4 |= (0x555 << 0); // Select all pins
}

void LabPwm::PwmSelectPin(PWM_PIN pwm_pin_arg)
{
    if(pwm_pin_arg == k2_0)
    {
        LPC_PINCON->PINSEL4 &= ~(0x3 << 0);
        LPC_PINCON->PINSEL4 |= (0x1 << 0);
    }
    else if(pwm_pin_arg == k2_1)
    {
        LPC_PINCON->PINSEL4 &= ~(0x3 << 2);
        LPC_PINCON->PINSEL4 |= (0x1 << 2);
    }
    else if(pwm_pin_arg == k2_2)
    {
        LPC_PINCON->PINSEL4 &= ~(0x3 << 4);
        LPC_PINCON->PINSEL4 |= (0x1 << 4);
    }
    else if(pwm_pin_arg == k2_3)
    {
        LPC_PINCON->PINSEL4 &= ~(0x3 << 6);
        LPC_PINCON->PINSEL4 |= (0x1 << 6);
    }
    else if(pwm_pin_arg == k2_4)
    {
        LPC_PINCON->PINSEL4 &= ~(0x3 << 8);
        LPC_PINCON->PINSEL4 |= (0x1 << 8);
    }
    else if(pwm_pin_arg == k2_5)
    {
        LPC_PINCON->PINSEL4 &= ~(0x3 << 10);
        LPC_PINCON->PINSEL4 |= (0x1 << 10);
    }
}

void LabPwm::PwmInitSingleEdgeMode(uint32_t frequency_Hz)
{
    LPC_SC->PCONP |= (0x1 << 6); // enable PCPWM1 power/clock control bit
    LPC_SC->PCLKSEL0 &= ~(0x3 << 12); // 13:12 peripheral clock selection for PWM1
    LPC_PWM1->PR &= ~(0x1 << 0); // prescalar register set to 0
    LPC_PWM1->CTCR &= ~(0x3 << 0); // [1:0] Timer Mode: the TC is incremented when the Prescale Counter matches the Prescale Register.
    LPC_PWM1->CTCR &= ~(0x3 << 2); // [3:2] PCAP1.0
    LPC_PWM1->MCR |= (0x1 << 1); // reset on PWMMRO: the PWMTC will be reset if PWMMR0 matches it.
    //SetFrequency(frequency_Hz);
    if (frequency_Hz == 0)
    {
        frequency_Hz = 100;
    }
    unsigned long cpuClk = sys_get_cpu_clock();
    mCT = (cpuClk / frequency_Hz);
    LPC_PWM1->MR0 = mCT;
    LPC_PWM1->TCR |= (0x1 << 0); // The PWM Timer Counter and PWM Prescale Counter are enabled for counting.
    LPC_PWM1->TCR |= (0x1 << 3); // PWM enable
    LPC_PWM1->PCR &= ~(0x3 << 0); //1:0 unused
    LPC_PWM1->PCR &= ~(0x1 << 2); // Selects single edge controlled mode for PWM2.
    LPC_PWM1->PCR &= ~(0x1 << 3); // Selects single edge controlled mode for PWM3
    LPC_PWM1->PCR &= ~(0x1 << 4); // Selects single edge controlled mode for PWM4
    LPC_PWM1->PCR &= ~(0x1 << 5); // Selects single edge controlled mode for PWM5
    LPC_PWM1->PCR &= ~(0x1 << 6); // Selects single edge controlled mode for PWM6
    LPC_PWM1->PCR |= (0x1 << 9); // The PWM1 output enabled.
    LPC_PWM1->PCR |= (0x1 << 10); // The PWM2 output enabled.
    LPC_PWM1->PCR |= (0x1 << 11); // The PWM3 output enabled.
    LPC_PWM1->PCR |= (0x1 << 12); // The PWM4 output enabled.
    LPC_PWM1->PCR |= (0x1 << 13); // The PWM5 output enabled.
    LPC_PWM1->PCR |= (0x1 << 14); // The PWM6 output enabled.
    LPC_PWM1->MR1 = 0; // match register 1 to 0
    LPC_PWM1->MR2 = 0; // match register 2 to 0
    LPC_PWM1->MR3 = 0; // match register 3 to 0
    LPC_PWM1->MR4 = 0; // match register 4 to 0
    LPC_PWM1->MR5 = 0; // match register 5 to 0
    LPC_PWM1->MR6 = 0; // match register 6 to 0
}

void LabPwm::SetDutyCycle(PWM_PIN pwm_pin_arg, float duty_cycle_percentage)
{
    if (duty_cycle_percentage < 0)
    {
        duty_cycle_percentage = 0.0;
    }
    else if (duty_cycle_percentage > 1)
    {
        duty_cycle_percentage = 1.0;
    }

    const unsigned int out = mCT * duty_cycle_percentage;

    if(pwm_pin_arg == k2_0)
    {
        LPC_PWM1->MR1 = out;
        LPC_PWM1->LER |= (0x1 << 1);
    }
    else if(pwm_pin_arg == k2_1)
    {
        LPC_PWM1->MR2 = out;
        LPC_PWM1->LER |= (0x1 << 2);
    }
    else if(pwm_pin_arg == k2_2)
    {
        LPC_PWM1->MR3 = out;
        LPC_PWM1->LER |= (0x1 << 3);
    }
    else if(pwm_pin_arg == k2_3)
    {
        LPC_PWM1->MR4 = out;
        LPC_PWM1->LER |= (0x1 << 4);
    }
    else if(pwm_pin_arg == k2_4)
    {
        LPC_PWM1->MR5 = out;
        LPC_PWM1->LER |= (0x1 << 5);
    }
    else if(pwm_pin_arg == k2_5)
    {
        LPC_PWM1->MR6 = out;
        LPC_PWM1->LER |= (0x1 << 6);
    }
}

void LabPwm::BlinkDutyCycle(PWM_PIN pwm_pin_arg, float blink)
{

    while(blink == 1)
    {
    const unsigned int out = mCT * blink;

    if(pwm_pin_arg == k2_0)
    {
        LPC_PWM1->MR1 = out;
        LPC_PWM1->LER |= (0x1 << 1);
    }
    else if(pwm_pin_arg == k2_1)
    {
        LPC_PWM1->MR2 = out;
        LPC_PWM1->LER |= (0x1 << 2);
    }
    else if(pwm_pin_arg == k2_2)
    {
        LPC_PWM1->MR3 = out;
        LPC_PWM1->LER |= (0x1 << 3);
    }
    else if(pwm_pin_arg == k2_3)
    {
        LPC_PWM1->MR4 = out;
        LPC_PWM1->LER |= (0x1 << 4);
    }
    else if(pwm_pin_arg == k2_4)
    {
        LPC_PWM1->MR5 = out;
        LPC_PWM1->LER |= (0x1 << 5);
    }
    else if(pwm_pin_arg == k2_5)
    {
        LPC_PWM1->MR6 = out;
        LPC_PWM1->LER |= (0x1 << 6);
    }
    blink = 0;
    delay_ms(100);
    }
    while(blink == 0)
    {
        const unsigned int out = mCT * blink;

            if(pwm_pin_arg == k2_0)
            {
                LPC_PWM1->MR1 = out;
                LPC_PWM1->LER |= (0x1 << 1);
            }
            else if(pwm_pin_arg == k2_1)
            {
                LPC_PWM1->MR2 = out;
                LPC_PWM1->LER |= (0x1 << 2);
            }
            else if(pwm_pin_arg == k2_2)
            {
                LPC_PWM1->MR3 = out;
                LPC_PWM1->LER |= (0x1 << 3);
            }
            else if(pwm_pin_arg == k2_3)
            {
                LPC_PWM1->MR4 = out;
                LPC_PWM1->LER |= (0x1 << 4);
            }
            else if(pwm_pin_arg == k2_4)
            {
                LPC_PWM1->MR5 = out;
                LPC_PWM1->LER |= (0x1 << 5);
            }
            else if(pwm_pin_arg == k2_5)
            {
                LPC_PWM1->MR6 = out;
                LPC_PWM1->LER |= (0x1 << 6);
            }
            blink = 1;
            delay_ms(100);
    }
}



