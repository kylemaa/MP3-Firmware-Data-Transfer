/*
 * LabPwm.h
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

#ifndef LABPWM_H_
#define LABPWM_H_

class LabPwm
{
    private:
    static uint32_t mCT;
    public:

    enum PWM_PIN
    {
        k2_0, // PWM1.1
        k2_1, // PWM1.2
        k2_2, // PWM1.3
        k2_3, // PWM1.4
        k2_4, // PWM1.5
        k2_5, // PWM1.6
    };
/// Nothing needs to be done within the default constructor
    LabPwm();
    /**
    * 1) Select PWM functionality on all PWM-able pins.
    */
    void PwmSelectAllPins();
    /**
    * 1) Select PWM functionality of pwm_pin_arg
    *
    * @param pwm_pin_arg is the PWM_PIN enumeration of the desired pin.
    */
    void PwmSelectPin(PWM_PIN pwm_pin_arg);
    /**
    * Initialize your PWM peripherals. See the notes here:
    *
    http://books.socialledge.com/books/embedded-drivers-real-time-operating-systems/page/pwm-%28
    pulse-width-modulation%29
    *
    * In general, you init the PWM peripheral, its frequency, and initialize your PWM
    channels and set them to 0% duty cycle
    *
    * @param frequency_Hz is the initial frequency in Hz.
    */
    void PwmInitSingleEdgeMode(uint32_t frequency_Hz);
    /**
    * 1) Convert duty_cycle_percentage to the appropriate match register value (depends on
    current frequency)
    * 2) Assign the above value to the appropriate MRn register (depends on pwm_pin_arg)
    *
    * @param pwm_pin_arg is the PWM_PIN enumeration of the desired pin.
    * @param duty_cycle_percentage is the desired duty cycle percentage.
    */
    void SetDutyCycle(PWM_PIN pwm_pin_arg, float duty_cycle_percentage);
    /**
    * Optional:
    * 1) Convert frequency_Hz to the appropriate match register value
    * 2) Assign the above value to MR0
    *
    * @param frequency_hz is the desired frequency of all pwm pins
    */
    void SetFrequency(uint32_t frequency_Hz);

    void BlinkDutyCycle(PWM_PIN pwm_pin_arg, float blink);

};



#endif /* LABPWM_H_ */
