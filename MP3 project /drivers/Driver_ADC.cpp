/*
 * Driver_ADC.h
 *
 *  Created on: Sep 19, 2018
 *      Author: David Brassfield
 */

#ifndef DRIVER_ADC_HPP_
#define DRIVER_ADC_HPP_

#include <stdio.h>
#include "utilities.h"
#include "io.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "uart0_min.h"
#include "gpio.hpp"

enum Pin
    {
        k0_25,       // AD0.2 <-- Light Sensor -->
        k0_26,       // AD0.3
        k1_30,       // AD0.4
        k1_31,       // AD0.5

        /* These ADC channels are compromised on the SJ-One,
         * hence you do not need to support them
         */
        // k0_23 = 0,   // AD0.0
        // k0_24,       // AD0.1
        // k0_3,        // AD0.6
        // k0_2         // AD0.7
    };



class D_ADC
{
public:


    // Nothing needs to be done within the default constructor
    D_ADC()
    {
        //do nothing

    }

    /**
    * 1) Powers up ADC peripheral
    * 2) Set peripheral clock
    * 2) Enable ADC
    * 3) Select ADC channels
    * 4) Enable burst mode
    */
    void AdcInitBurstMode()
    {




        //Peripheral reg set to enable ADC (set PCADC bit)
        LPC_SC -> PCONP |= (1<< 12);
        LPC_ADC->ADCR |= (1<<21);


        // clock ADCR [15:8] (CLKDIV set to 0)
        LPC_ADC-> ADCR &=  ~(1<<8|1<<9|1<<10|1<<11|1<<12|1<<13|1<<14|1<<15);
        //set pcl_ADC to 0 to equal sysclk/4    48Mhz / 4 - 12Mhz
        LPC_SC->PCLKSEL0 |= (1 <<24 );
        LPC_SC->PCLKSEL0 |= (1 <<25 );
        //set burst for ADCR
        LPC_ADC -> ADCR |= (1<<16);


        //set up 0.25 for default
        //select ADCR 0 for  light channel (SELECTED ad0) defult value
        //LPC_ADC-> ADCR &= ~(1<<0); // clear defult
        //LPC_ADC-> ADCR |= (1<<2);
        //pinsel1
        //15:14 P0.23[1] GPIO Port 0.23 AD0.0 I2SRX_CLK CAP3.0 00 Light sensor
        //set 0.23 pin mode to ADC value 01 set defult mode for this pin
        //LPC_PINCON->PINSEL1 |= (1<<18);
        //LPC_PINCON->PINSEL1 &= ~(1<<19);
        //pinsel3
        //29:28 P1.30 GPIO Port 1.30 Reserved VBUS AD0.4 00
        //31:30 P1.31 GPIO Port 1.31 Reserved SCK1 AD0.5 00



    }

    void AdcInitNBurstMode()
    {

        //Peripheral reg set to enable ADC (set PCADC bit)
        LPC_SC -> PCONP |= (1<< 12);
        LPC_ADC->ADCR |= (1<<21);


        // clock ADCR [15:8] (CLKDIV set to 0)
        LPC_ADC-> ADCR &=  ~(1<<8|1<<9|1<<10|1<<11|1<<12|1<<13|1<<14|1<<15);
        //set pcl_ADC to 0 to equal sysclk/4    48Mhz / 4 - 12Mhz
        LPC_SC->PCLKSEL0 &= ~(1 <<24 );
        LPC_SC->PCLKSEL0 &= ~(1 <<25 );

        LPC_ADC -> ADCR &= ~(1<<24);
        LPC_ADC -> ADCR &= ~(1<<25);
        LPC_ADC -> ADCR &= ~(1<<26);


        //set up 0.25 for default
        //select ADCR 0 for  light channel (SELECTED ad0) defult value
        //LPC_ADC-> ADCR &= ~(1<<0); // clear defult
        //LPC_ADC-> ADCR |= (1<<2);
        //pinsel1
        //15:14 P0.23[1] GPIO Port 0.23 AD0.0 I2SRX_CLK CAP3.0 00 Light sensor
        //set 0.23 pin mode to ADC value 01 set defult mode for this pin
        //LPC_PINCON->PINSEL1 |= (1<<18);
        //LPC_PINCON->PINSEL1 &= ~(1<<19);
        //pinsel3
        //29:28 P1.30 GPIO Port 1.30 Reserved VBUS AD0.4 00
        //31:30 P1.31 GPIO Port 1.31 Reserved SCK1 AD0.5 00
    }

    /**
    * 1) Selects ADC functionality of any of the ADC pins that are ADC capable
    *
    * @param pin is the LabAdc::Pin enumeration of the desired pin.
    *
    * WARNING: For proper operation of the SJOne board, do NOT configure any pins
    *           as ADC except for 0.26, 1.31, 1.30
    */
    void AdcSelectPin(Pin pin)
    {

//        LPC_PINCON -> PINSEL1 &= ~(0<<18);
//        LPC_PINCON -> PINSEL1 &= ~(0<<19);
//        LPC_PINCON -> PINSEL1 &= ~(0<<20);
//        LPC_PINCON -> PINSEL1 &= ~(0<<21);
//        LPC_PINCON -> PINSEL3 &= ~(0<<28);
//        LPC_PINCON -> PINSEL3 &= ~(0<<29);
//        LPC_PINCON -> PINSEL3 &= ~(0<<30);
//        LPC_PINCON -> PINSEL3 &= ~(0<<31);


        //to change each pin ADC sel must be cleared, the correct ADCR pin must be set high, the specific pin mode
        //must be set to ADC mode.

        // k0_25,       // AD0.2 <-- Light Sensor -->
        // k0_26,       // AD0.3
        // k1_30,       // AD0.4
        // k1_31,       // AD0.5
        // k0_23 = 0,   // AD0.0
        // k0_24,       // AD0.1
        // k0_3,        // AD0.6
        // k0_2         // AD0.7

        //RESET ADC SEL located ADC[7:0]; All set to 0. only one can be set ant any time
        LPC_ADC -> ADCR &= ~((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5)| (1 << 6) | (1<<7));
        //printf("set ADCR SEL to %lX \n",LPC_ADC -> ADCR);


        switch (pin)
        {
            case k0_25:
            //set ADC control pin to high for respective pin;
            LPC_ADC -> ADCR |= (1 << 2);
            //set ADC pin mode to ADC mode 01 for channel 0 pins 11 for channel 1 pins
            LPC_PINCON -> PINSEL1 |= (1<<18);
            LPC_PINCON -> PINSEL1 &= ~(1<<19);



            break;

            case k0_26:
            LPC_ADC -> ADCR |= (1 << 3);
            LPC_PINCON -> PINSEL1 |= (1<<20);
            LPC_PINCON -> PINSEL1 &= ~(1<<21);

            break;

            case k1_30:
            LPC_ADC -> ADCR |= (1 << 4);
            LPC_PINCON -> PINSEL3 |= (1<<28);
            LPC_PINCON -> PINSEL3 |= (1<<29);

            break;

            case k1_31:
            LPC_ADC -> ADCR |= (1 << 5);
            LPC_PINCON -> PINSEL3 |= (1<<30);
            LPC_PINCON -> PINSEL3 |= (1<<31);

            break;

            default: //sets up 0.25 for the light sensor defult case though this should never run;
            LPC_ADC -> ADCR |= (1 << 2);
            LPC_PINCON -> PINSEL1 |= (1<<18);
            LPC_PINCON -> PINSEL1 &= ~(1<<19);

        }

        //printf("set ADCR SEL to %lX \n",LPC_ADC -> ADCR);


    }

    /**
    * 1) Returns the voltage reading of the 12bit register of a given ADC channel
    *
    * @param channel is the number (0 through 7) of the desired ADC channel.
    */
    float ReadAdcVoltageByChannel(Pin pin)
    {

        // k0_25,       // AD0.2 <-- Light Sensor -->
        // k0_26,       // AD0.3
        // k1_30,       // AD0.4
        // k1_31,       // AD0.5

        uint32_t returnval;

        switch(pin)
        {
        case k0_25:
            returnval = LPC_ADC -> ADDR2 ;
            break;
        case k0_26:
            returnval = LPC_ADC -> ADDR3;
            break;
        case k1_30:
            returnval = LPC_ADC -> ADDR4;
            break;
        case k1_31:
            returnval = LPC_ADC -> ADDR5;
            break;
        default :
        returnval = 0;
        }


        return (returnval &= ~(65535<<16));

    }

    uint32_t readgloblevolt()
    {

        //
        LPC_ADC -> ADCR &= ~(1<<24);
        LPC_ADC -> ADCR |= (1<<24);
        delay_ms(10);
        int i = 0;
        uint32_t temp = LPC_ADC -> ADGDR;
        uint32_t channel;

       // printf("Temp %X\n",temp );
        //printf("go %X \n",temp & (1<<31));


            while(!(temp & (1<<31)))
            {
                i = i+1;

                if ( i >1000)
                {
                    printf("stuck and broke free\n");
                    break;
                }

               temp = LPC_ADC -> ADGDR;
            }

            LPC_ADC -> ADCR &= ~(1<<24);

            channel = temp;
            channel = channel &= ~(0xF8 <<27);
            channel = (channel >> 24);
            printf("channel read %X \n", channel);


            temp = ((temp) &= ~(0xFFFF<<16));    //FFFF dec val 65535
            temp = (temp >> 4);
            return temp;
    }

};




D_ADC ADC;



#endif /* DRIVER_ADC_HPP_ */