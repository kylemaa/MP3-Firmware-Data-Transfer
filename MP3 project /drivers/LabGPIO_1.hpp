#ifndef LABGPIO1_H
#define LABGPIO1_H

#include "LPC17xx.h"

class LabGPIO_1
{
private:
    /**
     * port, pin and any other variables should be placed here.
     * NOTE: that the state of the pin should not be recorded here. The true
     *      source of that information is embedded in the hardware registers
     */
     uint32_t pinNumber;
public:
    /**
     * You should not modify any hardware registers at this point
     * You should store the port and pin using the constructor.
     *
     * @param {uint8_t} pin  - pin number between 0 and 32
     */
	LabGPIO_1(uint32_t paramPin){
        pinNumber = paramPin;
    }
    /**
     * Should alter the hardware registers to set the pin as an input
     */
	void setAsInput(){

      LPC_GPIO1->FIODIR &= ~(1 << pinNumber);
    }
    /**
     * Should alter the hardware registers to set the pin as an output
     */   
	void setAsOutput(){
        LPC_GPIO1->FIODIR |= (1 << pinNumber);
    }
    /**
     * Should alter the set the direction output or input depending on the input.
     *
     * @param {bool} output - true => output, false => set pin to input
     */
    void setDirection(bool output){
    /**
     * Should alter the hardware registers to set the pin as high
     */
    if (output == false){
        LPC_GPIO1->FIODIR &= ~(1 << pinNumber);
    }

    else{
        LPC_GPIO1->FIODIR |= (1 << pinNumber);
    }

    }
    void setHigh(){
        LPC_GPIO1->FIOSET = (1 << pinNumber);
    }
    /**
     * Should alter the hardware registers to set the pin as low
     */
    void setLow(){
        LPC_GPIO1->FIOCLR = (1 << pinNumber);
    }
    /**
     * Should alter the hardware registers to set the pin as low
     *
     * @param {bool} high - true => pin high, false => pin low
     */
    void set(bool high){
        if(high == true){
            LPC_GPIO1->FIOSET = (1 << pinNumber);
        }
        else{
            LPC_GPIO1->FIOCLR = (1 << pinNumber);
        }
    }
    /**
     * Should return the state of the pin (input or output, doesn't matter)
     *
     * @return {bool} level of pin high => true, low => false
     */
	bool getLevel(){
            if(LPC_GPIO1->FIOPIN & (1 << pinNumber)){
                return true;
            }
            else{
                return false;
            }
    }
};

#endif