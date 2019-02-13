#ifndef LAB_SPI_H
#define LAB_SPI_H

#include "LPC17xx.h"

struct nibble{
	unsigned a:4;
};


enum FrameModes
	{
        fm0_0, // CPOL = 0, CPHA = 0
        fm0_1, // CPOL = 0, CPHA = 1
        fm1_0, // CPOL = 1, CPHA = 0
        fm1_1, // CPOL = 1, CPHA = 1
    };


class LabSpi
{
 public:

    /**
     * 1) Powers on SPPn peripheral
     * 2) Set peripheral clock
     * 3) Sets pins for specified peripheral to MOSI, MISO, and SCK
     *
     * @param data_size_select transfer size data width; To optimize the code, look for a pattern in the datasheet
     * @param format is the code format for which synchronous serial protocol you want to use.
     * @param divide is the how much to divide the clock for SSP; take care of error cases such as the value of 0, 1, and odd numbers
     *
     * @return true if initialization was successful
     */
    bool initialize(uint8_t data_size_select, FrameModes format, uint8_t divide){
    	/* 1) Powers on PCSSP0 bit of PCONP register.*/
    	LPC_SC->PCONP &= ~(1<<21); // PCSSP0 @ PCONP[21]. First, clear bit.
    	LPC_SC->PCONP |= (1<<21); // Sets bit. 

    	/* 2) Set peripheral clock.*/
    	LPC_SC->PCLKSEL1 |= (1<<10); // PCLK_SSP @ PCLKSEL0[21:20]. If 01, returned to system clock. 
    	LPC_SC->PCLKSEL1 |= (1<<11); // sets PCLK_SSP to CCLK/8

    	/*takes care of odd values of divide*/
    	if(divide == 0 || divide == 1 || divide % 2 != 0){
    		return false;
    	}

    	LPC_SSP0->CPSR |= divide;// divides PCLK by 8.

    	/* 3) Sets pin for specified peripheral to MOSI, MISO, SCK*/
    	LPC_PINCON->PINSEL0 |= (0x2<<30); // SCK0 @ PINSEL[31:30]. If 10, function SCK.
	    LPC_GPIO0->FIODIR |= (1<<15); // sets P0_15 as an output

	    LPC_PINCON->PINSEL1 |= (0x2<<2); // MISO0 @ PINSEL[3:2]. If 10, function MISO.
	    LPC_GPIO0->FIODIR &= ~(1<<17); // sets P0_17 as an input

	    LPC_PINCON->PINSEL1 |= (0x2<<4); // MOSI0 @ PINSEL[5:4]. If 10, function MOSI.
	    LPC_GPIO0->FIODIR |= (1<<18); // sets P0_18 as an output

	    LPC_SSP0->CR0 &= ~(1<<4); 
    	LPC_SSP0->CR0 &= ~(1<<5);// Frame Format @ SSP0CR0[5:4]. if 00, SPI. 

    	/* setting data_size_select */ 
    	if(data_size_select >=4 && data_size_select <= 16){
    		nibble bitNum; // declares a nibble for data_size_select

    		/* need to subtract 1 from data_size_select.
    		ex. if data_size_select = 16, we need binary 15 or 1111
    		for data size select since it can only hold 4 bits.
    		ex. if data_size_select input is 8, we need binary 7. etc
    		*/

    		bitNum.a = data_size_select - 1; 
    		LPC_SSP0->CR0 |= (bitNum.a << 0); // sets CR0[3:0](data_size_select) to our input

    		/*setting CPOL and CPHA bits depending on FrameModes format*/
    		if(format == fm0_0){
				LPC_SSP0->CR0 &= ~(1<<6); // CPOL(Polarity) @ SSPCR0[6]. If 0, maintains bus clock low between frames. 
			    LPC_SSP0->CR0 &= ~(1<<7); // CPHA(Phase) @ SSPCR0[7]. If 0, Transition away from the inter-frame state of clock line. 
    		}
    		else if(format == fm0_1){
    			LPC_SSP0->CR0 &= ~(1<<6); // CPOL(Polarity) @ SSPCR0[6]. If 0, maintains bus clock low between frames. 
   				LPC_SSP0->CR0 |= (1<<7); // CPHA(Phase) @ SSPCR0[7]. If 1, Transition back to the inter-frame state of clock line. 
    		}
    		else if(format == fm1_0){
    			LPC_SSP0->CR0 |= (1<<6); // CPOL(Polarity) @ SSPCR0[6]. If 1, maintains bus clock high between frames. 
   				LPC_SSP0->CR0 &= ~(1<<7); // CPHA(Phase) @ SSPCR0[7]. If 0, Transition away from the inter-frame state of clock line.
    		}
    		else if(format == fm1_1){
    			LPC_SSP0->CR0 |= (1<<6); // CPOL(Polarity) @ SSPCR0[6]. If 1, maintains bus clock high between frames. 
   				LPC_SSP0->CR0 |= (1<<7); // CPHA(Phase) @ SSPCR0[7]. If 1, Transition back to the inter-frame state of clock line.
    		}

    		LPC_SSP0->CR1 &= ~(1<<1); // SSE(SSP Enable) @ SSP0CR1[1]. If 0, disabled. If 1, enabled.
    		LPC_SSP0->CR1 &= ~(1<<2); // MS(Master/Slave mode) @ SSP0CR1. Can only be written if SSE is 0. If 0, SSP controller acts like a master. 
    		LPC_SSP0->CR1 |= (1<<1); //SSE(SSP Enable) set to 1, enabling the SSP controller.

    		return true;
    	} 

    	else{
    		return false;
    	}
    }

    /**
     * Transfers a byte via SSP to an external device using the SSP data register.
     * This region must be protected by a mutex static to this class.
     *
     * @return received byte from external device via SSP data register.
     */
    uint8_t transfer(uint8_t send){
        nibble bitNum;
        bitNum.a = 7;
        LPC_SSP0->CR0 |= (bitNum.a<<0);
    	LPC_SSP0->DR = send;
    	while(!(LPC_SSP0->SR & (1<<2)));
    	uint8_t result = (LPC_SSP0->DR & 0xFF);
    	return result;
    }

    uint16_t transferWord(uint16_t send){
        nibble bitNum;
        bitNum.a = 15;
        LPC_SSP0->CR0 |= (bitNum.a<<0);
        LPC_SSP0->DR = send;
        while(!(LPC_SSP0->SR & (1<<2)));
        uint16_t result = (LPC_SSP0->DR & 0xFFFF);
        return result;
    }

    // LabSpi();
    // ~LabSpi();
  
 private:
  
};

#endif 