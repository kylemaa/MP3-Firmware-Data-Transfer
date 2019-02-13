/*
 * UART_DR.hpp
 *
 *  Created on: Oct 6, 2018
 *      Author: David Brassfield
 */

#ifndef UART_DR_HPP_
#define UART_DR_HPP_

#include <stdio.h>
#include "utilities.h"
#include "io.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "uart0_min.h"
#include "gpio.hpp"
#include "queue.h"
#include <string>

enum Uni_channel {
    UART0, UART2, UART3
};

class UART_D {

public:

    /*
     * Constructor for UART_D driver objects
     * Sets of the queue that will be used for each separate UART module usabe on the SJOne controller
     * Enables interrupts for both UART2 and UART3 interrupts
     */
  
    UART_D()
    {
        Uart2_RX_queue = xQueueCreate(100, sizeof(uint8_t));
        Uart3_RX_queue = xQueueCreate(100, sizeof(uint8_t));
        NVIC_EnableIRQ(UART2_IRQn);
        NVIC_EnableIRQ(UART3_IRQn);
    }

    /*
     * Transmit function for sending out data packets 5~8 bits from UART Channels
     * Parameters include an enumerated type for the pin and the value that is to be transmitted
     */


    bool transmit(Uni_channel U, uint8_t tr_val)
    {
        switch (U) {
            case (UART0):
                //uart0 not set up can not be used
                return false;
                break;
            case (UART2):
                //send value to Transmit of uart2
                LPC_UART2->THR = tr_val;
                break;
            case (UART3):
                //send value to Transmit of uart2
                LPC_UART3->THR = tr_val;
                break;
            default:
                //error return false
                return false;
        }
        //proper execution return true;
        return true;
    }

    /*
     * initialization for UAET channels UART2 and UART3
     * Sets PCONP(power) register values, CLOCK, Baude rate, PINMODE for RX and TX
     * related to each channel and disables both pull up and pull down resistors for
     * every UART transmit and receive pin
     *
     * Parameter taken is the enumerated UART channel lable
     */

    void initialize(Uni_channel u)
    {
        LPC_UART_TypeDef *uart_lib;
        switch(u)
               {
                 case UART0 :
                   uart_lib = LPC_UART2;
                   break;
                 case UART2:
                   uart_lib = LPC_UART2;
                   break;
                 case UART3:
                   uart_lib = LPC_UART3;
                   break;
               }
               switch(u)
               {
                   case UART2:
                       LPC_SC -> PCONP |= (1<<24);
                       // SET up clock 00 reflects to uart0 clk = sysclk/4  48Mhz / 4 - 12Mhz
                      LPC_SC -> PCLKSEL0 &= ~(1<<16);
                      LPC_SC -> PCLKSEL0 &= ~(1<<17);
                       break;
                   case UART3:
                       //power up defaulted to 0 on startup, bit 25 for UART3
                       LPC_SC -> PCONP |= (1<<25);
                       // SET up clock 00 reflects to uart0 clk = sysclk/4  48Mhz / 4 - 12Mhz
                       LPC_SC -> PCLKSEL0 &= ~(1<<18);
                       LPC_SC -> PCLKSEL0 &= ~(1<<19);
                       break;
               }
               //set DLAB or DIVISER LATCH FOR SHADOW MEM FOR DLL and DLM access
               uart_lib -> LCR |= (1<<7);
               //set DLL and DLM for baud rate
               uart_lib -> DLM = 0;
               uart_lib -> DLL = 19;
               //reset DLAB to revert shadow mem back
               uart_lib -> LCR &= ~(1<<7);
               uart_lib -> LCR |= (3);
               //endable FIFO for UART3
               uart_lib -> FCR |= (1);
               //set up TXD3 in 4.28 val 11 selects TXD3 function
               LPC_PINCON -> PINSEL9 |= (1<<24);
               LPC_PINCON -> PINSEL9 |= (1<<25);
               //set up TXD pun P2.8 TXD2
               LPC_PINCON -> PINSEL4 &= ~(1<<16);
               LPC_PINCON -> PINSEL4 |= (1<<17);
               //set up RXD3 in 4.29 val 11 selects RXD3 function
               LPC_PINCON -> PINSEL9 |= (1<<26);
               LPC_PINCON -> PINSEL9 |= (1<<27);
               //set up TXD pun P2.9 RXD2
               LPC_PINCON -> PINSEL4 &= ~(1<<18);
               LPC_PINCON -> PINSEL4 |= (1<<19);
               //set up pin modes for TXD3 and RXD3
               //set up Pin 4.28 and 4.29 as neither pull up or pull down
               // val of 10 sets as both pull up and pull down disabled
               //4.28 set PINMODE9 [25:24]
               LPC_PINCON -> PINMODE9 &= ~(1<<24);
               LPC_PINCON -> PINMODE9 |= (1<<25);
               //2.8
               LPC_PINCON -> PINMODE9 &= ~(1<<16);
               LPC_PINCON -> PINMODE9 |= (1<<17);
               //4.29 set set PINMODE9 [27:26]
               LPC_PINCON -> PINMODE9 &= ~(1<<26);
               LPC_PINCON -> PINMODE9 |= (1<<27);
               //2.9
               LPC_PINCON -> PINMODE9 &= ~(1<<18);
               LPC_PINCON -> PINMODE9 |= (1<<19);
               uart_lib -> IER |= 1;
           }

    QueueHandle_t Uart2_RX_queue;
    QueueHandle_t Uart3_RX_queue;
private:

};
UART_D UART_HANDLE;

/*
 * interrupt function that is referenced by Vector table
 * for UART3 interrupts
 * no parameters taken
 */
void my_uart3_rx_intr(void)
{
    uint8_t Next_char = 100;

    //get data from interupt in reg
    Next_char = LPC_UART3->RBR;
    //send data to the que
    //printf("passed value on uart 3 %x \n", Next_char);
    xQueueSendFromISR(UART_HANDLE.Uart3_RX_queue, &Next_char, NULL);
}

/*
 * interrupt function that is referenced by Vector table
 * for UART2 interrupts
 * no parameters taken
 */
void my_uart2_rx_intr(void)
{
    uint8_t Next_char = 100;

    //get data from interupt in reg
    Next_char = LPC_UART2->RBR;
    //send data to the que
    //printf("passed value %x \n", Next_char);
    xQueueSendFromISR(UART_HANDLE.Uart2_RX_queue, &Next_char, NULL);
}

/*
 * function overites vector table references with
 *  above interrupt functions my_uart2_rx_intr and my_uart3_rx_intr
 *  no parameters taken
 */

void attach_interupts()
{
    isr_register(UART3_IRQn, my_uart3_rx_intr);
    isr_register(UART2_IRQn, my_uart2_rx_intr);
}

#endif /* UART_DR_HPP_ */
