#ifndef __GPIO_INTERRUPTS_H__
#define __GPIO_INTERRUPTS_H__

#include "printf_lib.h"

enum InterruptCondition 
{
    kRisingEdge,
    kFallingEdge,
    kBothEdges,
};

/**
 * Typdef a function pointer which will help in code readability
 * For example, with a function foo(), you can do this:
 * IsrPointer function_ptr = foo;
 * OR
 * IsrPointer function_ptr = &foo;
 */
typedef void (*IsrPointer)(void);

class LabGpioInterrupts
{
 public:
    /**
     * Optional: LabGpioInterrupts could be a singleton class, meaning, only one instance can exist at a time.
     * Look up how to implement this. It is best to not allocate memory in the constructor and leave complex
     * code to the Initialize() that you call in your main()
     */
    LabGpioInterrupts(){
        Initialize();
    }

    /**
     * This should configure NVIC to notice EINT3 IRQs; use NVIC_EnableIRQ()
     */
    void Initialize(){
        NVIC_EnableIRQ(EINT3_IRQn); // enables interrupt
    }

    /**
     * This handler should place a function pointer within the lookup table for the HandleInterrupt() to find.
     *
     * @param[in] port         specify the GPIO port, and 1st dimension of the lookup matrix
     * @param[in] pin          specify the GPIO pin to assign an ISR to, and 2nd dimension of the lookup matrix
     * @param[in] pin_isr      function to run when the interrupt event occurs
     * @param[in] condition    condition for the interrupt to occur on. RISING, FALLING or BOTH edges.
     * @return should return true if valid ports, pins, isrs were supplied and pin isr insertion was successful
     */
    bool AttachInterruptHandler(uint8_t port, uint32_t pin, IsrPointer pin_isr, InterruptCondition condition){

        /*conditonals creating interrupt pins at certain ports*/
            if(port == 0){
                // setting interrupt pin at port 0, set at passed parameter pin. EX. port = 0, pin = 3. interrupt pin set up at P0_3
                if(pin == 0 || pin == 1 || pin == 26 || pin == 29 || pin == 30){
                    LPC_GPIO0->FIODIR &= ~(1<<pin);
                    LPC_GPIOINT->IO0IntEnR |= (1<<pin);
                    pin_isr_map[port][pin] = pin_isr;
                }
                else{
                    return false;
                }
            }
            else if(port == 2){
                // setting interrupt pin at port 2, set at passed parameter pin. EX. port = 0, pin = 6. interrupt pin set up at P2_6
                if(pin <= 10){
                    LPC_GPIO2->FIODIR &= ~(1<<pin);
                    LPC_GPIOINT->IO2IntEnR |= (1<<pin);
                    pin_isr_map[port-1][pin] = pin_isr;

                }
                else{
                    return false;
                }
            }
            else{
                return false;
            }
            u0_dbg_printf("port = %lu, pin = %lu", port, pin);
            /* add function pointer to lookup matrix */ 
            //pin_isr_map[port][pin] = pin_isr;

            /*registers pin_isr in ISR*/
            //isr_register(EINT3_IRQn, pin_isr); // registering a callback function for an interrupt

            /*conditionals for rising/falling/both edge triggers*/
            if(condition == kRisingEdge){
                LPC_SC->EXTPOLAR |= 1;
            }
            else if(condition == kFallingEdge){
                LPC_SC->EXTPOLAR |= 0;

            }
            else if(kBothEdges){
                LPC_SC->EXTMODE |= 1;
            }
            else{
                return false;
            }
            //successfully registered in ISR and valid ports were supplied.
            u0_dbg_printf("ISR Registered.\n");
            return true;

    }
    
    /**
     * This function is invoked by the CPU (through Eint3Handler) asynchronously when a Port/Pin
     * interrupt occurs. This function is where you will check the Port status, such as IO0IntStatF,
     * and then invoke the user's registered callback and find the entry in your lookup table.
     *
     * VERY IMPORTANT!
     *  - Be sure to clear the interrupt flag that caused this interrupt, or this function will be called
     *    repetitively and lock your system.
     *  - NOTE that your code needs to be able to handle two GPIO interrupts occurring at the same time.
     */
    void HandleInterrupt(){
        /*checking interrupt status on port 0*/
        void (*function_ptr)(void);
        u0_dbg_printf("HandleInterrupt entered\n");


        /*checking interrupt status on port 0)*/
        if(LPC_GPIOINT->IntStatus & (1<<0)){

            /*checking interrupt status (rising and falling) on individual ports*/

            if(LPC_GPIOINT->IO0IntStatF & (1<<0) || LPC_GPIOINT->IO0IntStatR & (1<<0)){
                    LPC_GPIOINT->IO0IntClr = (1<<0); // clear interrupt
                    function_ptr = pin_isr_map[0][0]; // looks up in isr_map for ISR
                    u0_dbg_printf("Pin 0, ");
            }
            else if(LPC_GPIOINT->IO0IntStatF & (1<<1) || LPC_GPIOINT->IO0IntStatR & (1<<1)){
                    LPC_GPIOINT->IO0IntClr = (1<<1);
                    function_ptr = pin_isr_map[0][1];
                    u0_dbg_printf("Pin 1, ");
            }
            else if(LPC_GPIOINT->IO0IntStatF & (1<<26) || LPC_GPIOINT->IO0IntStatR & (1<<26)){
                    LPC_GPIOINT->IO0IntClr = (1<<26);
                    function_ptr = pin_isr_map[0][26];
                    u0_dbg_printf("Pin 26, ");
            }
            else if(LPC_GPIOINT->IO0IntStatF & (1<<29) || LPC_GPIOINT->IO0IntStatR & (1<<29)){
                    LPC_GPIOINT->IO0IntClr = (1<<29);
                    function_ptr = pin_isr_map[0][29];
                    u0_dbg_printf("Pin 29, ");
            }
            else if(LPC_GPIOINT->IO0IntStatF & (1<<30) || LPC_GPIOINT->IO0IntStatR & (1<<30)){
                    LPC_GPIOINT->IO0IntClr = (1<<30);
                    function_ptr = pin_isr_map[0][30];
                    u0_dbg_printf("Pin 30, ");
            }

            u0_dbg_printf("Port 0 Interrupt handled.\n");
            function_ptr(); // calls ISR found in isr_map

        }

        /*checking interrupt status on port 2*/
        else if(LPC_GPIOINT->IntStatus & (1<<2)){

            // if(LPC_GPIOINT->IO2IntStatF & (1<<0) || LPC_GPIOINT->IO2IntStatR & (1<<0)){
            //     LPC_GPIOINT->IO2IntClr = (1<<0);
            // }
            // else if(LPC_GPIOINT->IO2IntStatF & (1<<1) || LPC_GPIOINT->IO2IntStatR & (1<<1)){
            //     LPC_GPIOINT->IO2IntClr = (1<<1);
            // }

            int i, temp;


            /*loops from P2_0 to P2_9 looking for pin that is interrupted*/
            for(i = 0; i<10; i++){


                if(LPC_GPIOINT->IO2IntStatF & (1<<i) || LPC_GPIOINT->IO2IntStatR & (1<<i)){
                    LPC_GPIOINT->IO2IntClr = (1<<i);
                    function_ptr = pin_isr_map[1][i];
                    temp = i;
                }

            }

            function_ptr();
            u0_dbg_printf("Pin %d, Port 2 Interrupt handled.\n", temp);

        }

    }


 private:
    /**
     * Allocate a lookup table matrix here of function pointers (avoid dynamic allocation)
     * Upon AttachInterruptHandler(), you will store the user's function callback
     * Upon the EINT3 interrupt, you will find out which callback to invoke based on Port/Pin status.
     */
    IsrPointer pin_isr_map[2][32];
};

#endif