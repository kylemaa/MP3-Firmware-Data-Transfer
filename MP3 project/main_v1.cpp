/**
 * @file
 * @brief This is the application entry point.
 */

#include <stdio.h>
#include "utilities.h"
#include "io.hpp"
#include "FreeRTOS.h"
#include "LPC17xx.h"
#include "LabGPIO_0.hpp"
#include "LabGPIO_1.hpp"
#include "lab_spi.hpp"
#include "gpio_interrupts.hpp"
#include "queue.h"

#include <string.h>
#include "utilities.h"
#include "io.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "uart0_min.h"
#include "gpio.hpp"
#include "ff.h"     /* Declarations of FatFs API */
#include "task.h"               // uxTaskGetSystemState()
#include "utilities.h"          // printMemoryInfo()

#include "array_file.hpp"
#include "Driver_ADC.hpp"


#include "LabPwm.h"

#include "LCD_DRIVER.hpp"

#define mode_reg 0x00
#define vol_reg 0x0B
#define bass_reg 0x02
#define sci_write 0x02
#define sci_read 0x03
#define clk_reg 0x03
#define song_name 3
#define artist_name 33

/*
XDCS FOR SDI, XCS FOR SCI
sine test:
do a hardware reset
set register SM_MODE to SM_TESTS
    SM_MODE[5], set to 1. 
send test command sequence to SDI bus
send 0x53, 0xEF, 0x6E, n, 0000, which defines the sine test to use where
example
-to start test
    bytes[0] = spiKid.transfer(0x53);
    bytes[1] = spiKid.transfer(0xEF);
    bytes[2] = spiKid.transfer(0x6E);
    bytes[3] = spiKid.transfer(0x7E);
    bytes[4] = spiKid.transfer(0x00);
    bytes[5] = spiKid.transfer(0x00);
    bytes[6] = spiKid.transfer(0x00);
    bytes[7] = spiKid.transfer(0x00);
-to end test
    bytes[0] = spiKid.transfer(0x45);
    bytes[1] = spiKid.transfer(0x78);
    bytes[2] = spiKid.transfer(0x69);
    bytes[3] = spiKid.transfer(0x74);
    bytes[4] = spiKid.transfer(0x00);
    bytes[5] = spiKid.transfer(0x00);
    bytes[6] = spiKid.transfer(0x00);
    bytes[7] = spiKid.transfer(0x00);
n[7:5] = sample rate index
n[4:0] = sine skip speed 
example gives 0b01111110. which yields sine frequency 5168Hz
to exit, 0x45, 0x78, 0x69, 0x74, 0 0 0 0
*/

LabSpi spiKid;

LabGPIO_0 xCSpin(0);
LabGPIO_0 xDCSpin(1); 
LabGPIO_0 DREQpin(29);
LabGPIO_1 RSTpin(19);

LabGPIO_1 stop(9);
LabGPIO_1 play(10);
LabGPIO_1 up(14);
LabGPIO_1 down(15);

void initializeDecoderTest(void);
void startSineTest(void);
void endSineTest(void);
void readSetClkReg(void);


//PWM Initialization for LED
LabPwm my_pwm;

//LCD Variables 
bool stopped = true;
bool paused = false;
int current = 0;
int song_index = 0;
bool song_menu = false;
LCD_D lcd;


array_file  list[30];
int len = 0;

bool pause_bool;
bool stop_bool;
QueueHandle_t read_queue;

SemaphoreHandle_t spi_bus_mutex = NULL;
SemaphoreHandle_t knob_control_sem = NULL;
LabGpioInterrupts gpio_interrupt;

//get files in the storage device
//for all mp3 files get file name, song name, artist and size and store it in global list
// File size get from FILINFO
//get song name artist and artist from reading tag at end of file
//This implementation does not travel down into other directories just root all MP3's must be in root '/'


void vPlay_file(void * PvParameters )
{
    /* Get Parameter */

    /* Define Constants Here */

    uint8_t read_buff[1024];             //read data buffer
    UINT read_num = 0;                  //holder for number of bits read in read functions;
    unsigned long int offset = 0;
    FRESULT STAT;                       //Status for FATF function return values
    static FILINFO fno;                 //File info storing information on stored
    FIL file;
    DIR dir;
    uint8_t temp;
    uint8_t Next_val =0;



    while(1)
    {
        if (xQueueReceive(read_queue, &Next_val, 0xFFFFFFFF))
        {

            //queue here to wake task waiting for passed int for song index

            STAT = f_open(&file,list[Next_val].fname,FA_READ);           //open file

            xDCSpin.setLow();

            if(STAT != FR_OK )                                      //close if file encounters error
            {
                printf("error encountered\n");
                printf(" error code: %X \n",STAT);
            }
            printf("made it");
            while (offset + 1024 < list[Next_val].file_size)
            {

                //while loop to check pause condition wait conditition global bool

                while (pause_bool)
                {
                    //cry
                    if(stop_bool)
                    {
                        break;
                    }
                }

                while (stop_bool)
                {
                    stop_bool = 0;
                    pause_bool = 0;
                    break;
                }
                //if condition cheking for stop codition, this condition global bool
                //will brake form loop and close current file
                //this loop will reset this bool



                offset = offset + 1024;
                xDCSpin.setHigh();

                //if spi_bus_mutex is available, read from SD card
                if(xSemaphoreTake(spi_bus_mutex, portMAX_DELAY)){
                
                f_read(&file,&read_buff,1024,&read_num);
                    xSemaphoreGive(spi_bus_mutex);
                }



                //mutex for spi take. if spi_bus_mutex is available, send data to decoder
                if(xSemaphoreTake(spi_bus_mutex, portMAX_DELAY)){
                xDCSpin.setLow();
                for (int i = 0; i < 1024; i= i+32)
                {


                    while(!(DREQpin.getLevel()))
                    {

                    }


                    temp = spiKid.transfer(read_buff[i]);
                    temp = spiKid.transfer(read_buff[i+1]);
                    temp = spiKid.transfer(read_buff[i+2]);
                    temp = spiKid.transfer(read_buff[i+3]);
                    temp = spiKid.transfer(read_buff[i+4]);
                    temp = spiKid.transfer(read_buff[i+5]);
                    temp = spiKid.transfer(read_buff[i+6]);
                    temp = spiKid.transfer(read_buff[i+7]);
                    temp = spiKid.transfer(read_buff[i+8]);
                    temp = spiKid.transfer(read_buff[i+9]);
                    temp = spiKid.transfer(read_buff[i+10]);
                    temp = spiKid.transfer(read_buff[i+11]);
                    temp = spiKid.transfer(read_buff[i+12]);
                    temp = spiKid.transfer(read_buff[i+13]);
                    temp = spiKid.transfer(read_buff[i+14]);
                    temp = spiKid.transfer(read_buff[i+15]);
                    temp = spiKid.transfer(read_buff[i+16]);
                    temp = spiKid.transfer(read_buff[i+17]);
                    temp = spiKid.transfer(read_buff[i+18]);
                    temp = spiKid.transfer(read_buff[i+19]);
                    temp = spiKid.transfer(read_buff[i+20]);
                    temp = spiKid.transfer(read_buff[i+21]);
                    temp = spiKid.transfer(read_buff[i+22]);
                    temp = spiKid.transfer(read_buff[i+23]);
                    temp = spiKid.transfer(read_buff[i+24]);
                    temp = spiKid.transfer(read_buff[i+25]);
                    temp = spiKid.transfer(read_buff[i+26]);
                    temp = spiKid.transfer(read_buff[i+27]);
                    temp = spiKid.transfer(read_buff[i+28]);
                    temp = spiKid.transfer(read_buff[i+29]);
                    temp = spiKid.transfer(read_buff[i+30]);
                    temp = spiKid.transfer(read_buff[i+31]);



                }
                xDCSpin.setHigh();
                //mutex for spi give
                xSemaphoreGive(spi_bus_mutex);
                }
            }

        }

        STAT = f_close (&file);
        STAT = f_closedir (&dir);


    }

}


//LCD Tasks Here

void vTaskPlayFromPause(void * PvParameters)
{
    while(1){
        if(play.getLevel() && paused){
            LE.on(2);
            lcd.clear_lcd();
            paused = false;
            lcd.write("0", "2", "Resuming Song...");
            delay_ms(500);
            lcd.clear_line("2");
            lcd.write("0", "0", "Playing Song!");
            lcd.write("0", "1", list[current].sname);
            LE.off(2);
    }
            vTaskDelay(1);
        }
    }

void vTaskPause(void * PvParameters)
{
    while(1){
        if(up.getLevel() && stopped == false){
            lcd.clear_lcd();    
            lcd.write("0", "0", "Paused!");
            delay_ms(500);
            paused = true;
            //lcd.clear_lcd();

        }
            vTaskDelay(1);
    }

}


void vTaskStop(void * PvParameters)
{
    while(1){
        if(stop.getLevel() && stopped == false){
                LE.on(1);
                lcd.clear_lcd();
                stopped = true;
                lcd.write("0", "0", "Stopped!");
                delay_ms(500);
                lcd.clear_lcd(); 
                lcd.write("5", "0", "Song Menu");
                lcd.write("0", "1", "--------------------");
                lcd.write("2", "2", "Press Up or Down");
                lcd.write("2", "3", "To Select a Song!");
                LE.off(1);


                            }
            vTaskDelay(1);
    }

}

void vTaskPlayAfterMenu(void * PvParameters){
    while(1){
        if(play.getLevel() && stopped && song_menu){
            LE.on(2);
            lcd.clear_lcd();
            stopped = false;
            song_menu = false;
            current = song_index;
            xQueueSend(read_queue, &current, 0);
            lcd.write("0", "0", "Playing Song!");
            lcd.write("0", "1", list[current].sname);
            LE.off(2);
        }
        vTaskDelay(1);
    }

}

void vTaskPlayNoMenu(void * PvParameters){
    while(1){
        if(play.getLevel() && stopped && song_menu == false){
                LE.on(2);
                lcd.clear_lcd();
                stopped = false;
                xQueueSend(read_queue, &current, 0);
                lcd.write("0", "0", "Playing Song!");
                lcd.write("0", "1", list[current].sname);
                LE.off(2);
            }
            vTaskDelay(1);
        }

    }

void vTaskUpMenu(void * PvParameters){
    while(1){
    if(up.getLevel() && stopped){
        LE.on(3);
        song_menu = true;
        song_index++;
        lcd.clear_lcd();
        lcd.write("0", "0", "Pick a Song!");
        lcd.write("0", "1", list[song_index].sname);
        delay_ms(100);
        LE.off(3);
        }
        vTaskDelay(1);
    }

}


void vTaskDownMenu(void * PvParameters)
{
    while(1){
        if(down.getLevel() && stopped){
            LE.on(4);
            song_menu = true;
            song_index--;
            lcd.clear_lcd();
            lcd.write("0", "0", "Pick a Song!");
            lcd.write("0", "1", list[song_index].sname);
            delay_ms(100);
            LE.off(4);
        }

        vTaskDelay(1);
    }
}




void Eint3Handler(void)
{
    u0_dbg_printf("Eint3Handler entered\n");
    gpio_interrupt.HandleInterrupt();
}

void knob_set_ISR(void){
    // may need to set xDCS high here so we don't run into contention
    long yield = 0;
    xSemaphoreGiveFromISR(knob_control_sem, &yield);
    portYIELD_FROM_ISR(yield);
}


//need interupt set up for this
void vBass_vol_treble(void * PvParameters )
{
    /* TODO: MOVE THIS TO INITIALIZE FUNCTION*/
    ADC.AdcInitBurstMode();


    while(1)
    {
        //unlock with semphomore from interrupt
        if(xSemaphoreTake(knob_control_sem, 0xFFFFFFFF)){

            uint16_t ADC_val1;
            uint16_t ADC_val2;
            uint16_t ADC_val3;

            uint8_t bassVal;
            uint8_t trebleVal;
            uint8_t volumeVal;

            ADC.AdcSelectPin(k1_30);

            ADC_val1 = ADC.readgloblevolt();

            ADC.AdcSelectPin(k1_31);

            ADC_val2 = ADC.readgloblevolt();

            ADC.AdcSelectPin(k0_26);

            ADC_val3 = ADC.readgloblevolt();

            //set the bass volume and temble based off of these values obtained
            //take mutex set the controll registers give mutex

            //SCI_BASS reg (0x2)
            //ST_AMPLITUDE (SCI_BASS[15:12])
            // controls treble in 1.5dB steps.
            // SCI_BASS[11:8] control lower limit frequency (doesn't need to be modified)
            //SB_AMPLITUDE (SCI_BASS[7:4])
            // controls bass in 1dB steps
            // SCI_BASS[3:0] control lower limit frequency (doesn't need to be modified)
            // need to take 16bit ADC_val1 and truncate it to 4bits
            // can just take the 4 MSBs of 16 bit value?

            //SCI_VOL reg (0x0B)
            // max volume 0x0000, min volume 0xFEFE
            // 8 MSBs, left channel, 8 LSBs, right channel


            bassVal = ((ADC_val1 >> 12) & 0xF); // getting 4 MSBs
            trebleVal = ((ADC_val2 >> 12) & 0xF); // getting 4 MSBs
            volumeVal = ((ADC_val3 >> 8) & 0xFF); // getting 8 MSBs

            volumeVal = ~(volumeVal);

            bassVal = ((bassVal << 4)); // shifting left bc SCI_BASS[7:4] controls bass
            trebleVal = ((trebleVal << 4)); // shifting left bc SCI_BASS [15:12] controls treble

            if(volumeVal < 0x20){
                volumeVal = 0x20;
            }

            uint8_t dummy_byte;

            // writing to SCI_BASS reg

            //ADD MUTEXT TAKE

            if(xSemaphoreTake(spi_bus_mutex, portMAX_DELAY)){
            while(!(DREQpin.getLevel()));

            xCSpin.setLow();
            dummy_byte = spiKid.transfer(sci_write);
            dummy_byte = spiKid.transfer(bass_reg);
            //word[2] = spiKid.transferWord(0xFFFF);
            dummy_byte = spiKid.transfer(trebleVal);
            dummy_byte = spiKid.transfer(bassVal);
            xCSpin.setHigh();

            //writing to vol_reg
            while(!(DREQpin.getLevel()));

            xCSpin.setLow();
            dummy_byte = spiKid.transfer(sci_write);
            dummy_byte = spiKid.transfer(vol_reg);
            //word[2] = spiKid.transferWord(0xFFFF);
            dummy_byte = spiKid.transfer(volumeVal);
            dummy_byte = spiKid.transfer(volumeVal);
            xCSpin.setHigh();

            while(!(DREQpin.getLevel()));

            xSemaphoreGive(spi_bus_mutex);
        }

            //ADD MUTEX GIVE

            // Eint3Handler(); maybe????
            //xSemaphoreGive(knob_semaphore);
        }
        vTaskDelay(10);

    }

}


void vLED_jazz(void * PvParameters )
{
   while(1)
    {
        my_pwm.PwmInitSingleEdgeMode(1000);
        my_pwm.PwmSelectPin(my_pwm.k2_0);
        my_pwm.PwmSelectPin(my_pwm.k2_1);
        my_pwm.PwmSelectPin(my_pwm.k2_2);

        my_pwm.BlinkDutyCycle(my_pwm.k2_0, 1);
        my_pwm.BlinkDutyCycle(my_pwm.k2_1, 1);
        my_pwm.BlinkDutyCycle(my_pwm.k2_2, 1);

        vTaskDelay(1);
    }

}







FRESULT Load_file_info()
{

    char buff[4];                       //stores the directory. used to store'/'
    uint8_t read_buff[256];             //read data buffer
    UINT read_num = 0;                  //holder for number of bits read in read functions;
    unsigned long int offset = 0;
    FRESULT STAT;                       //Status for FATF function return values
    static FILINFO fno;                 //File info storing information on stored
    FIL file;
    DIR dir;

    strcpy(buff, "/");                  //set directory to root by setting to '/' with nothing after

    STAT = f_opendir(&dir, buff);       //open directory to root directory

    if (STAT == FR_OK)                  //if the directory opened procced to get all the files
    {
        while(1)
        {
            STAT = f_readdir(&dir, &fno);    //read directory items

            if(STAT != FR_OK || fno.fname[0] == 0)     //break on error or end of '/'
            {
                break;
            }

            for( int x = 0; x<13;x++)                  //Set File name
            {
               list[len].fname[x] = fno.fname[x];
            }

            bool mp3 = false;
            for (int y=0; y<12; y ++ )  //loops checks for presence of mp3 or MP3 in file name to confirm it is an mp3 file
            {
                if((list[len].fname[y] == 109) && (list[len].fname[y + 1] == 112) && (list[len].fname[y + 2] == 51) ) //check for mp3
                {
                    mp3 = true;
                }

                if((list[len].fname[y] == 77) && (list[len].fname[y + 1] == 80) && (list[len].fname[y + 2] == 51) ) //check for MP3
                {
                    mp3 = true;
                }
            }

            if(mp3) // if it is an mp3 file it initializes an object with its data.
            {
                                                                    //get needed info from current file

            list[len].file_size = fno.fsize;                        //set file size


            STAT = f_open(&file,list[len].fname,FA_READ);           //open file

            if(STAT != FR_OK )                                      //close if file encounters error
            {
                printf("error encountered\n");
                printf(" error code: %X \n",STAT);
                return STAT;
            }

            offset = (list[len].file_size - 128);                   //set offset to get TAG info for file
            f_lseek(&file,offset);

            f_read(&file,&read_buff,127,&read_num);                 //read the entire TAG

            int count = 0;
                for (int x = 0; x < 30; x ++)                      //get song name info and save it
                    {                                              //song_name enum value for offset in TAG

                        if((read_buff[x + song_name] >= 32 && read_buff[x + song_name] <= 125) )
                        {
                            char temp = read_buff[x + song_name];
                            list[len].sname[count] = temp;
                            count = count + 1;
                        }
                    }
                list[len].sname[count] = '\0';                     //terminate with '\0' to make it a string;
                list[len].sname[30] = '\0';
                count = 0;
                for (int x = 0; x < 30; x ++)                      //get artist name info and save it
                        {                                          //artist_name emum value for offset in TAG
                            if(read_buff[x + artist_name] >= 32 && read_buff[x + artist_name] <= 125)
                            {
                                char temp = read_buff[x + artist_name];
                                list[len].aname[count] = temp;
                                count = count + 1;
                            }
                        }
                list[len].aname[count] = '\0';
                list[len].aname[30] = '\0';

                len = len + 1;
                printf(" / %s located and initialized\n", fno.fname);
            }

        }
    }

return STAT;
}



// void readfile()
// {
//     printf("entered \n");
//     uint8_t read_buff[1024];             //read data buffer
//     UINT read_num = 0;                  //holder for number of bits read in read functions;
//     unsigned long int offset = 0;
//     FRESULT STAT;                       //Status for FATF function return values
//     static FILINFO fno;                 //File info storing information on stored
//     FIL file;
//     DIR dir;
//     uint8_t temp;

//     STAT = f_open(&file,list[6].fname,FA_READ);           //open file

//     xDCSpin.setLow();

//     if(STAT != FR_OK )                                      //close if file encounters error
//     {
//         printf("error encountered\n");
//         printf(" error code: %X \n",STAT);
//     }
//     printf("made it");
//     while (offset + 1024 < list[6].file_size)
//     {

//         //f_lseek(&file,offset);
//         offset = offset + 1024;
//         xDCSpin.setHigh();
//         f_read(&file,&read_buff,1024,&read_num);
//         xDCSpin.setLow();

//         for (int i = 0; i < 1024; i= i+32)
//         {


//             while(!(DREQpin.getLevel()))
//             {

//             }


//             temp = spiKid.transfer(read_buff[i]);
//             temp = spiKid.transfer(read_buff[i+1]);
//             temp = spiKid.transfer(read_buff[i+2]);
//             temp = spiKid.transfer(read_buff[i+3]);
//             temp = spiKid.transfer(read_buff[i+4]);
//             temp = spiKid.transfer(read_buff[i+5]);
//             temp = spiKid.transfer(read_buff[i+6]);
//             temp = spiKid.transfer(read_buff[i+7]);
//             temp = spiKid.transfer(read_buff[i+8]);
//             temp = spiKid.transfer(read_buff[i+9]);
//             temp = spiKid.transfer(read_buff[i+10]);
//             temp = spiKid.transfer(read_buff[i+11]);
//             temp = spiKid.transfer(read_buff[i+12]);
//             temp = spiKid.transfer(read_buff[i+13]);
//             temp = spiKid.transfer(read_buff[i+14]);
//             temp = spiKid.transfer(read_buff[i+15]);
//             temp = spiKid.transfer(read_buff[i+16]);
//             temp = spiKid.transfer(read_buff[i+17]);
//             temp = spiKid.transfer(read_buff[i+18]);
//             temp = spiKid.transfer(read_buff[i+19]);
//             temp = spiKid.transfer(read_buff[i+20]);
//             temp = spiKid.transfer(read_buff[i+21]);
//             temp = spiKid.transfer(read_buff[i+22]);
//             temp = spiKid.transfer(read_buff[i+23]);
//             temp = spiKid.transfer(read_buff[i+24]);
//             temp = spiKid.transfer(read_buff[i+25]);
//             temp = spiKid.transfer(read_buff[i+26]);
//             temp = spiKid.transfer(read_buff[i+27]);
//             temp = spiKid.transfer(read_buff[i+28]);
//             temp = spiKid.transfer(read_buff[i+29]);
//             temp = spiKid.transfer(read_buff[i+30]);
//             temp = spiKid.transfer(read_buff[i+31]);



//         }

//     }
//     xDCSpin.setHigh();
// }


int main (void)
{
    pause_bool = 0;
    stop_bool = 0;
    FATFS fs;
    FRESULT res;
    res = f_mount(&fs, "", 1);
    read_queue = xQueueCreate(2, sizeof(uint8_t));

    xCSpin.setAsOutput();
    xDCSpin.setAsOutput();
    DREQpin.setAsInput();
    RSTpin.setAsOutput();

    if(spiKid.initialize(8, fm0_0, 2)){
        printf("SPI Properly initialized.\n");
    }
    else{
        printf("SPI Not properly initialized\n");
    } // initializing 8 bit transfer, CPOL = 0, CPHA = 0, and dividing by 64

    initializeDecoderTest(); // setting SCI_MODE register to allow for SCI tests and setting appropriate volume

    gpio_interrupt.Initialize();
    knob_control_sem = xSemaphoreCreateBinary();

    void (*foo)(void) = Eint3Handler;

    isr_register(EINT3_IRQn, foo);
    gpio_interrupt.AttachInterruptHandler(0, 1, knob_set_ISR, kFallingEdge);

    startSineTest();
    delay_ms(1000);
    endSineTest();
    delay_ms(1000);

    // initialize ADC driver

    readSetClkReg();
    delay_ms(10);


     if (res == FR_OK)
     {
         Load_file_info();
     }

//         for (int x = 0 ; x <= len-1; x ++)
//         {
//             printf("in loop");
//             lcd.clear_lcd();
//             lcd.write("0", "0", "Song Details");
//             lcd.write("0", "1", list[x].fname);
//             lcd.write("0", "2", list[x].sname);
//             lcd.write("0", "3", list[x].aname);
//             delay_ms(5000);
//             printf("File name: %s \n",list[x].fname);
//             printf("song name: %s \n",list[x].sname);
//             printf("artist name: %s \n",list[x].aname);
//             //printf("File size:0X%X bytes\n\n",list[x].fname);
//         }

    // printf("calling read file\n");

    // //readfile();

    // printf("done read file\n");


    ADC.AdcInitBurstMode();// DELETE THIS SOON!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! putting in the Test initialization

    //LCD INTIALIZATION
    lcd.clear_lcd(); 
    stop.setAsInput();
    play.setAsInput();
    up.setAsInput();
    down.setAsInput();

    const uint32_t STACK_SIZE_WORDS = 1024;

    lcd.write("0", "1", "Press Play To Begin!");
    delay_ms(800);
 
    xTaskCreate(vPlay_file, "Play Songs", STACK_SIZE_WORDS, (void *) false, 1, NULL);
    xTaskCreate(vBass_vol_treble, "Adjust Bass/vol/treble", STACK_SIZE_WORDS, (void *) false, 1, NULL);
    xTaskCreate(vLED_jazz, "LED Lights", STACK_SIZE_WORDS, (void *) false, 1, NULL);
    
    xTaskCreate(vTaskStop, "Stop Song", STACK_SIZE_WORDS, (void *) false, 1, NULL);
    xTaskCreate(vTaskPlayNoMenu, "Play Before Menu", STACK_SIZE_WORDS, (void *) false, 1, NULL);
    xTaskCreate(vTaskPlayAfterMenu, "Play After Select Song", STACK_SIZE_WORDS, (void *) false, 1, NULL);
    xTaskCreate(vTaskDownMenu, "Decrement Song", STACK_SIZE_WORDS, (void *) false, 1, NULL);
    xTaskCreate(vTaskUpMenu, "Increment Song", STACK_SIZE_WORDS, (void *) false, 1, NULL);
    xTaskCreate(vTaskPause, "Pause Song", STACK_SIZE_WORDS, (void *) false, 1, NULL);
    xTaskCreate(vTaskPlayFromPause, "Play Pause Song", STACK_SIZE_WORDS, (void *) false, 1, NULL);





    vTaskStartScheduler();

    return 0;



         // while(1)
         // {
         // }
}

/*
SCI Write
1. Set XCS to low
2. Send write OPCODE (1 byte)
3. Send address of register to write to. (1 byte)
4. Send data (2 bytes)
5. Set XCS to high
*/
void initializeDecoderTest(void){
    printf("Entered initialize decoder test\n");
    uint8_t bytes[12];
    uint16_t word[3];

    //hardware reset 
    RSTpin.setLow();
    delay_ms(100);
    RSTpin.setHigh();
    xDCSpin.setHigh();

    while(!(DREQpin.getLevel()));

    xCSpin.setHigh();
    //setting MODE register
    xCSpin.setLow();
    bytes[0] = spiKid.transfer(sci_read);
    bytes[1] = spiKid.transfer(mode_reg);
    //may need to change to 16 bit SPI transaction rather than 8
    //i.e uint16_t word[1];
    //word[0] = spiKid.transferWord(0x4820);
    bytes[2] = spiKid.transfer(0xFF);
    bytes[3] = spiKid.transfer(0xFF);
    xCSpin.setHigh();
    //printf("mode_reg = %01xh\n", word[0]);
    printf("mode_reg = %01xh\n", bytes[2]);
    printf("mode_reg = %01xh\n", bytes[3]);


    while(!(DREQpin.getLevel()));

    xCSpin.setLow();
    bytes[4] = spiKid.transfer(sci_write);
    bytes[5] = spiKid.transfer(mode_reg);
    //word[1] = spiKid.transferWord(0xFFFF);
    bytes[6] = spiKid.transfer(0x48);
    bytes[7] = spiKid.transfer(0x20);
    xCSpin.setHigh();
    // printf("mode_reg = %01xh\n", word[1]);
    // printf("mode_reg = %01xh\n", bytes[6]);
    // printf("mode_reg = %01xh\n", bytes[7]);

    while(!(DREQpin.getLevel()));

    xCSpin.setLow();
    bytes[8] = spiKid.transfer(sci_write);
    bytes[9] = spiKid.transfer(vol_reg);
    // word[2] = spiKid.transferWord(0x7777);
    bytes[10] = spiKid.transfer(0x22); //setting volume
    bytes[11] = spiKid.transfer(0x22);
    //bytes[7] = spiKid.transfer(0x77);
    //bytes[8] = spiKid.transfer(0x77);
    xCSpin.setHigh();

    while(!(DREQpin.getLevel()));

    xCSpin.setLow();
    bytes[12] = spiKid.transfer(sci_read);
    bytes[13] = spiKid.transfer(vol_reg);
    //word[2] = spiKid.transferWord(0xFFFF);
    bytes[14] = spiKid.transfer(0xFF);
    bytes[15] = spiKid.transfer(0xFF);
    printf("mode_reg = %01xh\n", bytes[14]);
    printf("mode_reg = %01xh\n", bytes[15]);
    xCSpin.setHigh();
    // printf("mode_reg = %01xh\n", word[2]);

    while(!(DREQpin.getLevel()));

    printf("Decoder initialized.\n");

}

void startSineTest(void){
    printf("Starting Sine Test\n");
    uint8_t bytes[8];
    /*mode reg, set SCI_TEST bit (SCI_MODE[5]) to 1*/
    /*0x20*/
    xDCSpin.setLow();
    bytes[0] = spiKid.transfer(0x53);
    // xDCSpin.setHigh();
    // xDCSpin.setLow();
    bytes[1] = spiKid.transfer(0xEF);
    // xDCSpin.setHigh();
    // xDCSpin.setLow();
    bytes[2] = spiKid.transfer(0x6E);
    // xDCSpin.setHigh();
    // xDCSpin.setLow();
    bytes[3] = spiKid.transfer(0x7E);
    // xDCSpin.setHigh();
    // xDCSpin.setLow();
    bytes[4] = spiKid.transfer(0x00);
    // xDCSpin.setHigh();
    // xDCSpin.setLow();
    bytes[5] = spiKid.transfer(0x00);
    // xDCSpin.setHigh();
    // xDCSpin.setLow();
    bytes[6] = spiKid.transfer(0x00);
    // xDCSpin.setHigh();
    // xDCSpin.setLow();
    bytes[7] = spiKid.transfer(0x00);
    xDCSpin.setHigh();
}

void endSineTest(void){

    printf("Ending Sine Test\n");
    uint8_t bytes[8];
    xDCSpin.setLow();
    bytes[0] = spiKid.transfer(0x45);
    // xDCSpin.setHigh();
    // xDCSpin.setLow();
    bytes[1] = spiKid.transfer(0x78);
    // xDCSpin.setHigh();
    // xDCSpin.setLow();
    bytes[2] = spiKid.transfer(0x69);
    // xDCSpin.setHigh();
    // xDCSpin.setLow();
    bytes[3] = spiKid.transfer(0x74);
    // xDCSpin.setHigh();
    // xDCSpin.setLow();
    bytes[4] = spiKid.transfer(0x00);
    // xDCSpin.setHigh();
    // xDCSpin.setLow();
    bytes[5] = spiKid.transfer(0x00);
    // xDCSpin.setHigh();
    // xDCSpin.setLow();
    bytes[6] = spiKid.transfer(0x00);
    // xDCSpin.setHigh();
    // xDCSpin.setLow();
    bytes[7] = spiKid.transfer(0x00);
    xDCSpin.setHigh();
}

void readSetClkReg(void){
    printf("Reading then setting the clk_reg.\n");
    uint8_t dummy_byte;
    uint8_t clock_bytes[2];
    //reading from clk_reg
    while(!(DREQpin.getLevel()));
    xCSpin.setLow();
    dummy_byte = spiKid.transfer(sci_read);
    dummy_byte = spiKid.transfer(clk_reg); //clk_reg = 0x03
    clock_bytes[0] = spiKid.transfer(0xFF); //getting bytes
    clock_bytes[1] = spiKid.transfer(0xFF);
    xCSpin.setHigh();
    printf("clock_bytes %01x %01xh\n", clock_bytes[0], clock_bytes[1]); // printing clk_reg

    //writing to clk_reg
    while(!(DREQpin.getLevel()));
    xCSpin.setLow();
    dummy_byte = spiKid.transfer(sci_write);
    dummy_byte = spiKid.transfer(clk_reg); //clk_reg = 0x03
    printf("setting multiplier\n");
    dummy_byte = spiKid.transfer(0xE0); // 0x6000 , setting clk_mult[2:0] = 3
    dummy_byte = spiKid.transfer(0x00);
    xCSpin.setHigh();

    while(!(DREQpin.getLevel()));
    xCSpin.setLow();
    dummy_byte = spiKid.transfer(sci_read);
    dummy_byte = spiKid.transfer(clk_reg); //clk_reg = 0x03
    clock_bytes[0] = spiKid.transfer(0xFF); //getting bytes
    clock_bytes[1] = spiKid.transfer(0xFF);
    xCSpin.setHigh();
    printf("clock_bytes %01x %01xh\n", clock_bytes[0], clock_bytes[1]); // printing clk_reg

}
