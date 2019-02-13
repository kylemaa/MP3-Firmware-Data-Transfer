#include <cstring>
#include "UART_DR.hpp"
#include "stdio.h"
#include "io.hpp"
#include <string.h>
#include <cstring>
#include "FreeRTOS.h"
#include "task.h"

// #include "LABGPIO.hpp"
#include "LabGPIO_1.hpp"
#include "LCD_DRIVER.hpp"

bool stopped = true;
bool paused = false;
string list[] = {"Song 1", "Song 2", "Song 3", "Song 4", "Song 5", "Song 6", "Song 7", "Song 8", "Song 9", "Song 10"}; 
int current = 0;
int song_index = 0;
bool song_menu = false;

LabGPIO_1 stop(9);
LabGPIO_1 play(10);
LabGPIO_1 up(14);
LabGPIO_1 down(15);

LCD_D lcd;


void vTaskPause(void * PvParameters)
{
    while(1){
        if(up.getLevel()){
            lcd.clear_lcd();    
            lcd.write("0", "0", "Paused!");
            delay_ms(500);
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
            lcd.write("0", "0", "Playing Song!");
            lcd.write("0", "1", list[current]);
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
                lcd.write("0", "0", "Playing Song!");
                lcd.write("0", "1", list[current]);
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
        lcd.write("0", "1", list[song_index]);
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
            lcd.write("0", "1", list[song_index]);
            delay_ms(100);
            LE.off(4);
        }

        vTaskDelay(1);
    }
}

int main(){

    lcd.clear_lcd(); 

    stop.setAsInput();
    play.setAsInput();
    up.setAsInput();
    down.setAsInput();

    const uint32_t STACK_SIZE_WORDS = 512;

    lcd.write("0", "1", "Press Play To Begin!");
    delay_ms(800);
 
    xTaskCreate(vTaskStop, "Stop Song", STACK_SIZE_WORDS, (void *) false, 1, NULL);
    xTaskCreate(vTaskPlayNoMenu, "Play Before Menu", STACK_SIZE_WORDS, (void *) false, 1, NULL);
    xTaskCreate(vTaskPlayAfterMenu, "Play After Select Song", STACK_SIZE_WORDS, (void *) false, 1, NULL);
    xTaskCreate(vTaskDownMenu, "Decrement Song", STACK_SIZE_WORDS, (void *) false, 1, NULL);
    xTaskCreate(vTaskUpMenu, "Increment Song", STACK_SIZE_WORDS, (void *) false, 1, NULL);



    vTaskStartScheduler();

    return 0;
} 