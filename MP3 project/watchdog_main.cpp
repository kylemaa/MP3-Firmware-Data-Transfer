#include "LPC17xx.h"
#include "FreeRTOS.h"
#include "soft_timer.hpp"
#include "storage.hpp"
#include "rtc.h"
#include "queue.h"
#include <stdio.h>
#include "utilities.h"
#include "io.hpp"
#include "stdio.h"
#include "string.h"
#include "event_groups.h"
#include "handler_header.h"

#include "tasks.hpp"
#include "examples.hpp"
#include "periodic_callback.h"
#include "uart2.hpp"
#include "uart3.hpp"
#include "handlers.hpp"


EventGroupHandle_t xEventGroup;
QueueHandle_t sensorQueue;

void vProducerTask(void * pvParams);
void vConsumerTask(void * pvParams);
void vWatchdogTask(void * pvParams);


int main (void)
{
    scheduler_add_task(new terminalTask(PRIORITY_HIGH)); // adding terminal task to your program
    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL)); 

    sensorQueue = xQueueCreate(10, sizeof(float)); // creating sensor queue
    xEventGroup = xEventGroupCreate(); // creating event group 

    BaseType_t xProducerTask;
    BaseType_t xConsumerTask;
    BaseType_t xWatchdogTask;

    const uint32_t STACK_SIZE = 2048;

    //creating our various tasks
    xProducerTask = xTaskCreate(vProducerTask, "Producer Task", STACK_SIZE, (void*) 1, 1, &xProducerHandle);
    xConsumerTask = xTaskCreate(vConsumerTask, "Consumer Task", STACK_SIZE, (void*) 1, 1, &xConsumerHandle);
    xWatchdogTask = xTaskCreate(vWatchdogTask, "Watchdog Task", STACK_SIZE, (void*) 1, 2, &xWatchdogHandle);

    scheduler_start();
    vTaskStartScheduler();


    return 0;
}

/*
- Take 1 light sensor value every 1ms.
- Compute average after collecting 100 samples. 
- Write average to sensor queue. 
- Set a bit using xEventGroupSetBits() at the end of each loop. 

LS.getRawValue(); //returns raw light sensor value.
*/
void vProducerTask(void * pvParams){
    float average;
    EventBits_t uxBit0;
    SoftTimer myTimer(1);
    int ms;
    /* every ms, take LS value*/
    while(1){
        ms = 0;
        while(ms != 100){
            if(myTimer.expired()){
                average = average + LS.getRawValue();
                ms++;
                myTimer.restart();
            }
        }
        ms = 0;
        average = average / 100; // calculate average of LS
        //printf("Producer = %f\n", average);
        uxBit0 = xEventGroupSetBits(xEventGroup, (1<<0)); // producer task should set bit 0 
        xQueueSend(sensorQueue, (void*) &average, portMAX_DELAY); // write average to queue. 
        }
}

/*
- Pull data off of sensor queue.
- Use infinite timeout value (portMAX_DELAY)
- Open a file (sensor.txt) and append the data to an output file on the SD card
- Save data using "printf("%i, %i\m", time, light)"
- Use better method than opening/closing a file every 100ms
- Use medium priority for this task. 
- Set a bit using xEventGroupSetBits() at the end of each loop. 
*/
void vConsumerTask(void * pvParams){
    float dataReceived = 0;
    int dataReady = 0;
    char buffer[256];
    int tmp;
    EventBits_t uxBit1;
    while(1){
        if(dataReady == 10){
            for(int i = 0; i<10; i++){
                if(xQueueReceive(sensorQueue, &dataReceived, portMAX_DELAY)){ // if we were able to receive
                    tmp = snprintf(buffer, sizeof(buffer), "%f", dataReceived); // turn float to char and add to buffer
                    strcat(buffer, ", ");
                    strcat(buffer, rtc_get_date_time_str()); // get date and time 
                    strcat(buffer, "\n");
                    Storage::append("1:sensor.txt", buffer, 39, 0); // append to sensor.txt
                }
                else{
                    //printf("Consumer: nothing to receive\n");
                }

            }
            dataReady = 0;
            uxBit1 = xEventGroupSetBits(xEventGroup, (1<<1)); // set uxbit1
        }
        else{
        dataReady++;
        uxBit1 = xEventGroupSetBits(xEventGroup, (1<<1)); // set uxbit1
        }
    }
}

/*
- Use high priority.
- Use timeout of 1 second, and wait for all the bits to set. 
- If bits are not set, append file with info with task may be stuck.
- Open file -> append data -> close file. Flush out the data. 
*/
void vWatchdogTask(void * pvParams){
    EventBits_t wdBits;
    const TickType_t xTicksToWait = pdMS_TO_TICKS(1000); // ticks for our wait bits to actually go off 
    char watchddogBuffer[1024]; 
    //char buffer[256];
    SoftTimer cpuTimer(60000); // timer for cpu usage
    int tmp;

    while(1){
        //Every 60 seconds, write CPU usage to cpu.txt 
        if(cpuTimer.expired()){
            printf("cpuTimer expired\n");
            const unsigned portBASE_TYPE maxTasks = 16;
            TaskStatus_t status[maxTasks]; // array of task's statuses 
            uint32_t totalRunTime = 0;
            uint32_t tasksRunTime = 0;
            const unsigned portBASE_TYPE uxArraySize = uxTaskGetSystemState(&status[0], maxTasks, &totalRunTime); // get array size of tasks 
            for(unsigned priorityNum = 0; priorityNum < configMAX_PRIORITIES; priorityNum++){
                for(unsigned i = 0; i<uxArraySize; i++){
                    TaskStatus_t *e = &status[i];
                    if(e->uxBasePriority == priorityNum){
                        tasksRunTime+=e->ulRunTimeCounter;
                        const uint32_t cpuPercent = (0 == totalRunTime) ? 0: e->ulRunTimeCounter/(totalRunTime/100);
                        //printf("%s, %lu %% \n", e->pcTaskName, cpuPercent);
                        strncat(watchddogBuffer, "\n", 3);
                        strncat(watchddogBuffer, e->pcTaskName, 10);
                        strncat(watchddogBuffer, ", ", 2);
                        tmp = snprintf(watchddogBuffer, 5, "%lu", cpuPercent);
                        strncat(watchddogBuffer, "%%\n",5);
                        Storage::append("1:cpu.txt", watchddogBuffer, 20, 0); // append cpu usage to cpu.txt
                    }
                }
            }
            //printf("%s\n", watchddogBuffer);
            cpuTimer.restart();
            printf("Timer restarted\n");
        }
        wdBits = xEventGroupWaitBits(xEventGroup, // name of event group
                                    (1<<0)|(1<<1), // checking bits 1 and 0
                                    pdTRUE, // if set to pdTRUE, clears bits after exit
                                    pdTRUE, // func will return when either of the bits are set in the 
                                    xTicksToWait);
        if( (wdBits & ((1<<0) | (1<<1)) ) == ( ((1<<0)| (1<<1)) ) ) {
            //bits are set so tasks are running normally
            printf("Tasks running normally\n");
        }
        else{
            if((wdBits & (1<<0)) !=0){
            //producer task finished but consumer task didn't finish
            //1. open file
            //2. append to stuck.txt that consumer task didn't finish
            //3. close file
            //repeat for next conditions
            Storage::append("1:stuck.txt", "Consumer task did not finish\n", 30, 0);
            printf("Consumer task did not finish\n");
            }
            else if((wdBits & (1<<1)) !=0){
                //consummer task finished but producer task didn't finish
                Storage::append("1:stuck.txt", "Producer task did not finish\n", 30, 0);
                printf("Producer task did not finish\n");
            }
            else{
                //neither task finished
                // strcat(buffer, "At ");
                // strcat(buffer, rtc_get_date_time_str());
                Storage::append("1:stuck.txt", "Neither task finished\n", 30, 0);
                printf("Neither task finished\n");
            }
        }
    }
}
