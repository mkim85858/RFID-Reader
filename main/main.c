/*
********************************************************************************
*                       INCLUDES
********************************************************************************
*/
/* Insert include files here */
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "Globals.h"
#include "HardwareConfig.h"
#include "ReaderApi.h"
#include "PN532Drv.h"
#include "BuzzerDrv.h"
#include "StorageApi.h"
#include "ButtonDrv.h"
/*
********************************************************************************
*                       GLOBAL(EXPORTED) VARIABLES & TABLES
********************************************************************************
*/
/* Insert global variables & tables here */
/*
********************************************************************************
*                       LOCAL DEFINITIONS & MACROS
********************************************************************************
*/
/* Insert #define here */
/*
********************************************************************************
*                       LOCAL DATA TYPES & STRUCTURES
********************************************************************************
*/
/* Insert local typedef & structure here */
/*
********************************************************************************
*                       GLOBAL(FILE SCOPE) VARIABLES & TABLES
********************************************************************************
*/
/* Insert file scope variable & tables here */
static BOOLEAN saveTag = false;
static INT64U pressedTime = 0;
static SemaphoreHandle_t semaphoreHandle = NULL;
/*
********************************************************************************
*                       LOCAL FUNCTION PROTOTYPES
********************************************************************************
*/
/* Insert static function prototypes here */
static void pollingTask(void* arg);
static void buttonTask(void* arg);
static void buttonInterrupt(void* arg);
/*
********************************************************************************
*                       GLOBAL(EXPORTED) FUNCTIONS
********************************************************************************
*/
/* Insert global functions here */
/*
********************************************************************************
*                       LOCAL FUNCTIONS
********************************************************************************
*/
/* Insert local functions here */

void app_main(void) {
    Reader_Init();
    //Buzzer_Init();
    //Storage_Init();
    //Button_Init(buttonInterrupt);
    //semaphoreHandle = xSemaphoreCreateBinary();

    //xTaskCreate(pollingTask, "polling tag", 4096, NULL, 2, NULL);
    //xTaskCreate(buttonTask, "button pressed", 2048, NULL, 3, NULL);
}

// Task that continuously polls for a tag
void pollingTask(void *arg) {
    while(1) {
        if (stopPolling == false) {
            if (saveTag == false) {
                Reader_PollTag();
            }
            else {
                Reader_SaveTag();
                saveTag = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task that handles button presses
void buttonTask(void* arg) {
    while (1) {
        if (xSemaphoreTake(semaphoreHandle, portMAX_DELAY) == pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(5));
            // start counting time on rising edge
            if (gpio_get_level(BUTTON_PIN) == 1) {
                stopPolling = true;
                pressedTime = esp_timer_get_time();
            }
            // calculate how long button was pressed on falling edge
            else {
                pressedTime = esp_timer_get_time() - pressedTime;
                // if button was held for over 2 seconds, clear memory
                if (pressedTime > 2000000) {
                    Storage_Clear();
                }
                // if button was not held, save next tag
                else {
                    saveTag = true;
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                stopPolling = false;
            }
        }
    }
}

// Interrupt for when the button is pressed/released
void buttonInterrupt(void *arg) {
    xSemaphoreGiveFromISR(semaphoreHandle, NULL);
}