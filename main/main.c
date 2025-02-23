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
/*
********************************************************************************
*                       LOCAL FUNCTION PROTOTYPES
********************************************************************************
*/
/* Insert static function prototypes here */
static void pollingTask(void* arg);
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
    Buzzer_Init();
    Storage_Init();
    Button_Init(buttonInterrupt);
    xTaskCreate(pollingTask, "polling tag", 1024, NULL, 5, NULL);
}

// Task that continuously polls for a tag
void pollingTask(void *arg) {
    while(1) {
        while(stopPolling)
        // Poll for next tag
        if (saveTag == 0) {
            Reader_PollTag();
        }
        // Save next tag
        else {
            Reader_SaveTag();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Interrupt for when the button is pressed/released
void buttonInterrupt(void *arg) {
    stopPolling = true;
    BOOLEAN level = gpio_get_level(BUTTON_PIN);
    // start counting time on rising edge
    if (level == 1) {
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
        }
        stopPolling = false;
    }
}