/*
********************************************************************************
*                       INCLUDES
********************************************************************************
*/
/* Insert include files here */
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

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
TaskHandle_t pollTaskHandle = NULL;
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
    xTaskCreate(pollingTask, "polling tag", 4096, NULL, 5, &pollTaskHandle);
}

// Task that continuously polls for a tag
void pollingTask(void *arg) {
    while(1) {
        stopPolling = false;
        ReaderPollTag();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Interrupt for when the button is pressed
void buttonInterrupt(void *arg) {
    stopPolling = true;
    xTaskNotifyFromISR(pollTaskHandle, 0, eNoAction, NULL);
}