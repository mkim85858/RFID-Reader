/*
********************************************************************************
*                       INCLUDES
********************************************************************************
*/
/* Insert include files here */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "Globals.h"
#include "HardwareConfig.h"
#include "Scanner/ScannerApi.h"
#include "Scanner/PN532Drv.h"
#include "Temp/WithFramework.h"
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
#define TAG                 "main"
/*
********************************************************************************
*                       GLOBAL(FILE SCOPE) VARIABLES & TABLES
********************************************************************************
*/
/* Insert file scope variable & tables here */
/*
********************************************************************************
*                       LOCAL FUNCTION PROTOTYPES
********************************************************************************
*/
/* Insert static function prototypes here */
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
    PN532_Init();

    INT8U command[] = {0x14, 0x01, 0x00, 0x01};
    PN532_WriteCommand(command, 4);


    INT8U ack[6];
    memset(ack, 0, 6);
    INT8U rsplen = 0;
    PN532_ReadResponse(ack, 6, &rsplen);

    // for (int i = 0; i < 6; i ++) {
    //     ESP_LOGI(TAG, "0x%02x", ack[i]);
    // }
}