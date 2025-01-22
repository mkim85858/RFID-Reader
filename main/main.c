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

    // Sending GetFirmwareVersion command
    INT8U cmd[] = {0x02};
    PN532_WriteCommand(cmd, 1);

    // Receiving ACK frame
    INT8U ack[6];
    memset(ack, 0, 6);
    PN532_ReadResponse(ack, 6);

    // Receiving response frame
    INT8U rsp[11];
    memset(rsp, 0, 11);
    PN532_ReadResponse(rsp, 11);

    ESP_LOGI(TAG, "IC: %02x", rsp[6]);
    ESP_LOGI(TAG, "Ver: %02x", rsp[7]);
    ESP_LOGI(TAG, "Rev: %02x", rsp[8]);
    ESP_LOGI(TAG, "Support: %02x", rsp[9]);
}