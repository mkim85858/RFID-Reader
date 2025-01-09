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
#include "CpuDrv.h"
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
    INT8U command[] = {0x02};
    INT8U response[20];
    INT8U size;

    //DR, PREAMBLE, STARTCODE1, STARTCODE2, LENGTH, LCS, [TFI, 03, IC, Ver, Rec, Support], DCS, POSTAMBLE

    // Technically, this could be a hardware issue, from the jumper wires, breadboard, PN532, or ESP32. Any one of these has a (small but existant) chance to not work, which just fucks up everything. The only way I could solve that problem would be to get a multimeter or something.

    PN532_Init();

    PN532_WriteCommand(command, 1);
    ESP_LOGI(TAG, "Writing command successful");

    PN532_ReadResponse(response, 20, &size);
    ESP_LOGI(TAG, "%x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x", 
        response[0], response[1], response[2], response[3], response[4], response[5], response[6], 
        response[7], response[8], response[9], response[10], response[11], response[12], response[13]);
        
    ESP_LOGI(TAG, "Test completed.");
}