/**
********************************************************************************
*                       INCLUDES
********************************************************************************
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ScannerApi.h"
#include "Common/Globals.h"
#include "HardwareConfig.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
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
#define PN532_PREAMBLE          0x00
#define PN532_STARTCODE1        0x00
#define PN532_STARTCODE2        0xFF
#define PN532_POSTAMBLE         0x00
#define PN532_HOST_TO_PN532     0xD4
#define PN532_ACK_FRAME         {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}
/*
********************************************************************************
*                       LOCAL DATA TYPES & STRUCTURES
********************************************************************************
*/
/* Insert local typedef & structure here */
spi_device_handle_t spi;
esp_err_t ret;
char* tag = "PN532";
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
void PN532_Init(void);
void PN532_write(INT8U *data, INT8U size);
void PN532_read(INT8U* data, INT8U size);
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
/**
********************************************************************************
* @brief    PN532 Init
* @param    none
* @return   none
* @remark   Used for initializing PN532 at startup
********************************************************************************
*/
void PN532_Init(void) {
    spi_bus_config_t buscfg = {                 // Config for the SPI bus
        .miso_io_num = SCANNER_MISO_PIN,
        .mosi_io_num = SCANNER_MOSI_PIN,
        .sclk_io_num = SCANNER_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {    // Config for the SPI device
        .clock_speed_hz = 1 * 1000 * 1000,      // 1MHz (for now)
        .mode = 0,
        .spics_io_num = SCANNER_SS_PIN,
        .queue_size = 1,
    };

    ret = spi_bus_initialize(SCANNER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(SCANNER_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

/**
********************************************************************************
* @brief    PN532 Write
* @param    data = data being sent
            size = size of data
* @return   none
* @remark   Used for sending data to PN532
********************************************************************************
*/
void PN532_write(INT8U *data, INT8U size) {
    INT8U buffer[64];
    INT8U totalSize = size + 8;
    if (totalSize > sizeof(buffer)) {              // If data is too big for the buffer
        ESP_LOGE(tag, "Data too big!");
    }

    // Start of frame for PN532
    buffer[0] = PN532_PREAMBLE;                    // Preamble
    buffer[1] = PN532_STARTCODE1;                  // Startcode
    buffer[2] = PN532_STARTCODE2;
    buffer[3] = (size + 1);                        // Length
    buffer[4] = ~buffer[3] + 1;                   // Length Checksum
    buffer[5] = PN532_HOST_TO_PN532;               // Data (including direction byte)
    memcpy(&buffer[6], data, size);
    INT8U dataChecksum = PN532_HOST_TO_PN532;     // Data checksum
    for (INT8U i = 0; i < size; i++) {
        dataChecksum += buffer[i];
    }
    buffer[6 + size] = ~dataChecksum + 1;
    buffer[7 + size] = PN532_POSTAMBLE;            // Postsum
    // End of frame

    spi_transaction_t transaction = {               // Config for transaction
        .length = totalSize * 8,
        .tx_buffer = buffer,
        .rx_buffer = NULL,
    };

    ret = spi_device_transmit(spi, &transaction);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Write failed!");
    }
    else {
        ESP_LOGV(tag, "Write successful!");
    }
}

/**
********************************************************************************
* @brief    PN532 Read
* @param    data = data received
            size = size of data
* @return   none
* @remark   Used for reading data from PN532
********************************************************************************
*/
void PN532_read(INT8U* data, INT8U size) {
    INT8U buffer[64];
    memset(buffer, 0, sizeof(buffer));

    spi_transaction_t transaction = {
        .length = sizeof(buffer) * 8,
        .tx_buffer = NULL,
        .rx_buffer = buffer,
    };

    esp_err_t ret = spi_device_transmit(spi, &transaction);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Read failed!");
    }


    if (buffer[0] != PN532_PREAMBLE ||            // Verifying preamble/start code
        buffer[1] != PN532_STARTCODE1 || 
        buffer[2] != PN532_STARTCODE2) {
        ESP_LOGE(tag, "Invalid start code!");
    }


    INT8U length = buffer[3];                     // Retrieving and verifying length
    INT8U lengthChecksum = buffer[4];
    if ((length + lengthChecksum) != 0x00) {
        ESP_LOGE(tag, "Invalid length checksum!");
    }


    INT8U dataChecksum = 0;                         // Retrieving and verifying data
    for (INT8U i = 0; i < length; i++) {
        dataChecksum += buffer[5 + i];
    }
    INT8U calculatedChecksum = ~dataChecksum + 1;
    if (calculatedChecksum != buffer[5 + length]) {
        ESP_LOGE(tag, "Invalid data checksum!");
    }


    INT8U dataSize = length - 1;
    if (dataSize > size) {                          // If data is too big for buffer
        ESP_LOGE(tag, "Data too big!");
    }
    memcpy(data, &buffer[6], dataSize);
}