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
#include "PN532Drv.h"
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
#define PN532_DW                0x01
#define PN532_SR                0x02
#define PN532_DR                0x03

#define PN532_PREAMBLE          0x00
#define PN532_STARTCODE1        0x00
#define PN532_STARTCODE2        0xFF
#define PN532_TFI               0xD4
#define PN532_POSTAMBLE         0x00

#define TAG                     "pn532"
/*
********************************************************************************
*                       LOCAL DATA TYPES & STRUCTURES
********************************************************************************
*/
/* Insert local typedef & structure here */
static spi_device_handle_t pn532;
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
static void PN532_sendFrame(INT8U *frame, INT8U frame_length);
static void PN532_receiveFrame(INT8U *buf, INT8U buf_length, INT8U *rsp_length);
static void PN532_waitForIRQ(void);
static void PN532_waitForRDY(void);

/*
********************************************************************************
*                       GLOBAL(EXPORTED) FUNCTIONS
********************************************************************************
*/
/* Insert global functions here */
/**
********************************************************************************
* @brief    PN532 Init
* @param    none
* @return   none
* @remark   Used for initializing PN532 at startup
********************************************************************************
*/
void PN532_Init(void) {
    // Initializing the SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = SCANNER_MISO_PIN,
        .mosi_io_num = SCANNER_MOSI_PIN,
        .sclk_io_num = SCANNER_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_bus_initialize(SCANNER_HOST, &buscfg, SPI_DMA_CH_AUTO);

    // Adding PN532 to the SPI bus
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = (1 * 1000 * 1000),            // 1MHz
        .mode = 0,                                      // SPI mode 0
        .spics_io_num = SCANNER_SS_PIN,
        .queue_size = 1,
        .flags = SPI_DEVICE_BIT_LSBFIRST | SPI_DEVICE_HALFDUPLEX,
        .cs_ena_pretrans = (2 * 1000),                  // 2ms delay before transaction
    };
    spi_bus_add_device(SCANNER_HOST, &devcfg, &pn532);
    ESP_LOGI(TAG, "PN532 initialized!");
}

/**
********************************************************************************
* @brief    PN532 Write Command
* @param    cmd = command being sent
            cmd_length = length of command in bytes
* @return   none
* @remark   Used for sending a command to PN532
********************************************************************************
*/
void PN532_WriteCommand(INT8U *cmd, INT8U cmd_length) {
    INT8U frame[cmd_length + 9];

    // Start of frame
    frame[0] = PN532_DW;                                // Direction Byte
    frame[1] = PN532_PREAMBLE;                          // Preamble
    frame[2] = PN532_STARTCODE1;                        // Startcode
    frame[3] = PN532_STARTCODE2;
    frame[4] = (cmd_length + 1);                        // Length
    frame[5] = (INT8U)(~frame[4] + 1);                  // Length Checksum
    frame[6] = PN532_TFI;                               // Data (including direction byte)
    memcpy(&frame[7], cmd, cmd_length);
    frame[7 + cmd_length] = PN532_TFI;                  // Data Checksum
    for (INT8U i = 0; i < cmd_length; i++) {
        frame[7 + cmd_length] += frame[7 + i];
    }
    frame[7 + cmd_length] = (INT8U)(~frame[7 + cmd_length] + 1);
    frame[8 + cmd_length] = PN532_POSTAMBLE;            // Postamble
    // End of frame

    PN532_sendFrame(frame, cmd_length + 9);

    ESP_LOGI(TAG, "Command sent!");
}

/**
********************************************************************************
* @brief    PN532 Read Response
* @param    buf = buffer to hold response
            buf_length = size of buffer in bytes
            rsp_length = size of response in bytes
* @return   none
* @remark   Used for reading a response from PN532
********************************************************************************
*/
void PN532_ReadResponse(INT8U *buf, INT8U buf_length, INT8U *rsp_length) {
    // Waiting until IRQ pin pulls down
    //PN532_waitForIRQ();
    //PN532_waitForRDY();
    vTaskDelay(pdMS_TO_TICKS(10));

    // Sending DR byte
    INT8U dr = PN532_DR;
    PN532_sendFrame(&dr, 1);

    // Reading data from PN532 and saving in buffer
    PN532_receiveFrame(buf, buf_length, rsp_length);

    ESP_LOGI(TAG, "Response received!");
}

/*
********************************************************************************
*                       LOCAL FUNCTIONS
********************************************************************************
*/
/* Insert local functions here */

// Sends a frame to the scanner
static void PN532_sendFrame(INT8U *frame, INT8U frame_length) {
    spi_transaction_t transaction = {
        .length = 8 * frame_length,
        .tx_buffer = frame,
        .rx_buffer = NULL,
    };
    spi_device_transmit(pn532, &transaction);
}

// Receives a frame from the scanner
static void PN532_receiveFrame(INT8U *buf, INT8U buf_length, INT8U *rsp_length) {
    spi_transaction_t transaction = {
        .length = 0,
        .tx_buffer = NULL,
        .rx_buffer = buf,
        .rxlength = 8 * buf_length,
    };
    spi_device_transmit(pn532, &transaction);

    *rsp_length += transaction.rxlength;
}

// Waits for the scanner to lower the IRQ pin
static void PN532_waitForIRQ(void) {
    INT16U elapsed_time = 0;

    while (gpio_get_level(SCANNER_IRQ_PIN) == 0x01) {
        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed_time += 10;

        if (elapsed_time > 1000) {
            ESP_LOGE(TAG, "Timeout waiting for IRQ pin!");
            break;
        }
    }
}

// Waits for the scanner to send a RDY byte
static void PN532_waitForRDY(void) {
    INT16U elapsed_time = 0;
    INT8U SR = PN532_SR;
    INT8U RDY = 0x00;
    INT8U RDYlen;

    do {
        PN532_sendFrame(&SR, 1);
        PN532_receiveFrame(&RDY, 1, &RDYlen);

        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed_time += 10;
        if (elapsed_time > 1000) {
            ESP_LOGE(TAG, "Timeout waiting for RDY byte!");
            break;
        }
    }
    while (RDY == 0x00);
}