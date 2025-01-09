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
static esp_err_t ret;
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
static esp_err_t PN532_sendFrame(INT8U *frame, INT8U frame_length);
static esp_err_t PN532_receiveFrame(INT8U *buf, INT8U buf_length, INT8U *rsp_length);
static esp_err_t PN532_validateFrame(INT8U *buf, INT8U length);
static esp_err_t PN532_waitForReady(void);
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
* @return   error message during init
* @remark   Used for initializing PN532 at startup
********************************************************************************
*/
esp_err_t PN532_Init(void) {

    // Config for the SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = SCANNER_MISO_PIN,
        .mosi_io_num = SCANNER_MOSI_PIN,
        .sclk_io_num = SCANNER_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ret = spi_bus_initialize(SCANNER_HOST, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialization failed!");
        return ret;
    }

    // Config for adding PN532
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = (1 * 1000 * 1000),
        .mode = 0,
        .spics_io_num = SCANNER_SS_PIN,
        .queue_size = 1,
    };
    ret = spi_bus_add_device(SCANNER_HOST, &devcfg, &pn532);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PN532 add failed!");
        return ret;
    }

    // Config for the IRQ pin
    gpio_config_t iocfg = {
        .pin_bit_mask = (1ULL << SCANNER_IRQ_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&iocfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PN532 IRQ pin add failed!");
        return ret;
    }

    // Resetting hardware
    gpio_set_level(SCANNER_RSTO_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(SCANNER_RSTO_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "PN532 initialized!");
    return ESP_OK;
}

/**
********************************************************************************
* @brief    PN532 Write Command
* @param    cmd = command being sent
            cmd_length = length of command
* @return   error message during write
* @remark   Used for sending command to PN532
********************************************************************************
*/
esp_err_t PN532_WriteCommand(INT8U *cmd, INT8U cmd_length) {
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

    return PN532_sendFrame(frame, cmd_length + 9);
}

/**
********************************************************************************
* @brief    PN532 Read Response
* @param    buf = buffer to contain response
            buf_length = length of buffer
            rsp_length = length of response
* @return   error message during read
* @remark   Used for reading response from PN532
********************************************************************************
*/
esp_err_t PN532_ReadResponse(INT8U *buf, INT8U buf_length, INT8U *rsp_length) {
    ret = PN532_waitForReady();
    if (ret != ESP_OK) return ret;

    ret = PN532_receiveFrame(buf, buf_length, rsp_length);
    if (ret != ESP_OK) return ret;

    return PN532_validateFrame(buf, *rsp_length);
}

/*
********************************************************************************
*                       LOCAL FUNCTIONS
********************************************************************************
*/
/* Insert local functions here */
static esp_err_t PN532_sendFrame(INT8U *frame, INT8U frame_length) {
    spi_transaction_t transaction = {               // Transaction sent to PN532 via SPI
        .length = frame_length * 8,
        .tx_buffer = frame,
        .rx_buffer = NULL,
    };
    ret = spi_device_transmit(pn532, &transaction);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sending frame failed!");
        return ret;
    }

    return ESP_OK;
}

static esp_err_t PN532_receiveFrame(INT8U *buf, INT8U buf_length, INT8U *rsp_length) {
    memset(buf, 0, buf_length);

    spi_transaction_t transaction = {               // Transaction sent to ESP32 via SPI
        .length = buf_length * 8,
        .rx_buffer = buf,
        .tx_buffer = NULL,
    };
    ret = spi_device_transmit(pn532, &transaction);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Receiving frame failed!");
        return ret;
    }

    *rsp_length = transaction.rxlength / 8;
    if (*rsp_length == 0) {
        ESP_LOGE(TAG, "No data received!");
    }

    return ESP_OK;
}

static esp_err_t PN532_validateFrame(INT8U *buf, INT8U length) {
    // Validating frame start
    if (buf[0] != PN532_PREAMBLE || buf[1] != PN532_STARTCODE1 || buf[2] != PN532_STARTCODE2) {
        ESP_LOGE(TAG, "Invalid frame start!");
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Validating length checksum
    if ((INT8U)(buf[3] + buf[4]) != 0) {
        ESP_LOGE(TAG, "Invalid length checksum!");
        return ESP_ERR_INVALID_CRC;
    }

    // Validating data checksum
    INT8U dataChecksum = 0;
    for (INT8U i = 0; i < buf[3]; i++) {
        dataChecksum += buf[5 + i];
    }
    if ((INT8U)(dataChecksum + buf[5 + buf[3]]) != 0) {
        ESP_LOGE(TAG, "Invalid data checksum!");
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

static esp_err_t PN532_waitForReady(void) {
    INT32U elapsed_ms = 0;

    while (gpio_get_level(SCANNER_IRQ_PIN) == 1) {  // Wait until IRQ pin is low
        if (elapsed_ms >= 1000) {                   // If timeout
            ESP_LOGE(TAG, "Timeout waiting for IRQ pin!");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed_ms += 10;
    }
    return ESP_OK;
}