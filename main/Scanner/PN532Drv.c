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
#define SPI_SR                  0x02
#define SPI_DW                  0x01
#define SPI_DR                  0x03

#define PN532_PREAMBLE          0x00
#define PN532_STARTCODE1        0x00
#define PN532_STARTCODE2        0xFF
#define PN532_POSTAMBLE         0x00
#define PN532_HOST_TO_PN532     0xD4
#define PN532_ACK_FRAME         {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}
#define PN532_NACK_FRAME        {0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00}

#define TAG                     "PN532"
/*
********************************************************************************
*                       LOCAL DATA TYPES & STRUCTURES
********************************************************************************
*/
/* Insert local typedef & structure here */
spi_device_handle_t spi;
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

    // Config for the SPI device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = (1 * 1000 * 1000),
        .mode = 1,
        .spics_io_num = SCANNER_SS_PIN,
        .queue_size = 1,
    };

    // Config for the RDY pin
    gpio_config_t iocfg = {
        .pin_bit_mask = (1ULL << SCANNER_RDY_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SCANNER_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SCANNER_HOST, &devcfg, &spi));
    ESP_ERROR_CHECK(gpio_config(&iocfg));

    ESP_LOGI(TAG, "PN532 SPI Initialized");
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
esp_err_t PN532_writeCommand(INT8U *cmd, INT8U cmd_length) {
    INT8U frame[cmd_length + 8];

    // Start of frame
    frame[0] = SPI_DW;                                  // Direction Byte
    frame[1] = PN532_PREAMBLE;                          // Preamble
    frame[2] = PN532_STARTCODE1;                        // Startcode
    frame[3] = PN532_STARTCODE2;
    frame[4] = (cmd_length + 1);                        // Length
    frame[5] = (INT8U)(~frame[4] + 1);                  // Length Checksum
    frame[6] = PN532_HOST_TO_PN532;                     // Data (including direction byte)
    memcpy(&frame[7], cmd, cmd_length);
    frame[7 + cmd_length] = 0;                          // Data Checksum
    for (INT8U i = 0; i < cmd_length + 1; i++) {
        frame[7 + cmd_length] += frame[6 + i];
    }
    frame[7 + cmd_length] = (INT8U)(~frame[7 + cmd_length] + 1);
    frame[8 + cmd_length] = PN532_POSTAMBLE;            // Postsum
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
esp_err_t PN532_readResponse(INT8U *buf, INT8U buf_length, INT8U *rsp_length) {
    esp_err_t ret = PN532_waitForReady();
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

    return spi_device_transmit(spi, &transaction);
}

static esp_err_t PN532_receiveFrame(INT8U *buf, INT8U buf_length, INT8U *rsp_length) {
    spi_transaction_t transaction = {               // Transaction sent to ESP32 via SPI
        .length = buf_length * 8,
        .rx_buffer = buf,
        .tx_buffer = NULL,
    };

    esp_err_t ret = spi_device_transmit(spi, &transaction);
    if (ret == ESP_OK && rsp_length) {
        *rsp_length = transaction.rxlength / 8;
    }

    return ret;
}

static esp_err_t PN532_validateFrame(INT8U *buf, INT8U length) {
    // Validating frame start
    if (buf[0] != PN532_PREAMBLE || buf[1] != PN532_STARTCODE1 || buf[2] != PN532_STARTCODE2) {
        ESP_LOGE(TAG, "Invalid frame start");
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Validating length checksum
    if ((INT8U)(buf[3] + buf[4]) != 0) {
        ESP_LOGE(TAG, "Invalid length checksum");
        return ESP_ERR_INVALID_CRC;
    }

    // Validating data checksum
    INT8U dataChecksum = 0;
    for (INT8U i = 0; i < buf[3]; i++) {
        dataChecksum = buf[5 + i];
    }
    if ((INT8U)(dataChecksum + buf[5 + buf[3]]) != 0) {
        ESP_LOGE(TAG, "Invalid data checksum");
        return ESP_ERR_INVALID_CRC;
    }

    ESP_LOGI(TAG, "Frame validation successful");
    return ESP_OK;
}

static esp_err_t PN532_waitForReady(void) {
    INT32U elapsed_ms = 0;

    while (gpio_get_level(SCANNER_RDY_PIN) == 0) {  // Wait until RDY pin is set
        if (elapsed_ms >= 1000) {                   // If timeout
            ESP_LOGE(TAG, "Timeout waiting for RDY pin");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed_ms += 10;
    }
    return ESP_OK;
}