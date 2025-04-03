/**
********************************************************************************
*                       INCLUDES
********************************************************************************
*/
#include <string.h>

#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "driver/spi_master.h"

#include "Globals.h"
#include "HardwareConfig.h"
#include "PN532Drv.h"
/*
********************************************************************************
*                       GLOBAL(EXPORTED) VARIABLES & TABLES
********************************************************************************
*/
/* Insert global variables & tables here */
BOOLEAN stopPolling = false;
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

#define ACK                     {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}
#define NACK                    {0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00}
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
static spi_device_handle_t spi_handle;
/*
********************************************************************************
*                       LOCAL FUNCTION PROTOTYPES
********************************************************************************
*/
static void SPI_waitForRDY(void);
static void SPI_write(INT8U *data, INT8U data_length);
static void SPI_writeRead(INT8U *data, INT8U data_length, INT8U *rsp, INT8U rsp_length);

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
    spi_bus_config_t spicfg = {
        .miso_io_num = READER_MISO_PIN,
        .mosi_io_num = READER_MOSI_PIN,
        .sclk_io_num = READER_CLK_PIN,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .max_transfer_sz = 4096,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = SPI_DEVICE_BIT_LSBFIRST | SPI_DEVICE_HALFDUPLEX,
    };

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << READER_SS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    spi_bus_initialize(SPI2_HOST, &spicfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);
    gpio_config(&io_conf);

    gpio_set_level(READER_SS_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
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
    frame[6] = PN532_TFI;                               // Data (including TFI)
    memcpy(&frame[7], cmd, cmd_length);
    frame[7 + cmd_length] = PN532_TFI;                  // Data Checksum
    for (INT8U i = 0; i < cmd_length; i++) {
        frame[7 + cmd_length] += frame[7 + i];
    }
    frame[7 + cmd_length] = (INT8U)(~frame[7 + cmd_length] + 1);
    frame[8 + cmd_length] = PN532_POSTAMBLE;            // Postamble
    // End of frame

    // Sending frame via SPI
    SPI_write(frame, cmd_length + 9);
}

/**
********************************************************************************
* @brief    PN532 Write NACK
* @param    none
* @return   none
* @remark   Used for sending a NACK frame to PN532
********************************************************************************
*/
void PN532_WriteNACK(void) {
    INT8U nack[] = NACK;
    SPI_write(nack, 6);
}

/**
********************************************************************************
* @brief    PN532 Read Response
* @param    buf = buffer to hold response
            buf_length = length of buffer in bytes
            rsp_length = length of response in bytes
* @return   none
* @remark   Used for reading a response from PN532
********************************************************************************
*/
void PN532_ReadResponse(INT8U *buf, INT8U buf_length) {
    // Waiting until RDY bit is 0x01
    SPI_waitForRDY();

    // Receiving data via SPI
    INT8U DR = PN532_DR;
    SPI_writeRead(&DR, 1, buf, buf_length);
}

/*
********************************************************************************
*                       LOCAL FUNCTIONS
********************************************************************************
*/
/* Insert local functions here */

// Waits for the reader to send a RDY byte
static void SPI_waitForRDY(void) {
    INT8U SR = PN532_SR;
    INT8U RDY = 0x00;
    while(1) {
        SPI_writeRead(&SR, 1, &RDY, 1);
        if (RDY == 0x01 || stopPolling) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Writes data to PN532
static void SPI_write(INT8U *data, INT8U data_length) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = data_length * 8;
    t.tx_buffer = data;

    gpio_set_level(READER_SS_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    spi_device_transmit(spi_handle, &t);
    gpio_set_level(READER_SS_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
}

// Writes data to PN532 and receives response
static void SPI_writeRead(INT8U *data, INT8U data_length, INT8U *rsp, INT8U rsp_length) {
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));
    t.length = data_length * 8;
    t.tx_buffer = data;
    gpio_set_level(READER_SS_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    spi_device_transmit(spi_handle, &t);
    gpio_set_level(READER_SS_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    memset(&t, 0, sizeof(t));
    t.rxlength = rsp_length * 8;
    t.rx_buffer = rsp;
    gpio_set_level(READER_SS_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    spi_device_transmit(spi_handle, &t);
    gpio_set_level(READER_SS_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
}