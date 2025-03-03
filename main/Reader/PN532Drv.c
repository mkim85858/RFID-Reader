/**
********************************************************************************
*                       INCLUDES
********************************************************************************
*/
#include <string.h>

#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"

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
/*
********************************************************************************
*                       LOCAL FUNCTION PROTOTYPES
********************************************************************************
*/
static void SPI_waitForRDY(void);
static void SPI_writeByte(INT8U byte);
static void SPI_readByte(INT8U *byte);

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
    // Configuring SS pin
    gpio_config_t sscfg = {
        .pin_bit_mask = (1ULL << READER_SS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&sscfg);
    gpio_set_level(READER_SS_PIN, 1);

    // Configuring CLK pin
    gpio_config_t clkcfg = {
        .pin_bit_mask = (1ULL << READER_CLK_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&clkcfg);
    gpio_set_level(READER_CLK_PIN, 0);

    // Configuring MISO pin
    gpio_config_t misocfg = {
        .pin_bit_mask = (1ULL << READER_MISO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&misocfg);
    
    // Configuring MOSI pin
    gpio_config_t mosicfg = {
        .pin_bit_mask = (1ULL << READER_MOSI_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&mosicfg);
    gpio_set_level(READER_MOSI_PIN, 0);
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
    gpio_set_level(READER_SS_PIN, 0);
    ets_delay_us(100);
    for (int i = 0; i < cmd_length + 9; i++) {
        SPI_writeByte(frame[i]);
    }
    gpio_set_level(READER_SS_PIN, 1);
    ets_delay_us(50);
}

/**
********************************************************************************
* @brief    PN532 write NACK
* @param    none
* @return   none
* @remark   Used for sending a NACK signal to PN532
********************************************************************************
*/
void PN532_WriteNACK(void) {
    INT8U nack[] = NACK;

    // Sending frame via SPI
    gpio_set_level(READER_SS_PIN, 0);
    ets_delay_us(50);
    for (int i = 0; i < 6; i++) {
        SPI_writeByte(nack[i]);
    }
    gpio_set_level(READER_SS_PIN, 1);
    ets_delay_us(50);
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
    gpio_set_level(READER_SS_PIN, 0);
    ets_delay_us(50);
    SPI_waitForRDY();
    gpio_set_level(READER_SS_PIN, 1);
    ets_delay_us(50);

    // Receiving data via SPI
    gpio_set_level(READER_SS_PIN, 0);
    ets_delay_us(50);
    SPI_writeByte(PN532_DR);
    for (int i = 0; i < buf_length; i++) {
        SPI_readByte(&buf[i]);
    }
    gpio_set_level(READER_SS_PIN, 1);
    ets_delay_us(50);
}

/*
********************************************************************************
*                       LOCAL FUNCTIONS
********************************************************************************
*/
/* Insert local functions here */

// Waits for the reader to send a RDY byte
static void SPI_waitForRDY(void) {
    INT8U RDY = 0x00;
    while(1) {
        SPI_writeByte(PN532_SR);
        SPI_readByte(&RDY);
        if (RDY == 0x01 || stopPolling) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Writes a byte to PN532
static void SPI_writeByte(INT8U byte) {
    for (INT8U i = 0; i < 8; i++) { 
        if (byte & (1 << i)) {
            gpio_set_level(READER_MOSI_PIN, 1);
        }
        else {
            gpio_set_level(READER_MOSI_PIN, 0);
        }
        gpio_set_level(READER_CLK_PIN, 1);
        ets_delay_us(50);
        gpio_set_level(READER_CLK_PIN, 0);
        ets_delay_us(50);
    }
}

 // Reads a byte from PN532
static void SPI_readByte(INT8U* byte) {
    *byte = 0;
    for (int i = 0; i < 8; i++) {
        gpio_set_level(READER_CLK_PIN, 1);
        ets_delay_us(50);
        if (gpio_get_level(READER_MISO_PIN)) {
            *byte |= (1 << i);
        }
        gpio_set_level(READER_CLK_PIN, 0);
        ets_delay_us(50);
    }
}