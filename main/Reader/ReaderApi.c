/**
********************************************************************************
*                       INCLUDES
********************************************************************************
*/
#include <string.h>

#include "rom/ets_sys.h"
#include "Globals.h"
#include "HardwareConfig.h"

#include "ReaderApi.h"
#include "PN532Drv.h"
#include "BuzzerDrv.h"
#include "StorageApi.h"

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
#define SAMCONFIG_NORM              {0x14, 0x01, 0x00, 0x00}
#define RFCONFIG_AUTO               {0x32, 0x01, 0x02}
#define INAUTOPOLL                  {0x60, 0xFF, 0x05, 0x00}
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
static INT8U rspBuffer[50];
/*
********************************************************************************
*                       LOCAL FUNCTION PROTOTYPES
********************************************************************************
*/
static void Reader_SendCommand(INT8U *cmd, INT8U cmd_length, INT8U rsp_length);
/*
********************************************************************************
*                       GLOBAL(EXPORTED) FUNCTIONS
********************************************************************************
*/
/* Insert global functions here */
/**
********************************************************************************
* @brief    Reader Init
* @param    none
* @return   none
* @remark   Initializes reader for reading tags
********************************************************************************
*/
void Reader_Init(void) {
    PN532_Init();
    Reader_SendCommand((INT8U[])SAMCONFIG_NORM, 4, 1);
    Reader_SendCommand((INT8U[])RFCONFIG_AUTO, 3, 1);
    // add any more config needed during startup
    memset(rspBuffer, 0, 50);
}

/**
********************************************************************************
* @brief    Reader Poll Tag
* @param    none
* @return   none
* @remark   Continuously polls for a tag and checks if it's valid/invalid
********************************************************************************
*/
void Reader_PollTag(void) {
    Reader_SendCommand((INT8U[])INAUTOPOLL, 4, 30);

    // extracting tag information from buffer
    INT8U tagData[14];
    memcpy(tagData, rspBuffer + 5, 14);
    memset(rspBuffer, 0, 50);

    // Checking if tag is already stored in memory or not
    if (Storage_Read(tagData)) {
        Buzzer_Once();
    }
    else {
        Buzzer_Twice();
    }
}

/**
********************************************************************************
* @brief    Reader Save Tag
* @param    none
* @return   none
* @remark   Continuously polls for a tag and saves it to memory
********************************************************************************
*/
void Reader_SaveTag(void) {
    Reader_SendCommand((INT8U[])INAUTOPOLL, 4, 30);

    // extracting tag information from buffer
    INT8U tagData[14];
    memcpy(tagData, rspBuffer + 5, 14);
    memset(rspBuffer, 0, 50);

    // If tag is not already stored, write to memory
    if (Storage_Read(tagData) == 0) {
        Storage_Write(tagData);
        Buzzer_Once();
    }
    else {
        Buzzer_Twice();
    }
}

/*
********************************************************************************
*                       LOCAL FUNCTIONS
********************************************************************************
*/
/* Insert local functions here */

// Sends a command, receives ACK and response
static void Reader_SendCommand(INT8U *cmd, INT8U cmd_length, INT8U rsp_length) {
    memset(rspBuffer, 0, 50);
    PN532_WriteCommand(cmd, cmd_length);            // Sending command
    PN532_ReadResponse(rspBuffer, 6);               // Receiving ack/nack
    for (INT8U i = 0; i < 5; i++) {                 // Receiving response
        PN532_ReadResponse(rspBuffer, rsp_length + 8);
        if (rspBuffer[1] == 0xFF || rspBuffer[2] == 0xFF) {
            break;
        }
        else {
            PN532_WriteNACK();
        }
    }
    ets_delay_us(100);
}