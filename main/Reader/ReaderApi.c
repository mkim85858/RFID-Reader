/**
********************************************************************************
*                       INCLUDES
********************************************************************************
*/
#include <stdio.h>
#include <string.h>
#include "ReaderApi.h"
#include "PN532Drv.h"
#include "Globals.h"
#include "HardwareConfig.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
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
static INT8U rspBuffer[30];
/*
********************************************************************************
*                       LOCAL FUNCTION PROTOTYPES
********************************************************************************
*/
static void ReaderSendCommand(INT8U *cmd, INT8U cmd_length, INT8U rsp_length);
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
void ReaderInit(void) {
    PN532_Init();
    ReaderSendCommand((INT8U[])SAMCONFIG_NORM, 4, 7);
    ReaderSendCommand((INT8U[])RFCONFIG_AUTO, 3, 8);
    // add any more config needed during startup
}

void ReaderPoll(void) {
    int i = 0;
}


/*
********************************************************************************
*                       LOCAL FUNCTIONS
********************************************************************************
*/
/* Insert local functions here */

// Sends a command, receives ACK and response
static void ReaderSendCommand(INT8U *cmd, INT8U cmd_length, INT8U rsp_length) {
    memset(rspBuffer, 0, 30);
    PN532_WriteCommand(cmd, cmd_length);            // Sending command
    PN532_ReadResponse(rspBuffer, 6);               // Receiving ack
    PN532_ReadResponse(rspBuffer, rsp_length);      // Receiving response
    ets_delay_us(1000);
}