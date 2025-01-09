#ifndef _PN532_DRV_H
#define _PN532_DRV_H

#include "Globals.h"

/*
********************************************************************************
*                       GLOBAL DEFINITIONS & MACROS
********************************************************************************
*/
/* Insert #define here */
/*
********************************************************************************
*                       GLOBAL DATA TYPES & STRUCTURES
********************************************************************************
*/
/* Insert global typedef & struct here */
/*
********************************************************************************
*                       GLOBAL(EXPORTED) VARIABLE EXTERNALS
********************************************************************************
*/
/* Insert global variable externals here */
/*
********************************************************************************
*                       GLOBAL(EXPORTED) FUNCTION PROTOTYPES
********************************************************************************
*/
/* Insert global function prototypes here */
extern esp_err_t PN532_Init(void);
extern esp_err_t PN532_WriteCommand(INT8U *cmd, INT8U cmd_length);
extern esp_err_t PN532_ReadResponse(INT8U *buf, INT8U buf_length, INT8U *rsp_length);

#endif
