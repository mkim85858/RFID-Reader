#ifndef _HARDWARE_CONFIG_H
#define _HARDWARE_CONFIG_H

/*
********************************************************************************
*                       GLOBAL DEFINITIONS & MACROS
********************************************************************************
*/
/* Insert #define here */
	
    /** RFID SCANNER */
    #define SCANNER_HOST        VSPI_HOST
    #define SCANNER_MISO_PIN    GPIO_NUM_1
    #define SCANNER_SS_PIN      GPIO_NUM_8
    #define SCANNER_SCK_PIN     GPIO_NUM_9
    #define SCANNER_MOSI_PIN    GPIO_NUM_23
    
    /** BUZZER */
    #define BUZZER_PIN          GPIO_NUM_22
    
/*
*
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

#endif /* _HARDWARE_CONFIG_H */
