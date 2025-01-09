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
    #define SCANNER_SS_PIN      GPIO_NUM_5
    #define SCANNER_SCK_PIN     GPIO_NUM_18
    #define SCANNER_MISO_PIN    GPIO_NUM_19
    #define SCANNER_RSTO_PIN    GPIO_NUM_21
    #define SCANNER_IRQ_PIN     GPIO_NUM_22
    #define SCANNER_MOSI_PIN    GPIO_NUM_23
    
    /** BUZZER */
    //#define BUZZER_PIN          GPIO_NUM_21
    
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
