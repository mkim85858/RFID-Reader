#ifndef _HARDWARE_CONFIG_H
#define _HARDWARE_CONFIG_H

/*
********************************************************************************
*                       GLOBAL DEFINITIONS & MACROS
********************************************************************************
*/
/* Insert #define here */
	
    /** RFID READER */
    #define READER_HOST        VSPI_HOST
    #define READER_SS_PIN      GPIO_NUM_5
    #define READER_CLK_PIN     GPIO_NUM_18
    #define READER_MISO_PIN    GPIO_NUM_19  
    #define READER_MOSI_PIN    GPIO_NUM_23
    
    /** BUZZER */
    #define BUZZER_PIN          GPIO_NUM_22

    /** BUTTON */
    #define BUTTON_PIN          GPIO_NUM_4 // will change
    
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
