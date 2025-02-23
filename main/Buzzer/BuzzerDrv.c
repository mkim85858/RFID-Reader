/**
********************************************************************************
*                       INCLUDES
********************************************************************************
*/
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"

#include "Globals.h"
#include "HardwareConfig.h"
#include "BuzzerDrv.h"
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

/*
********************************************************************************
*                       GLOBAL(EXPORTED) FUNCTIONS
********************************************************************************
*/
/* Insert global functions here */
/**
********************************************************************************
* @brief    Buzzer Init
* @param    none
* @return   none
* @remark   Used for initializing buzzer during startup
********************************************************************************
*/
void Buzzer_Init(void) {
    // Configuring buzzer pin
    gpio_config_t buzzercfg = {
        .pin_bit_mask = (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&buzzercfg);
    gpio_set_level(BUZZER_PIN, 0);
}

/**
********************************************************************************
* @brief    Buzz Once
* @param    none
* @return   none
* @remark   Used to indicate a valid tag
********************************************************************************
*/
void Buzzer_Once(void) {
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(BUZZER_PIN, 0);
}

/**
********************************************************************************
* @brief    Buzz Twice
* @param    none
* @return   none
* @remark   Used to indicate an invalid tag
********************************************************************************
*/
void Buzzer_Twice(void) {
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(BUZZER_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(BUZZER_PIN, 0);
}
/*
********************************************************************************
*                       LOCAL FUNCTIONS
********************************************************************************
*/
/* Insert local functions here */
