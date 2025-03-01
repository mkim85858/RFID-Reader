/**
********************************************************************************
*                       INCLUDES
********************************************************************************
*/
#include <string.h>

#include "esp_spiffs.h"
#include "esp_log.h"

#include "Globals.h"
#include "HardwareConfig.h"
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
#define TAG_LENGTH          14
#define PATH                "/spiffs/tag_data.bin"
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
* @brief    Storage Init
* @param    none
* @return   none
* @remark   Used for initializing storage during startup
********************************************************************************
*/
void Storage_Init(void) {
    // initializing spiffs
    esp_vfs_spiffs_conf_t spiffscfg = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true,
    };
    esp_vfs_spiffs_register(&spiffscfg);

    // ensuring the file exists
    FILE *file = fopen(PATH, "rb");
    if (!file) {
        file = fopen(PATH, "wb");
        fclose(file);
    }
}

/**
********************************************************************************
* @brief    Storage Write
* @param    tag = tag being stored
* @return   none
* @remark   Used to write tag data to storage
********************************************************************************
*/
void Storage_Write(INT8U* tag) {
    FILE *file = fopen(PATH, "ab");
    
    fwrite(tag, 1, TAG_LENGTH, file);
    fclose(file);
}

/**
********************************************************************************
* @brief    Storage Read
* @param    tag = tag to compare against
* @return   valid/invalid
* @remark   Used to check if tag data is in storage
********************************************************************************
*/
BOOLEAN Storage_Read(INT8U *tag) {
    FILE *file = fopen(PATH, "rb");
    INT8U temp[TAG_LENGTH];

    // checking if tag matches
    while (fread(temp, TAG_LENGTH, 1, file) == TAG_LENGTH) {
        if (!memcmp(temp, tag, TAG_LENGTH)) {
            return 1;
        }
    }
    return 0;
}

/**
********************************************************************************
* @brief    Storage Clear
* @param    none
* @return   none
* @remark   Used to clear storage
********************************************************************************
*/
void Storage_Clear(void) {
    remove(PATH);
}

/*
********************************************************************************
*                       LOCAL FUNCTIONS
********************************************************************************
*/
/* Insert local functions here */
