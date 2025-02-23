set(srcs "main.c" 
    "Reader/ReaderApi.c"
    "Reader/PN532Drv.c"
    "Buzzer/BuzzerDrv.c"
    "Storage/StorageApi.c"
    "Button/ButtonDrv"
    )

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "." "Common" "Reader" "Buzzer" "Storage" "Button")
