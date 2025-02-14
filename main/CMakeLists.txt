set(srcs "main.c" 
    "Reader/ReaderApi.c"
    "Reader/PN532Drv.c"
    )

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "." "Common" "Reader")
