set(srcs "main.c" 
    "Scanner/ScannerApi.c"
    )

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "." "Common" "Hardware" "Scanner")
