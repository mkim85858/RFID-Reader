set(srcs "main.c" 
    "Hardware/CpuDrv.c"
    "Scanner/ScannerApi.c"
    "Scanner/PN532Drv.c"
    )

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "." "Common" "Hardware" "Scanner")
