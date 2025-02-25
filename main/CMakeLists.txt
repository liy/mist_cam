idf_component_register(SRCS "main.c"
    PRIV_INCLUDE_DIRS .
    PRIV_REQUIRES driver nvs_flash esp_psram fatfs)
