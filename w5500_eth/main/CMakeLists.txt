idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_eth esp_netif nvs_flash ethernet_init
    PRIV_REQUIRES driver
)