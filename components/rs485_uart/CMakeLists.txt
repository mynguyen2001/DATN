idf_component_register(SRCS "rs485_uart.cpp" 
                    INCLUDE_DIRS "." ".." "../../main"
                    REQUIRES esp_firebase
                    
                    PRIV_REQUIRES esp_http_client esp-tls nvs_flash )