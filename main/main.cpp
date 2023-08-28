
#include <iostream>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "jsoncpp/value.h"
#include "jsoncpp/json.h"

#include "esp_firebase/app.h"
#include "esp_firebase/rtdb.h"

#include "wifi_utils.h"

#include "firebase_config.h"
#include "time.h"
#include "rs485_uart.h"
using namespace ESPFirebase;
extern int receive_flag;
extern uint8_t request[8];
extern uint8_t rx_buffer[512];
extern float actual_voltage,recharge_current,residual_capacity, en_temp, cell_temp, board_temp;
extern int cell_quantity;
extern int  soc, soh;
extern float cell0, cell1, cell2, cell3, cell4, cell5, cell6, cell7, cell8, cell9, cell10, cell11, cell12, cell13;
// extern SemaphoreHandle_t response_semaphore;

extern "C" void app_main(void)
{
    //init uart
    uart_init_configure();
    //connect to WiFi
    wifiInit(SSID, PASSWORD); // blocking until it connects

    // Config and Authentication
    xTaskCreate(uart_event_task, "UART Event Task", 8192, NULL, 1, NULL);
    xTaskCreate(check_resquest_response_task, "check_modbus_response_task", 8192, NULL, 1, NULL);
 
}
