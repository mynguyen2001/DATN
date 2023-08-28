#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "jsoncpp/value.h"
#include "jsoncpp/json.h"

#include "esp_firebase/app.h"
#include "esp_firebase/rtdb.h"

#include "wifi_utils.h"

#include "firebase_config.h"
#include <math.h>

#include <esp_err.h>
// #include <nvs_flash.h>
// #include <nvs.h>
#include "rs485_uart.h"

/**
 * This is a example which echos any data it receives on UART back to the sender using RS485 interface in half duplex mode.
 */
#define TAG "RS485_ECHO_APP"

// Note: Some pins on target chip cannot be assigned for UART communication.
// Please refer to documentation for selected board and target to configure pins using Kconfig.

uint8_t rx_buffer[512];
uint8_t request[8];
TickType_t request_sent_time = 0;
int receive_flag = 0;
QueueHandle_t uart_queue;
int length = 0;
// declare global variable
float actual_voltage,recharge_current, residual_capacity, en_temp, cell_temp, board_temp;
float total_voltage, av_cell_voltage,  equilibrium_alarm, vol_difference, balance_current, balance_vol, max_balance_current, balance_switch_set;
int cell_quantity;
int  soc, soh;
float cell0, cell1, cell2, cell3, cell4, cell5, cell6, cell7, cell8, cell9, cell10, cell11, cell12, cell13, cell14, cell15, cell16, cell17, cell18, cell19, cell20, cell21, cell22, cell23;

// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin

// static void echo_receive()
//  An example of echo test with hardware flow control on UART
void uart_init_configure()
{
    const int uart_num = UART_NUM;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
    };

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "Start RS485 application test and configure UART.");

    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    uart_driver_install(uart_num, BUF_SIZE * 2, 512, 10, &uart_queue, 0);

    // Configure UART parameters
    uart_param_config(uart_num, &uart_config);

    ESP_LOGI(TAG, "UART set pins, mode and install driver.");

    // Set UART pins as per KConfig settings
    uart_set_pin(uart_num, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // set driver
    uart_enable_pattern_det_baud_intr(uart_num, '+', 3, 9, 0, 0);
    // Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(uart_num, 20);

    ESP_LOGI(TAG, "UART start recieve loop.\r\n");
}
uint16_t calculate_crc(const uint8_t *request, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i)
    {
        unsigned short b = request[i];
        for (size_t j = 0; j < 8; ++j)
        {
            crc = ((b ^ crc) & 1 ? (crc >> 1) ^ 0xA001 : (crc >> 1));
            b >>= 1;
        }
    }
    return (crc << 8 | crc >> 8);
}

void create_modbus_request(uint8_t *request)
{
    request[0] = SLAVE_ID;
    request[1] = FUNCTION_CODE;
    request[2] = REGISTER_ADDRESS_MSB;
    request[3] = REGISTER_ADDRESS_LSB;
    request[4] = REGISTER_COUNT_LSB;
    request[5] = REGISTER_COUNT_MSB;

    // Calculate CRC and add it to the request
    uint16_t crc = calculate_crc(request, 6);
    request[7] = crc & 0xFF;        // CRC LSB
    request[6] = (crc >> 8) & 0xFF; // CRC MSB
}

void convertHexToDecimal(const uint8_t *rx_buffer, int length, unsigned int *decimalArray)
{
    for (int i = 3 ; i < length; i += 2)
    {
        uint8_t highByte = rx_buffer[i];
        uint8_t lowByte = rx_buffer[i + 1];
        uint16_t decimalNumber = (highByte << 8) | lowByte;
        decimalArray[i / 2] = decimalNumber;
    }
 }
void send_modbus_request()
{
    create_modbus_request(request);
    uart_write_bytes(UART_NUM_2, request, sizeof(request));
    vTaskDelay(100 / portTICK_RATE_MS);
    int bytes_sent = uart_write_bytes(UART_NUM_2, request, sizeof(request));
    if (bytes_sent > 0)
    {
        // printf("Data sent successfully: %d bytes\n", bytes_sent);
        printf("Sending request: ");
        for (int i = 0; i < 8; i++)
        {
            printf("%02X ", request[i]);
        }
        printf("\n");
    }

    else
    {
        printf("Failed to send data\n");
    }
    uart_wait_tx_done(UART_NUM, 100 / portTICK_RATE_MS);

}
void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    // BaseType_t xResult;
    while (1)
    {
        // Waiting for UART event.
        if (xQueueReceive(uart_queue, (void *)&event, (portTickType)portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA:
            {   

            ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length));
            // length = uart_read_bytes(UART_NUM_2, rx_buffer, length, 100);
            uart_read_bytes(UART_NUM_2, rx_buffer, length, 100);
            receive_flag = 1;
            uart_flush_input(UART_NUM_2);
            xQueueReset(uart_queue);

                break;
            }

            default:
            {
                break;
            }
            }
        }
    }
}
void process_data()
{
    printf("Received data:\n");
    for (int i = 0; i <length; i++)
    {
        printf("%02X ", rx_buffer[i]);
    }
    printf("\n");
    int arrayLength = length-2;
    int decimalArrayLength = arrayLength / 2;
    unsigned int decimalArray[decimalArrayLength];
    convertHexToDecimal(rx_buffer, arrayLength, decimalArray);
    printf("Decimal: ");
    for (int i = 1; i < decimalArrayLength; i++)
    {
        printf(" %u", decimalArray[i]);
    }
    printf("\n");
    actual_voltage = decimalArray[1];
    cell_quantity = decimalArray[2];
    soc = decimalArray[3];
    residual_capacity = decimalArray[4];
    soh= decimalArray[5];
    recharge_current = decimalArray[6];
    en_temp = decimalArray[7];
    cell_temp = decimalArray[8];
    board_temp = decimalArray[9];
    cell1 = decimalArray[10];
    cell2 = decimalArray[11];
    cell3 = decimalArray[12];
    cell4 = decimalArray[13];
    cell5 = decimalArray[14];
    cell6 = decimalArray[15];
    cell7 = decimalArray[16];
    cell8 = decimalArray[17];
    cell9 = decimalArray[18];
    cell10 = decimalArray[19];
    cell11 = decimalArray[20];
    cell12 = decimalArray[21];
    cell13 = decimalArray[22];


}
void check_resquest_response_task(void *pvParameters)
{
    // init access to Firebase
    ESPFirebase::user_account_t account = {USER_EMAIL, USER_PASSWORD};
    ESPFirebase::FirebaseApp app = ESPFirebase::FirebaseApp(API_KEY);

    app.loginUserAccount(account);
    ESPFirebase::RTDB db = ESPFirebase::RTDB(&app, DATABASE_URL);
    while (1)
    {
        send_modbus_request();
        request_sent_time = xTaskGetTickCount();
        while (1)
        {
            if (receive_flag == 1)
            {
                break;
            }
            if (xTaskGetTickCount() - request_sent_time > 5000 / portTICK_PERIOD_MS )
            {
                break;
            }
        }
        if (receive_flag != 1)
        {
            printf(" resending request\n");
            continue;
        }
        // process data
        process_data();
        receive_flag = 0;
        Json::Value data;
        Json::Reader reader;
    //   reader.parse(json_str, data);
    // put data to "modbus2" field in realtime database
        data["actual_voltage"] = actual_voltage/100;
        data["cell_quantity"] = cell_quantity;
        data["soc"] = soc;
        data["residual_capacity"] =residual_capacity/100;
        data["soh"] = soh;
        data["recharge_current"] = recharge_current/100;
        data["en_temp"] = en_temp;
        data["cell_temp"] = cell_temp;
        data["board_temp"] = board_temp;
        data["cell1"] = cell1/1000;
        data["cell2"] = cell2/1000;
        data["cell3"] = cell3/1000;
        data["cell4"] = cell4/1000;
        data["cell5"] = cell5/1000;
        data["cell6"] = cell6/1000;
        data["cell7"] = cell7/1000;
        data["cell8"] = cell8/1000;
        data["cell9"] = cell9/1000;
        data["cell10"] = cell10/1000;
        data["cell11"] = cell11/1000;
        data["cell12"] = cell12/1000;
        data["cell13"] = cell13/1000;

        db.putData("/modbus2", data);
        // vTaskDelay(600000 / portTICK_RATE_MS);
    }
}
