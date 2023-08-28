#ifndef _RS485_UART_H_
#define _RS485_UART_H_

#include <stdint.h> // Include the necessary header for uint16_t and uint8_t
#include <stddef.h> // Include the necessary header for size_t
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define TAG "RS485_ECHO_APP"

// Note: Some pins on target chip cannot be assigned for UART communication.
// Please refer to documentation for selected board and target to configure pins using Kconfig.
#define UART_NUM UART_NUM_2
#define TX_PIN 17
#define RX_PIN 16
#define RTS_PIN 27

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define ECHO_TEST_RTS   (CONFIG_ECHO_UART_RTS)

// CTS is not used in RS485 Half-Duplex Mode

#define BUF_SIZE        (512)
#define BAUD_RATE       (CONFIG_ECHO_UART_BAUD_RATE)

// Read packet timeout
#define PACKET_READ_TICS        (100 / portTICK_RATE_MS)
#define ECHO_TASK_STACK_SIZE    (2048)
#define ECHO_TASK_PRIO          (10)
#define ECHO_UART_PORT          (CONFIG_ECHO_UART_PORT_NUM)
// create modbus
#define SLAVE_ID 1
#define FUNCTION_CODE 3
#define REGISTER_ADDRESS_MSB 0x00
#define REGISTER_ADDRESS_LSB 0x00
#define REGISTER_COUNT_MSB 0x16
#define REGISTER_COUNT_LSB 0x00

#define REQUEST_DELAY_MS 10000
#define RESPONSE_TIMEOUT_MS 5000


#define RD_BUF_SIZE (BUF_SIZE)

void uart_init_configure();
uint16_t calculate_crc(const uint8_t *request, size_t length);
void create_modbus_request(uint8_t *request);
void process_data();
void convertHexToDecimal(const uint8_t* rx_buffer , int length, unsigned int* decimalArray);
void uart_event_task(void *pvParameters);
void send_modbus_request();
void check_resquest_response_task(void *pvParameters);


#endif