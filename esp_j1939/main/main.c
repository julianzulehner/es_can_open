/*
 * Copyright (c) 2024 Your Julian Zulehner
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for more information.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "math.h"


/*************** DEFINITIONS ***************/

/* task priorities*/
#define MAIN_TASK_PRIO 1
#define RX_INTERRUPT_PRIO 5
#define CHECK_BUS_PRIO 1
#define CAN_NO_RESET 0

/* CONFIGURATION PART - CHANGE VALUES OF THIS SECTION */
#define CAN_TX GPIO_NUM_0
#define CAN_RX GPIO_NUM_2
#define CAN_BAUD_RATE TWAI_TIMING_CONFIG_250KBITS()
#define CAN_MSG_FILTER TWAI_FILTER_CONFIG_ACCEPT_ALL()
#define TX_TIMEOUT pdMS_TO_TICKS(500)
#define CHECK_CAN_BUS_PERIOD pdMS_TO_TICKS(3000)

/* UART CONFIGURATION */
#define UART_TX GPIO_NUM_6
#define UART_RX GPIO_NUM_5
#define UART_BUFFER_SIZE (1024)
#define UART_BAUDRATE 115200
#define UART_DELAY 10000000 // 10 seconds in us

/* Sensor specific definitions - do not change */
#define MSG_ID_VISC_DENS_DIEL  0xFD08
#define MSG_ID_RESI  0xFA67
#define MSG_ID_TEMP  0xFEEE
#define MSG_ID_DIAG  0xFF31
#define DIAGNOSTIC_MASK 0x3FF // first 6 bits are ignored

#define NR_DECIMAL_DEFAULT 3
#define FAC_VISCOSITY 0.015625
#define FAC_DENSITY 0.00003052
#define FAC_DIELECTRIC 0.00012207
#define FAC_RESISTANCE 1
#define FAC_TEMPERATURE 0.03125
#define OFFSET_TEMPERATURE -273.15
#define NO_OFFSET 0.0

/* structure definitions */
typedef struct {
    uint32_t viscosity;
    uint32_t density;
    uint32_t dielectric;
    uint32_t resistance;
    uint32_t temperature;
    uint32_t diagnostics;
} sensor_data_t;

/*************** GLOBAL VARIABLES ***************/
uint8_t reset = CAN_NO_RESET; // variable that can exit the main program. 
esp_err_t err;
TaskHandle_t rxInterruptTaskHandle;
TaskHandle_t checkBusTaskHandle;
TaskHandle_t mainTaskHandle;
twai_status_info_t controllerStatus;
twai_message_t rxMsg;
sensor_data_t sensorData;
esp_timer_handle_t uartTimerHandle;
uart_port_t uartPort = UART_NUM_1;
QueueHandle_t uartQueueHandle;

/* Helper functions */
uint16_t extract_uint16(twai_message_t *msg, uint8_t startbyte){
    uint16_t value = 0;
    for(uint8_t i = 0; i < 2; i++){
        value |= (msg->data[startbyte+i] << (8 * i));
    }
    return value;
}

uint32_t extract_uint24(twai_message_t *msg, uint8_t startbyte){
    uint32_t value = 0;
    for(uint8_t i = 0; i < 3; i++){
            value |= (msg->data[startbyte+i] << (8 * i));
    }
    return value;
}

uint32_t extract_uint32(twai_message_t *msg, uint8_t startbyte){
    uint32_t value = 0;
    for(uint8_t i = 0; i < 4; i++){
            value |= (msg->data[startbyte+i] << (8 * i));
    }
    return value;
}

/* Writes 0 values to the full data structur sensor_data_t */
void init_sensor_data(sensor_data_t* data){
    data->density = 0;
    data->diagnostics = 0;
    data->dielectric = 0;
    data->resistance = 0;
    data->temperature = 0;
    data->viscosity = 0;
}

uint32_t get_extd_identifier(twai_message_t* msg){
    uint32_t identifier = (msg->identifier >> 8) & 0xFFFF;
    return identifier;
}

void print_sensor_data(sensor_data_t* data){
    printf("Viscosity:    %lu\n", data->viscosity);
    printf("Density:    %lu\n", data->density);
    printf("Dielectric:    %lu\n", data->dielectric);
    printf("Resistance:    %lu\n", data->resistance);
    printf("Temperature:    %lu\n", data->temperature);
    printf("Diagnostics:    0x%lX\n", data->diagnostics);
}

/* Sends the sensor data via UART to second ESP32C3 device or any other device */
void uart_send_sensor_data(void* arg){
    print_sensor_data(&sensorData);
    uint8_t buffer[sizeof(sensor_data_t)];
    memcpy(buffer, &sensorData, sizeof(sensor_data_t));
    uart_write_bytes(uartPort, (const char*)buffer, sizeof(sensor_data_t));
}

/* Initializes the uart timer */
void init_uart_timer(){
    esp_timer_create_args_t uart_timer_args;
    uart_timer_args.callback = (void*)uart_send_sensor_data;
    uart_timer_args.dispatch_method = ESP_TIMER_TASK;
    uart_timer_args.name = "uart_timer";
    ESP_ERROR_CHECK(esp_timer_create(&uart_timer_args, &uartTimerHandle));
}

/* Converts the raw sensor value into the physical unit and returns uint32_t */
uint32_t convert_value(uint32_t value, float factor, float offset){
    float floatValue =  (float)value;
    float floatResult =  ((floatValue*factor)+offset)*pow(10,NR_DECIMAL_DEFAULT);
    if(floatResult > 0){
        return (uint32_t)floatResult;
    } else{
        return (uint32_t)0;
    }
}

/* 
Parses raw J1939 messages and stores data in sensor_data_t struct.
This function also starts the UART timer. The latest data will then
(after a defined delay) be sent via UART to another device.
*/
void parse_message(sensor_data_t* data, twai_message_t* msg){
    uint32_t identifier = get_extd_identifier(msg);
    uint32_t rawValue;
    uint32_t convertedValue;
    switch(identifier){
        case MSG_ID_VISC_DENS_DIEL:
            rawValue = extract_uint16(msg, 0);
            convertedValue = convert_value(rawValue, FAC_VISCOSITY, NO_OFFSET);
            data->viscosity = convertedValue;
            printf("INFO: Viscosity updated\n");

            rawValue = extract_uint16(msg, 2);
            convertedValue = convert_value(rawValue, FAC_DENSITY, NO_OFFSET);
            data->density = convertedValue;
            printf("INFO: Density updated\n");

            rawValue = extract_uint16(msg, 6);
            convertedValue = convert_value(rawValue, FAC_DIELECTRIC, NO_OFFSET);
            data->dielectric = convertedValue;
            printf("INFO: Dielectric updated\n");
            break;

        case MSG_ID_RESI:
            rawValue = extract_uint24(msg, 0);
            convertedValue = convert_value(rawValue, FAC_RESISTANCE, NO_OFFSET);
            data->resistance = convertedValue;
            printf("INFO: Resistance updated\n");
            break;
        case MSG_ID_DIAG:
            data->diagnostics = extract_uint24(msg, 0) & DIAGNOSTIC_MASK;
            printf("INFO: Diagnostics updated\n");
            break;
        case MSG_ID_TEMP:
            rawValue = extract_uint16(msg, 2);
            convertedValue = convert_value(rawValue, FAC_TEMPERATURE, 
                                            OFFSET_TEMPERATURE);
            data->temperature = convertedValue;
            printf("INFO: Temperature updated\n");
            esp_timer_start_once(uartTimerHandle, UART_DELAY);
            break;
        default:
            printf("ERROR: Message ID 0x%lX not supported\n", identifier);
    }
}

/* Initializes UART communication */
void init_uart(uart_port_t uartPort){
    uart_config_t uartConfig = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    int intrAllocFlags = 0;
    ESP_ERROR_CHECK(uart_driver_install(uartPort, UART_BUFFER_SIZE, 0, 0,
                    NULL, intrAllocFlags));
    ESP_ERROR_CHECK(uart_param_config(uartPort, &uartConfig));
    ESP_ERROR_CHECK(uart_set_pin(uartPort, UART_TX, UART_RX, UART_PIN_NO_CHANGE,
                    UART_PIN_NO_CHANGE));
}

/* Prints a can tx message to the standard output*/
void can_print_message(twai_message_t *msg){
    printf("MESSAGE ID: %lX, DLC: %d, DATA: [0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X]\n", 
    (uint32_t)msg->identifier,
    msg->data_length_code,
    msg->data[0],
    msg->data[1],
    msg->data[2],
    msg->data[3],
    msg->data[4],
    msg->data[5],
    msg->data[6],
    msg->data[7]
    );
}

/* Config the CAN module */
void can_config_module(void){
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = CAN_BAUD_RATE;
    twai_filter_config_t f_config = CAN_MSG_FILTER;

    /* This should improve number of error frames on install */
    gpio_set_direction(CAN_TX, GPIO_MODE_OUTPUT);
    gpio_set_level(CAN_TX, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Uninstalled to stopped state
    printf("INFO: Installing CAN module... ");
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Done\n");
        }
    else{
        printf("\nERROR: Failed to install CAN module\n");
    }
}

/* Starts the CAN module */
void can_start_module(void){
    printf("INFO: Starting CAN module... ");
    if (twai_start() == ESP_OK) {
        printf("Done\n");
    }
    else {
        printf("\nERROR: Failed to start CAN module\n");
        }
}

/* Stops the can module */
void can_stop_module(void){
    printf("INFO: Stopping CAN module... ");
    if(twai_stop() == ESP_OK){
        printf("Done\n");
    } else {
        printf("\nERROR: Could not stop CAN module\n");
    }

}

/* Uninstalls module and frees memory */
void can_uninstall_module(void){
    printf("INFO: Uninstalling CAN module... ");
    if(twai_driver_uninstall() == ESP_OK){
        printf("Done\n");
    } else {
        printf("\nERROR: CAN module could not be uninstalled\n");
    }
}

/* INTERRUPT TASK FOR RECEIVING MESSAGES AND MESSAGE HANDLING */
void rx_interrupt_task(void *arg){
    while(1){
    if(!twai_receive(&rxMsg, portMAX_DELAY) == ESP_OK){
        printf("ERROR: CAN RX Error\n");
    } else {
        // Implement here the code of message handling
        parse_message(&sensorData, &rxMsg);
        //can_print_message(&rxMsg);
    }
    vTaskDelay(1);
    }
}

/* TASK THAT CHECKS THE HEALTH OF THE CAN CONTROLLER */
void check_bus_task(void *arg){
    while(1){
        ESP_ERROR_CHECK(twai_get_status_info(&controllerStatus));
        if((controllerStatus.tx_error_counter != 0 )|
           (controllerStatus.rx_error_counter != 0)){
            printf("INFO: TX Error Nr.=%lu, RX Error Nr.=%lu\n", 
            controllerStatus.tx_error_counter,
            controllerStatus.rx_error_counter);
           }
        vTaskDelay(CHECK_CAN_BUS_PERIOD);
    }
}



/* MAIN TASK THAT CONTAINS ALL OTHER SUBTASKS */
void main_task(void *arg){
    esp_task_wdt_add(mainTaskHandle);
    
    /* Prepare sensor data and set all values to 0 */
    init_sensor_data(&sensorData);

    /* Init the UART communication port */
    init_uart(uartPort);


    /* Installs CAN module */
    can_config_module(); // configuration in CANopen.h
    can_start_module();

    
    /* Create subtasks */
    xTaskCreate(rx_interrupt_task, "rx_interrupt_task", 4096, NULL, 
                RX_INTERRUPT_PRIO, &checkBusTaskHandle);
    xTaskCreate(check_bus_task, "check_bus_task", 2048, NULL, 
                CHECK_BUS_PRIO, &rxInterruptTaskHandle);

    /* Create timer handle to send data via uart after all sensor data was sent*/
    init_uart_timer();


    while(reset == CAN_NO_RESET){
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Exiting program */
    printf("INFO: Exiting main task\n");
    vTaskDelete(rxInterruptTaskHandle);
        vTaskDelete(checkBusTaskHandle);
    esp_timer_stop(uartTimerHandle);
    uart_driver_delete(uartPort);
    can_stop_module();
    can_uninstall_module();
    esp_restart();

}

/* MAIN ENTRY POINT AFTER START */
void app_main()
{
    xTaskCreate(main_task, "main_task", 4096, NULL, MAIN_TASK_PRIO, &mainTaskHandle);
}