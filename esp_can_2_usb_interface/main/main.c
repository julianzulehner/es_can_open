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
#include "driver/uart.h"
#include "esp_timer.h"

#include "can.h"
#include "lawicel_emulator.h"

/*************** DEFINITIONS ***************/

#define MAIN_TASK_PRIO 1
#define RX_INTERRUPT_PRIO 5
#define UART_RX_INTERRUPT_PRIO 5
#define CHECK_BUS_PRIO 1
#define CHECK_CAN_BUS_PERIOD pdMS_TO_TICKS(3000)
#define LED GPIO_NUM_8
#define VERBOSITY ESP_LOG_NONE

/*************** GLOBAL VARIABLES ***************/
uint8_t reset = 0; // variable that can exit the main program. 
esp_err_t err;
TaskHandle_t rxInterruptTaskHandle;
TaskHandle_t checkBusTaskHandle;
TaskHandle_t mainTaskHandle;
TaskHandle_t uartRxInterruptTaskHandle;
esp_timer_handle_t timerHandle;
twai_status_info_t controllerStatus;
twai_message_t rxMsg;
twai_message_t txMsg;

void toggle_led(void *arg){
    gpio_set_level(LED, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LED, 1);
}

/* INTERRUPT TASK FOR RECEIVING CAN MESSAGES AND MESSAGE HANDLING */
void rx_interrupt_task(void *arg)
{
    while(1)
    {
    if(!twai_receive(&rxMsg, portMAX_DELAY) == ESP_OK){
        printf("ERROR: CAN RX Error\n");
    } else {
        esp_timer_start_once(timerHandle, 0);
        twai_message_to_serial(&rxMsg);
    }
    vTaskDelay(1);
    }
}

void uart_rx_interrupt_task(void *arg)
{
    while(1)
    {
        uint8_t data[8];
        int length = uart_read_bytes(UART_NUM_0, data, 8, pdMS_TO_TICKS(20));
        if (length > 0) {
            data[length] = '\0';
            printf("Received: %s\n", data);
        }
    }
    vTaskDelay(1);
}

/* TASK THAT CHECKS THE HEALTH OF THE CAN CONTROLLER */
void check_bus_task(void *arg)
{
    while(1)
    {
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
void main_task(void *arg)
{
    esp_log_level_set("*", VERBOSITY);
    esp_task_wdt_add(mainTaskHandle);
    
    can_config_module(); // configuration in CANopen.h
    can_start_module();

    // Set LED gpio as output
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_set_level(LED, 1);

    // Init uart 
    const uart_config_t uartConfig = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uartConfig));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 2048, 2048, 0, NULL, 0));
    /* Create timers */
    esp_timer_create_args_t timerArgs = {
        .callback = toggle_led,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "toggle_led_timer",
    };

    esp_timer_create(&timerArgs, &timerHandle);
    /* Create subtasks */
    xTaskCreate(rx_interrupt_task, "rx_interrupt_task", 4096, NULL, 
                RX_INTERRUPT_PRIO, &checkBusTaskHandle);
    xTaskCreate(check_bus_task, "check_bus_task", 2048, NULL, 
                CHECK_BUS_PRIO, &rxInterruptTaskHandle);
    xTaskCreate(uart_rx_interrupt_task, "uart_rx_interrupt_task", 4096, NULL,
                UART_RX_INTERRUPT_PRIO, &uartRxInterruptTaskHandle);

    while(reset == 0)
    {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Exiting program */
    printf("INFO: Exiting main task\n");
    vTaskDelete(rxInterruptTaskHandle);
    vTaskDelete(checkBusTaskHandle);
    vTaskDelete(uartRxInterruptTaskHandle);
    can_stop_module();
    can_uninstall_module();
    esp_restart();
}

/* MAIN ENTRY POINT AFTER START */
void app_main()
{
    xTaskCreate(main_task, "main_task", 4096, NULL, MAIN_TASK_PRIO, &mainTaskHandle);
}