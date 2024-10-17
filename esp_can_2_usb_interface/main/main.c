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
#include "driver/usb_serial_jtag.h"
#include "can.h"
#include "lawicel_emulator.h"

/*************** DEFINITIONS ***************/

#define MAIN_TASK_PRIO 1
#define RX_INTERRUPT_PRIO 5
#define UART_RX_INTERRUPT_PRIO 5
#define CHECK_BUS_PRIO 1
#define CHECK_CAN_BUS_PERIOD pdMS_TO_TICKS(3000)
#define LED GPIO_NUM_8
#define BUF_SIZE 1024

typedef enum {
    STOP,
    RUNNING,
    ERROR,
} interface_state_t;

typedef struct {
    interface_state_t state;
    twai_message_t rxMsg;
    twai_message_t txMsg;
    twai_status_info_t twaiStatus;
} interface_t;

TaskHandle_t mainTaskHandle;
TaskHandle_t uartRxInterruptTaskHandle;
TaskHandle_t rxInterruptTaskHandle;
TaskHandle_t checkBusTaskHandle;
esp_timer_handle_t timerHandle;
twai_status_info_t controllerStatus;
twai_message_t rxMsg;
twai_message_t txMsg;
interface_t interface;
int reset = 0;
//uint8_t* buffer;
uint8_t counter = 0;
esp_err_t err;



void test_fun(twai_message_t * msg)
{
    printf("%d\n", msg->data[0]);
}

void send_can_msg_via_uart(const twai_message_t *msg)
{
    uint8_t buffer[sizeof(twai_message_t)];
    memcpy(buffer, msg, sizeof(twai_message_t));
    usb_serial_jtag_write_bytes(buffer, sizeof(twai_message_t), 
                                    pdMS_TO_TICKS(20));
}

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
        send_can_msg_via_uart(&rxMsg);
    }
    vTaskDelay(1);
    }
}

void uart_rx_interrupt_task(void *arg)
{
    uint8_t *rxBuffer = (uint8_t *) malloc(BUF_SIZE);
    while(1)
    {
        int length = usb_serial_jtag_read_bytes(rxBuffer, (BUF_SIZE-1), 
                     pdMS_TO_TICKS(1000));
        if (length == sizeof(twai_message_t)) 
        {   
            memcpy(&txMsg, rxBuffer, sizeof(twai_message_t));
            twai_transmit(&txMsg, portMAX_DELAY);
            esp_timer_start_once(timerHandle, 0);

        }
        vTaskDelay(1);
    }
    free(rxBuffer);
    
}


/* TASK THAT CHECKS THE HEALTH OF THE CAN CONTROLLER */
void check_bus_task(void *arg)
{
    while(1)
    {
        twai_get_status_info(&controllerStatus);
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
    esp_task_wdt_add(mainTaskHandle);
    
    // Set LED gpio as output
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_set_level(LED, 1);
    
    // Configure USB SERIAL JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = BUF_SIZE,
        .tx_buffer_size = BUF_SIZE,
    };

    /* Create timers */
    esp_timer_create_args_t timerArgs = {
        .callback = toggle_led,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "toggle_led_timer",
    };
    esp_timer_create(&timerArgs, &timerHandle);

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));

    can_config_module(); // configuration in CANopen.h
    
    
    xTaskCreate(uart_rx_interrupt_task, "uart_rx_interrupt", 4096, NULL,
                UART_RX_INTERRUPT_PRIO, &uartRxInterruptTaskHandle);

    can_start_module();
    /* Create subtasks */
    xTaskCreate(rx_interrupt_task, "rx_interrupt_task", 4096, NULL, 
                RX_INTERRUPT_PRIO, &rxInterruptTaskHandle);
    //xTaskCreate(check_bus_task, "check_bus_task", 2048, NULL, 
    //            CHECK_BUS_PRIO, &checkBusTaskHandle);

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
    usb_serial_jtag_driver_uninstall();
    esp_restart();
}

/* MAIN ENTRY POINT AFTER START */
void app_main()
{
    xTaskCreate(main_task, "main_task", 4096, NULL, MAIN_TASK_PRIO, &mainTaskHandle);
}
