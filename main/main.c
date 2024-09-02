#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "CANopen.h"
#include "esp_task_wdt.h"
#include "OD.h"

/*************** DEFINITIONS ***************/

/* task priorities*/
#define MAIN_TASK_PRIO 1
#define RX_INTERRUPT_PRIO 5
#define CHECK_BUS_PRIO 1

/* task periods */
#define CHECK_CAN_BUS_PERIOD pdMS_TO_TICKS(3000)


/*************** GLOBAL VARIABLES ***************/
uint8_t reset = CAN_NO_RESET; // variable that can exit the main program. 
esp_err_t err;
TaskHandle_t mainTaskHandle;
can_node_t canNode; 
twai_status_info_t controllerStatus;

/* INTERRUPT TASK FOR RECEIVING MESSAGES AND MESSAGE HANDLING */
void rx_interrupt_task(void *arg){
    while(reset == CAN_NO_RESET){
    if(!twai_receive(&canNode.rxMsg, portMAX_DELAY) == ESP_OK){
        ESP_LOGE("RX_INTERRUPT_TASK", "RX Error");
    } else {
        can_print_rx_message(&canNode.rxMsg);
        can_process_message(&canNode);
    }
    vTaskDelay(1);
    }
}

/* TASK THAT CHECKS THE HEALTH OF THE CAN CONTROLLER */
void check_bus_task(void *arg){
    while(reset== CAN_NO_RESET){
        ESP_ERROR_CHECK(twai_get_status_info(&controllerStatus));

        printf("TX ERROR: %lu, RX ERROR: %lu\n", 
            controllerStatus.tx_error_counter,
            controllerStatus.rx_error_counter);
        vTaskDelay(CHECK_CAN_BUS_PERIOD);
    }
}

/* MAIN TASK THAT CONTAINS ALL OTHER SUBTASKS */
void main_task(void *arg){
    can_od_t* OD = initOD(10);
    insertObject(OD, 0x1018, 0x0, UNSIGNED8, READ_ONLY, VOLATILE, 5);
    insertObject(OD, 0x1018, 0x1, UNSIGNED32, READ_ONLY, VOLATILE, 123456);
    insertObject(OD, 0x1018, 0x2, UNSIGNED32, READ_ONLY, VOLATILE, 1602109);
    insertObject(OD, 0x1018, 0x3, UNSIGNED32, READ_ONLY, VOLATILE, 0);
    insertObject(OD, 0x1018, 0x4, UNSIGNED32, READ_ONLY, VOLATILE, 1);
    
    esp_task_wdt_add(mainTaskHandle);
    
    can_config_module(); // configuration in CANopen.h
    can_start_module();

    /* Initialize CANopen node */
    can_node_init(&canNode);
    canNode.OD = OD; // adds the priorly generated OD to node
    
    ESP_LOGI("MAIN_TASK", "Node configured with node id %u", canNode.id);

    /* Create subtasks */
    xTaskCreate(rx_interrupt_task, "rx_interrupt_task", 4096, NULL, RX_INTERRUPT_PRIO, NULL);
    xTaskCreate(check_bus_task, "check_bus_task", 2048, NULL, CHECK_BUS_PRIO, NULL);
    while(reset == CAN_NO_RESET){
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Exiting program */
    ESP_LOGI("MAIN_TASK", "Exiting main task");
    can_stop_module();
    can_uninstall_module();
    freeOD(OD);
    vTaskDelete(NULL);
    esp_restart();

}

/* MAIN ENTRY POINT AFTER START */
void app_main()
{
    xTaskCreate(main_task, "main_task", 4096, NULL, MAIN_TASK_PRIO, NULL);
}