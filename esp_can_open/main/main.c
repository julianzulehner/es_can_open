/*
 * Copyright (c) 2024 Your Julian Zulehner
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for more information.
 */

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
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/uart.h"
#include "esp_timer.h"

/*************** DEFINITIONS ***************/

/* task priorities*/
#define MAIN_TASK_PRIO 1
#define RX_INTERRUPT_PRIO 5
#define CHECK_BUS_PRIO 1


/* task periods */
#define CHECK_CAN_BUS_PERIOD pdMS_TO_TICKS(3000)

/* OD settings */
#define NR_OBJECTS 117 // this is randomly selected to and has no hash collisions
#define NR_PERSISTENT_OBJECTS 9 

/* UART CONFIGURATION */
#define UART_TX GPIO_NUM_6
#define UART_RX GPIO_NUM_5
#define UART_BUFFER_SIZE (1024)
#define UART_BAUDRATE 115200
#define UART_DELAY 10000000 // 10 seconds in us

/* Structure definitions*/
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
TaskHandle_t rxUARTinterruptTaskHandle;
can_node_t canNode; 
twai_status_info_t controllerStatus;
nvs_handle_t nvsHandle;
sensor_data_t sensorData;
uart_port_t uartPort = UART_NUM_1;
QueueHandle_t uartQueueHandle;

/* INTERRUPT TASK FOR RECEIVING CAN MESSAGES AND MESSAGE HANDLING */
void rx_interrupt_task(void *arg){
    while(1){
    if(!twai_receive(&canNode.rxMsg, portMAX_DELAY) == ESP_OK){
        printf("ERROR: CAN RX Error\n");
    } else {
        can_print_rx_message(&canNode.rxMsg);
        can_process_message(&canNode);
    }
    vTaskDelay(1);
    }
}

/*  Prints the sensor data coming via UART from OPS3 sensor */
void print_sensor_data(sensor_data_t* data){
    printf("Viscosity:    %lu\n", data->viscosity);
    printf("Density:    %lu\n", data->density);
    printf("Dielectric:    %lu\n", data->dielectric);
    printf("Resistance:    %lu\n", data->resistance);
    printf("Temperature:    %lu\n", data->temperature);
    printf("Diagnostics:    0x%lX\n", data->diagnostics);
}


/* INTERRUPT TASK FOR RECEIVING UART MESSAGES FROM OPS3 SENSOR */
void rx_uart_interrupt_task(void *arg){
    while(1){
        printf("UART receive task start... \n");
        uint8_t buffer[sizeof(sensor_data_t)];
        int len = uart_read_bytes(uartPort, buffer, 
                    sizeof(sensor_data_t), portMAX_DELAY);
        if (len == sizeof(sensor_data_t)){
            memcpy(&sensorData, buffer, sizeof(sensor_data_t));
            printf("Dielectric: %lu\n", sensorData.dielectric);

            /* Write new values to OD objects */
            can_od_object_t* dielectricObject = getODentry(canNode.OD, 
                OD_PV_DIELECTRIC, 0);
            *dielectricObject->value = sensorData.dielectric;
        } else {
            printf("ERROR: uart_read_bytes failed or incomplete data received\n");
        }
        
        // uart_receive_sensor_data(NULL);
        // vTaskDelay(pdMS_TO_TICKS(25000));
        printf("UART receive task end. \n");
        vTaskDelay(pdMS_TO_TICKS(25000));

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

/* Writes 0 values to the full data structur sensor_data_t */
void init_sensor_data(sensor_data_t* data){
    data->density = 0;
    data->diagnostics = 0;
    data->dielectric = 0;
    data->resistance = 0;
    data->temperature = 0;
    data->viscosity = 0;
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

/*
Configures all the OD objects. Change the values here if the FW should have
different values by default. The values are volatile here because they are 
always loaded from the bootloader at startup and not from EEPROM 
*/
void load_OD(){
    can_od_t* OD = canNode.OD;
    /* DEVICE TYPE */
    insertObject(OD, OD_DEVICE_TYPE, 0x0, UNSIGNED32, READ_ONLY, VOLATILE, 1 << 17);

    /* ERROR REGISTER */
    insertObject(OD, OD_ERROR_REGISTER, 0x0, UNSIGNED8, READ_ONLY, VOLATILE, 0);

    /* STORE PARAMETERS */
    insertObject(OD, OD_STORE_PARAMETERS, 0x0, UNSIGNED8, CONST, VOLATILE, 1);
    insertObject(OD, OD_STORE_PARAMETERS, 0x1, UNSIGNED32, READ_WRITE, VOLATILE, 1);

    /* IDENTITY OBJECT */
    insertObject(OD, OD_IDENTITY_OBJECT, 0x0, UNSIGNED8, READ_ONLY, VOLATILE, 5);
    insertObject(OD, OD_IDENTITY_OBJECT, 0x1, UNSIGNED32, READ_ONLY, VOLATILE, 1280);
    insertObject(OD, OD_IDENTITY_OBJECT, 0x2, UNSIGNED32, READ_ONLY, VOLATILE, 1625810019);
    insertObject(OD, OD_IDENTITY_OBJECT, 0x3, UNSIGNED32, READ_ONLY, VOLATILE, 808517632);
    insertObject(OD, OD_IDENTITY_OBJECT, 0x4, UNSIGNED32, READ_ONLY, VOLATILE, 1);

    /* MPL values */
    insertObject(OD, OD_MPL_VALUE_INDEX, 0x0, UNSIGNED8, READ_ONLY, VOLATILE, 1);
    insertObject(OD, OD_MPL_VALUE_INDEX, 0x1, UNSIGNED16, READ_WRITE, VOLATILE, 0);

    /* Node ID */
    insertObject(OD, OD_NODE_ID, 0x0, UNSIGNED8, READ_WRITE, PERSISTENT, CAN_NODE_ID);

    /* TPDO 1 Communication Parameter*/
    insertObject(OD, OD_TPDO1_PARAMETER, 0x0, UNSIGNED8, READ_WRITE, PERSISTENT, 2);
    insertObject(OD, OD_TPDO1_PARAMETER, 0x1, UNSIGNED16, READ_WRITE, PERSISTENT, 0x180+canNode.id);
    insertObject(OD, OD_TPDO1_PARAMETER, 0x2, UNSIGNED8, READ_WRITE, PERSISTENT, 8);

    /* TPDO 2 Communication Parameter*/
    insertObject(OD, OD_TPDO2_PARAMETER, 0x0, UNSIGNED8, READ_WRITE, PERSISTENT, 0);

    /* TPDO 3 Communication Parameter*/
    insertObject(OD, OD_TPDO3_PARAMETER, 0x0, UNSIGNED8, READ_WRITE, PERSISTENT, 0);

    /* TPDO 4 Communication Parameter*/
    insertObject(OD, OD_TPDO4_PARAMETER, 0x0, UNSIGNED8, READ_WRITE, PERSISTENT, 0);

    /* TPDO 1 MAPPING */
    insertObject(OD, OD_TPDO1_MAPPING, 0x0, UNSIGNED8, CONST, VOLATILE, 3);
    insertObject(OD, OD_TPDO1_MAPPING, 0x1, UNSIGNED32, READ_WRITE, PERSISTENT, 
        OD_PV_DIELECTRIC << 16 | 0x0 << 8 | 0x4);
    insertObject(OD, OD_TPDO1_MAPPING, 0x2, UNSIGNED32, READ_WRITE, PERSISTENT,
        OD_MPL_VALUE_INDEX << 16 | 0x1 << 8 | 0x2);
    insertObject(OD, OD_TPDO1_MAPPING, 0x3, UNSIGNED32, READ_WRITE, PERSISTENT,
        OD_ERROR_REGISTER << 16 | 0x0 << 8 | 0x1);
    
    /* Physical unit */
    insertObject(OD, 0x6131, 0x0, UNSIGNED8, READ_ONLY, VOLATILE, 1);
    insertObject(OD, 0x6131, 0x1, UNSIGNED32, READ_ONLY, VOLATILE, 0);

    /* Number of Decimal Positions */
    insertObject(OD, 0x6132, 0x0, UNSIGNED8, READ_ONLY, VOLATILE, 1);
    insertObject(OD, 0x6132, 0x1, UNSIGNED8, READ_ONLY, VOLATILE, 5);

    /* Process Values 0x7100 = dielectric constant */
    insertObject(OD, OD_PV_DIELECTRIC, 0x0, UNSIGNED16, READ_ONLY, VOLATILE, 65535);
}

/* MAIN TASK THAT CONTAINS ALL OTHER SUBTASKS */
void main_task(void *arg){
    can_od_t* OD = initOD(NR_OBJECTS, NR_PERSISTENT_OBJECTS);
    esp_task_wdt_add(mainTaskHandle);
    
    can_config_module(); // configuration in CANopen.h
    can_start_module();

    /* Initialize UART communication */
    init_uart(uartPort);

    /* Initialize CANopen node */
    can_node_init(&canNode);
    canNode.OD = OD; // adds the priorly generated OD to node
    /* Loads and writes all the objects of the OD to structure ODv */
    init_nvs(&canNode, &nvsHandle);
    load_OD();
    load_OD_persistent(&canNode, &nvsHandle);
    update_node_id(&canNode); 
    printf("INFO: Node configured with node id %u\n", canNode.id);

    /* Send updated NMT status*/
    if(canNode.nmtState == CAN_NMT_PRE_OPERATIONAL){
        send_nmt_state(&canNode);
    }
    
    /* Create subtasks */
    xTaskCreate(rx_interrupt_task, "rx_interrupt_task", 4096, NULL, 
                RX_INTERRUPT_PRIO, &checkBusTaskHandle);
    xTaskCreate(check_bus_task, "check_bus_task", 2048, NULL, 
                CHECK_BUS_PRIO, &rxInterruptTaskHandle);
    xTaskCreate(rx_uart_interrupt_task, "rx_uart_interrupt_task", 4096, NULL,
                RX_INTERRUPT_PRIO, &rxUARTinterruptTaskHandle);
    while(reset == CAN_NO_RESET){
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Exiting program */
    printf("INFO: Exiting main task\n");
    vTaskDelete(rxInterruptTaskHandle);
    vTaskDelete(checkBusTaskHandle);
    vTaskDelete(rxUARTinterruptTaskHandle);
    can_stop_module();
    can_uninstall_module();
    uart_driver_delete(uartPort);
    freeOD(OD);
    nvs_close(nvsHandle);
    esp_restart();

}

/* MAIN ENTRY POINT AFTER START */
void app_main()
{
    xTaskCreate(main_task, "main_task", 4096, NULL, MAIN_TASK_PRIO, &mainTaskHandle);
}