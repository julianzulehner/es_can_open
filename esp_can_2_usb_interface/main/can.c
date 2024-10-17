#include "can.h"

/* Prints a can tx message to the standard output*/
void can_print_tx_message(twai_message_t *msg){
    printf("TX MESSAGE ID: %lX, DLC: %d, DATA: [0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X]\n", 
    msg->identifier,
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

/* Prints a can tx message to the standard output*/
void can_print_rx_message(twai_message_t *msg){
    printf("RX MESSAGE ID: %lX, DLC: %d, DATA: [0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X]\n", 
    msg->identifier,
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

/* Transmits and prints the message to stdout*/
void can_transmit(twai_message_t *msg){
    twai_transmit(msg, TX_TIMEOUT);
    can_print_tx_message(msg);
};


/* Config the CAN module */
void can_config_module(void){
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 50;
    g_config.tx_queue_len = 50;
    twai_timing_config_t t_config = CAN_BAUD_RATE;
    twai_filter_config_t f_config = {
        .acceptance_mask = CAN_ACCEPTANCE_MASK,
        .acceptance_code = CAN_ACCEPTANCE_CODE,
        .single_filter = CAN_SINGLE_FILTER
    };
    

    /* This should improve number of error frames on install */
    gpio_set_direction(CAN_TX, GPIO_MODE_OUTPUT);
    gpio_set_level(CAN_TX, 1);
    gpio_set_direction(CAN_RX, GPIO_MODE_OUTPUT);
    gpio_set_level(CAN_RX, 1);
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
    printf("INFO: Starting CAN module... \n");
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

/* Get*/
uint8_t extract_uint8(twai_message_t *msg, uint8_t startbyte){
    uint8_t value = 0;
    value = msg->data[startbyte];
    return value;
}

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

uint64_t extract_uint64(twai_message_t *msg, uint8_t startbyte){
    uint64_t value = 0;
    for(uint8_t i = 0; i < 8; i++){
            value |= (msg->data[startbyte+i] << (8 * i));
    }
    return value;
}

void insert_uint8(twai_message_t *msg,  uint8_t startbyte, uint32_t* value){
    if(value == NULL){
        printf("ERROR: Unsupported object cannot be changed\n");
        return;

        uint8_t byte = *(uint32_t*)value & 0xFF;
        msg->data[startbyte] = byte;
    }
}

void insert_uint16(twai_message_t *msg,  uint8_t startbyte, uint32_t* value){
    if(value == NULL){
        printf("ERROR: Unsupported object cannot be changed\n");
        return;
    }
    for(uint8_t i=0; i<2; i++){
        uint8_t byte = (*(uint32_t*)value >> (i * 8)) & 0xFF;
        msg->data[startbyte+i] = byte;
    }
}

void insert_uint24(twai_message_t *msg,  uint8_t startbyte, uint32_t* value){
    if(value == NULL){
        printf("ERROR: Unsupported object cannot be changed\n");
        return;
    }
    for(uint8_t i=0; i<3; i++){
        uint8_t byte = (*(uint32_t*)value >> (i * 8)) & 0xFF;
        msg->data[startbyte+i] = byte;
    }
}

void insert_uint32(twai_message_t *msg,  uint8_t startbyte, uint32_t* value){
    if(value == NULL){
        printf("ERROR: Unsupported object cannot be changed\n");
        return;
    }
    for(uint8_t i=0; i<4; i++){
        uint8_t byte = (*(uint32_t*)value >> (i * 8)) & 0xFF;
        msg->data[startbyte+i] = byte;
    }
}

    
