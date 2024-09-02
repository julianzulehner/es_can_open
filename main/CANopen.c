#include "CANopen.h"

/* Extern global variables */
extern uint8_t reset;

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

/* Config the CAN module */
void can_config_module(void){
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = CAN_BAUD_RATE;
    twai_filter_config_t f_config = CAN_MSG_FILTER;

    // Uninstalled to stopped state
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        ESP_LOGI(CAN_TAG,"CAN module installed");
        }
    else{
        ESP_LOGE(CAN_TAG, "Failed to install CAN module");
    }
}

/* Starts the CAN module */
void can_start_module(void){
    if (twai_start() == ESP_OK) {
        ESP_LOGI(CAN_TAG, "CAN module started");
    }
    else {
        ESP_LOGE(CAN_TAG, "Failed to start CAN module");
        }
}

/* Stops the can module */
void can_stop_module(void){
    if(twai_stop() == ESP_OK){
        ESP_LOGI(CAN_TAG, "CAN module stopped");
    } else {
        ESP_LOGE(CAN_TAG, "Could not stop CAN module");
    }

}

/* Uninstalls module and frees memory */
void can_uninstall_module(void){
    if(twai_driver_uninstall() == ESP_OK){
        ESP_LOGI(CAN_TAG, "CAN Module uninstalled");
    } else {
        ESP_LOGE(CAN_TAG, "CAN module could not be uninstalled");
    }
}

/* Initializes CAN node and sets NMT state to pre-operational */
void can_node_init(can_node_t *node){
    if(node){
        node->id = CAN_NODE_ID;
        node->nmtState = CAN_NMT_PRE_OPERATIONAL;
    } else {
        ESP_LOGE(CAN_TAG, "Invalid node cannot set to pre-operational");
        return;
    };

    /* Send updated NMT status*/
    send_nmt_state(node);
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

/* Sends response after chaning NMT state with current state */
void send_nmt_state(can_node_t *node){

    if(CAN_NODEGUARD && node->rxMsg.identifier == (CAN_HB_ID + node->id)){
        node->nmtState ^= 1 << CAN_NODEGUARD_TOGGLE_BIT;
    }
    node->txMsg.identifier =  (CAN_HB_ID + node->id);
    node->txMsg.data_length_code = 1;
    node->txMsg.data[0] = node->nmtState;


    ESP_ERROR_CHECK(twai_transmit(&node->txMsg, TX_TIMEOUT));
    can_print_tx_message(&node->txMsg);
};

/* Writes the value to the sdo object */
void write_sdo_object(can_node_t *node){};

/* Sets the whole can message data to 0*/
void empty_msg_data(twai_message_t* msg){
    for(int i = 0; i < 8; i++){
        msg->data[i] = 0;
    }
}

/* Sends data of current sdo object */
void send_sdo_object(can_node_t *node){
    empty_msg_data(&node->txMsg);

    /* Prepare right response byte */
    uint8_t sdoResponse = 0;
    uint8_t sdoCommand = node->rxMsg.data[0];
    uint8_t numberEmptyBytes = (sdoCommand & SDO_N_MASK) >> 2;
    printf("%u\n", numberEmptyBytes);
    switch(numberEmptyBytes){
        case 0:
            sdoResponse = SDO_UPLOAD_4_BYTES;
            break;
        case 1:
            sdoResponse = SDO_UPLOAD_3_BYTES;
            break;
        case 2: 
            sdoResponse = SDO_UPLOAD_2_BYTES;
            break;
        case 3:
            sdoResponse = SDO_UPLOAD_1_BYTE;
    }
    if (numberEmptyBytes > 0){
        if((sdoCommand & SDO_S_MASK) == 0 || (sdoCommand & SDO_E_MASK) == 0){
            sdoResponse = SDO_ABORT;
           }
    }

    node->txMsg.identifier = CAN_TX_SDO + node->id;
    node->txMsg.data_length_code = 8;

    /* Insert index and subindex into response*/
    
    node->txMsg.data[1] = node->rxMsg.data[1];
    node->txMsg.data[2] = node->rxMsg.data[2];
    node->txMsg.data[3] = node->rxMsg.data[3];


    uint16_t index = extract_uint16(&node->rxMsg, 1);
    uint8_t subindex = extract_uint8(&node->rxMsg, 3);
    void* value = getODValue(node->OD, index, subindex);
    if(value==NULL){
        /* Object not existing */
        sdoResponse = SDO_ABORT;
        // TODO: Add also check for access of object
    } else {
        /* This is not 100% correct  as it should adapt to the requested bytes */
        insert_uint32(&node->txMsg, 4, (uint32_t*)value);
    }
    node->txMsg.data[0] = sdoResponse;
    ESP_ERROR_CHECK(twai_transmit(&node->txMsg, TX_TIMEOUT));
    can_print_tx_message(&node->txMsg);
};

/* SDO service to handle SDO frames */
void sdo_service(can_node_t *node){
    uint8_t sdoCommand = node->rxMsg.data[0];
    if((sdoCommand & SDO_CSS_MASK) == SDO_INIT_EXP_UPLOAD){
        send_sdo_object(node);
    } else if((sdoCommand & SDO_CSS_MASK) == SDO_INIT_EXP_DOWNLOAD)
        write_sdo_object(node);
}

/* Processes the incoming message and sends response if needed */
void can_process_message(can_node_t *node){
    uint32_t identifier = node->rxMsg.identifier;
    if (identifier == CAN_NMT_ID && node->rxMsg.data[1] == node->id){
        switch(node->rxMsg.data[0]){
            case CAN_NMT_CMD_OPERATIONAL:
                node->nmtState = CAN_NMT_OPERATIONAL;
                break;
            case CAN_NMT_CMD_PRE_OPERATIONAL:
                node->nmtState = CAN_NMT_PRE_OPERATIONAL;
                break;
            case CAN_NMT_CMD_RESET_COMMUNICATION:
                // TODO: implement case
                break;
            case CAN_NMT_CMD_RESET_NODE:
                reset = CAN_RESET;
                break;
            case CAN_NMT_CMD_STOP:
                // TODO: implement case
                break;
            default:
                return;
        }
        send_nmt_state(node);
        return;
    };
    if (identifier == (CAN_HB_ID + node->id)){
        /* HEARTBEAT PROTOCOL*/
        send_nmt_state(node);
        return;
    }; 
    if (identifier == (CAN_RX_SDO + node->id)){
        sdo_service(node);
    }
}

