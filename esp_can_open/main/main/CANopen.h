#include <stdio.h>
#include "driver/twai.h"
#include "esp_log.h"
#include "OD.h"

#ifndef CAN_NODE_H
#define CAN_NODE_H

/* CONFIGURATION PART - CHANGE VALUES OF THIS SECTION */
#define CAN_TX GPIO_NUM_0
#define CAN_RX GPIO_NUM_2
#define CAN_NODE_ID 127
#define CAN_BAUD_RATE TWAI_TIMING_CONFIG_125KBITS()
#define CAN_MSG_FILTER TWAI_FILTER_CONFIG_ACCEPT_ALL()
#define TX_TIMEOUT pdMS_TO_TICKS(500)
#define CAN_NODEGUARD true 
#define CAN_NODEGUARD_TOGGLE_BIT 7

/* GENERAL CONFIGURATION - DON'T CHANGE THE VALUES OF THIS SECTION*/
#define CO_VERSION_MAJOR 5
#define CAN_NO_RESET 0
#define CAN_RESET 1

#define CAN_NMT_ID 0x0
#define CAN_HB_ID 0x700
#define CAN_SYNC 0x80
#define CAN_RX_SDO 0x600
#define CAN_TX_SDO 0x580

#define SDO_CSS_MASK            0b11100000
#define SDO_N_MASK              0b00001100
#define SDO_E_MASK              0b00000010
#define SDO_S_MASK              0b00000001

#define SDO_INIT_EXP_DOWNLOAD   0x20
#define SDO_INIT_SEG_DOWNLOAD   0x21
#define SDO_INIT_EXP_UPLOAD     0x40
#define SDO_INIT_SEG_UPLOAD     0x41
#define SDO_DOWNLOAD_SEGMENT    0x60
#define SDO_UPLOAD_SEGMENT      0x70
#define SDO_SAVE_SIGNATURE      0x65766173

#define SDO_DOWNLOAD_SUCCESS    0x60
#define SDO_ABORT               0x80
#define SDO_UPLOAD_4_BYTES      0x43
#define SDO_UPLOAD_3_BYTES      0x47
#define SDO_UPLOAD_2_BYTES      0x4B
#define SDO_UPLOAD_1_BYTE       0x4F

#define SDO_N_4_BYTES           0x04
#define SDO_N_3_BYTES           0x08
#define SDO_N_2_BYTES           0x0C
#define SDO_N_1_BYTE            0x10
#define SDO_EXP_TRANSFER        0x02
#define SDO_DATA_SIZE_INDICATED 0x01

#define LSS_RX_COB_ID              0x7E5
#define LSS_TX_COB_ID              0x7E4
#define LSS_CS_MODE_CONFIG         0x1
#define LSS_CS_MODE_OPERATION      0x0
#define LSS_CS_SWITCH_MODE_GLOBAL  0x4
#define LSS_CS_SELECTIVE_VENDOR    64
#define LSS_CS_SELECTIVE_PRODUCT   65
#define LSS_CS_SELECTIVE_REVISION  66
#define LSS_CS_SELECTIVE_SERIAL    67
#define LSS_CS_SELCTIVE_RESPONSE   68
#define LSS_CS_CONFIG_NODE         17
#define LSS_ERR_ID_OUT_OF_RANGE    1
#define LSS_NODE_SUCCESS           0
#define LSS_ERR_SPECIFIC           255
#define LSS_CS_STORE_CONFIG       23
#define LSS_STORE_SUCCESS          0
#define LSS_STORE_NOT_SUPPORTED    1
#define LSS_STORE_ACCESS_ERROR     2
#define LSS_OPERATION_MODE         0b0000
#define LSS_CONFIG_MODE            0b1111


#define OBJ_COB_ID_SYNC 0x1005
#define OBJ_STORE_PARAM 0x1010
#define OBJ_CONSUMER_HB_TIME 0x1016
#define OBJ_PRODUCER_HB_TIME 0x1017
#define OBJ_IDENDITY 0x1018
#define OBJ_TPDO1_MAP_PARAM 0x1A00
#define OBJ_TPDO1_COMM_PARAM 0x1800
#define OBJ_MPL_VALUE 0x2011
#define OBJ_PHY_UNIT 0x6131
#define OBJ_NR_DECIMALS 0x6132
#define OBJ_DIELECTRIC_CONST 0x7100

/* Possible NMT states of a CAN node */
typedef enum {
    CAN_NMT_INITIALIZING = 0x0,
    CAN_NMT_STOPPED = 0x4,
    CAN_NMT_OPERATIONAL = 0x5,
    CAN_NMT_PRE_OPERATIONAL = 0x7f,
} can_nmt_state; 

/* Possible NMT commands from NMT master */
typedef enum {
    CAN_NMT_CMD_OPERATIONAL = 0x1,
    CAN_NMT_CMD_STOP = 0x2,
    CAN_NMT_CMD_PRE_OPERATIONAL = 0x80,
    CAN_NMT_CMD_RESET_NODE = 0x81,
    CAN_NMT_CMD_RESET_COMMUNICATION = 0x82,
} can_nmt_cmd;

/* CAN Node structure that contains all the variables to handle */
typedef struct {
    uint8_t id;
    can_nmt_state nmtState;
    twai_message_t rxMsg;
    twai_message_t txMsg;
    can_od_t* OD;
    uint8_t lssMode;
} can_node_t;


/* Prints a can rx message to the standard output*/
void can_print_rx_message(twai_message_t *msg);

/* Prints a can tx message to the standard output*/
void can_print_tx_message(twai_message_t *msg);

/* Config the CAN module */
void can_config_module(void);

/* Starts the CAN module */
void can_start_module(void);

/* Stops the can module */
void can_stop_module(void);

/* Uninstalls module and frees memory */
void can_uninstall_module(void);

/* Initializes CAN node and sets NMT state to pre-operational */
void can_node_init(can_node_t *node);

/* Extracts uint8_t from msg data */
uint8_t extract_uint8(twai_message_t *msg, uint8_t startbyte);

/* Extracts uint16_t from msg data in byteorder little */
uint16_t extract_uint16(twai_message_t *msg, uint8_t startbyte);

/* Extracts uint32_t from msg data in byteorder little */
uint32_t extract_uint32(twai_message_t *msg, uint8_t startbyte);

/* Inserts uint32_t into data frame of message in byteorder little endian */
void insert_uint32(twai_message_t *msg,  uint8_t startbyte, uint32_t* value);

/* Extracts uint64_t from msg data in byteorder little */
uint64_t extract_uint64(twai_message_t *msg, uint8_t startbyte);

/* Sends response after chaning NMT state with current state */
void send_nmt_state(can_node_t *node);

/* Processes the incoming message and sends response if needed */
void can_process_message(can_node_t *node);

/* SDO service to handle SDO frames */
void sdo_service(can_node_t * node);

/* Sends data of current sdo object */
void send_sdo_object(can_node_t *node);

/* 
Stores the current configuration to non volatile stoarge
*/
void store_OD_persistent(can_node_t *node, nvs_handle_t *nvsHandle);

/* Loads the values from the persistent data and stores it to volatile */
void load_OD_persistent(can_node_t *node, nvs_handle_t *nvsHandle);

/* Initializes the non volatile storage of esp */
void init_nvs(can_node_t *node, nvs_handle_t *nvsHandle);

/* Updates the node id according to the NVS value */
void update_node_id(can_node_t *node);
#endif