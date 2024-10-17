/*
 * Copyright (c) 2024 Your Julian Zulehner
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for more information.
 */

#include <stdio.h>
#include "driver/twai.h"
#include "esp_log.h"

#ifndef CAN_H
#define CAN_H

/* CONFIGURATION PART - CHANGE VALUES OF THIS SECTION */
#define CAN_TX GPIO_NUM_0
#define CAN_RX GPIO_NUM_2
#define CAN_NODE_ID 127
#define CAN_BAUD_RATE TWAI_TIMING_CONFIG_125KBITS()
#define CAN_ACCEPTANCE_MASK 0xFFFFFFFF
#define CAN_ACCEPTANCE_CODE 0
#define CAN_SINGLE_FILTER true
#define TX_TIMEOUT pdMS_TO_TICKS(500)



/* Prints a can rx message to the standard output*/
void can_print_rx_message(twai_message_t *msg);

/* Prints a can tx message to the standard output*/
void can_print_tx_message(twai_message_t *msg);

/* Config the CAN module */
void can_config_module(void);

/* Starts the CAN module */
void can_start_module(void);

/* Transmits message and prints it to stdout */
void can_transmit(twai_message_t *msg);

/* Starts periodic message */
//void can_transmit_periodic(void handle, twai_message_t *msg, int intervall);

/* Stops periodic message */
//void can_periodic_stop(void handle);   

/* Stops the can module */
void can_stop_module(void);

/* Uninstalls module and frees memory */
void can_uninstall_module(void);

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

/* Processes the incoming message and sends response if needed */
void can_process_message(twai_message_t *msg);

#endif
