#include "driver/twai.h"

#ifndef LAWICEL_EMULATOR_H
#define LAWICEL_EMULATOR_H

#define CMD_CLOSE_CHANNEL "C"

#define ID_STD_FRAME "t"
#define ID_EXT_FRAME "T"

#define ID_EXT_MAX 0x1FFFFFFF
#define ID_STD_MAX 0x7FF

#define STD_DATA_START 5
#define EXT_DATA_START 10

void twai_message_to_serial(twai_message_t * msg);

#endif