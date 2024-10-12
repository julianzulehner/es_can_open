#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lawicel_emulator.h"

void twai_message_to_serial(twai_message_t * msg){
    char tempChar[3];
    char idStr[9];
    char retStr[20];
    
    if(msg->extd == true)
    {
        strcpy(retStr, "T");
        sprintf(idStr, "%08lX", (uint32_t)(msg->identifier));
        strcat(retStr, idStr); 
    } 
    else
    {
        strcpy(retStr, "t");
        sprintf(idStr, "%03X", (uint16_t)(msg->identifier));
        strcat(retStr, idStr); 
    }
    for(uint16_t i=0; i < msg->data_length_code; i++)
        {
            sprintf(tempChar, "%02X", msg->data[i]);
            strcat(retStr, tempChar);
        }
        printf("%s\n", retStr);
}

