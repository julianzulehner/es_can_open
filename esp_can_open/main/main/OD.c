/*
 * Copyright (c) 2024 Your Julian Zulehner
 * This file is licensed under the MIT License.
 * See the LICENSE file in the project root for more information.
 */

#include "OD.h"

/* Hash function for numbers */
unsigned int hash(uint16_t index, uint8_t subindex, unsigned int table_size) {
    unsigned int hash_value = index;
    hash_value = hash_value * 31 + subindex; // 31 is a prime number
    return hash_value % table_size;
}

/* 
Allocates the required memory for the object 
dictionary and returns pointer 
*/
can_od_t* initOD(int numberObjects, int numberPersistentObjects){
    can_od_t* OD = (can_od_t*)malloc(sizeof(can_od_t));
    OD->odObjects = (can_od_object_t*)malloc(
        numberObjects * sizeof(can_od_object_t));
    OD->numberObjects = numberObjects;
    OD->persistentObjectIds = (uint32_t*)calloc(
        numberPersistentObjects, sizeof(uint32_t));
    OD->numberPersistentObjects = numberPersistentObjects;
    if((OD->odObjects) == NULL || (OD->persistentObjectIds == NULL)){
        printf("ERROR: Memory allocation of object dictionary failed");
        free(OD);
        return NULL;
    }
    for(int i = 0; i < numberObjects; i++){
        OD->odObjects[i].value = NULL;
    }
    return OD;
}

/* 
Allocates the memory for the value itself. For simplicity it will be a 32 bit
unsigned integer for all objects independent of the value itself.
 */
uint32_t* allocateValue(){
    uint32_t* valuePtr = (uint32_t*)malloc(sizeof(uint32_t));
    return valuePtr;
}

/* Writes the data to the pointer type */
void writeValue(uint32_t* valuePtr, uint16_t dataType, int value) {
    *(int32_t*)valuePtr = (int32_t)value;
}

/* 
Adds an can_od_object_t object to can_od_t 
and stores it to the position of the hash 
*/
void insertObject(
    can_od_t* OD, 
    uint16_t index, 
    uint8_t subindex, 
    uint16_t dataType, 
    uint8_t access,
    uint8_t persistence,
    uint32_t value){

    int key = hash(index, subindex, OD->numberObjects);
    uint32_t* valuePtr = allocateValue();

    if(valuePtr == NULL){
        printf("Error: Object Dictionary value could not be allocated\n");
    }

    if (OD->odObjects[key].value == NULL){
        /* Memory is still unused and value can be written */
        printf("INFO: Setting value of OD object %X.%X.h to: %lu\n", 
            index, subindex, value);

    } else if (OD->odObjects[key].value != NULL &&
               OD->odObjects[key].index != index){
        /* Hash function creates the same hash for two different index values */
        printf("Error: Hash collision. \
                Collision handling not implemented yet.\n");
        return;
    } else if(OD->odObjects[key].value != NULL 
            && OD->odObjects[key].index == index 
            && OD->odObjects[key].subindex == subindex){
        /* Object already exists and will be overwritten */
        printf("Warning: Object %X.%Xh was inserted multiple times. \
            Values will be overwritten.\n",
            index, subindex);
    } 

    writeValue(valuePtr, dataType, value);

    OD->odObjects[key].index = index;
    OD->odObjects[key].subindex = subindex;
    OD->odObjects[key].dataType = dataType;
    OD->odObjects[key].access = access;
    OD->odObjects[key].persistence = persistence;
    OD->odObjects[key].value = valuePtr;

    // Add to list of persistent values if persistent
    if(persistence == PERSISTENT){
        for(int i=0; i < OD->numberPersistentObjects; i++){
            if(OD->persistentObjectIds[i] == 0){
                OD->persistentObjectIds[i] = index << 8 | subindex;
                break;
            }
                
        }
    }
}

/* Frees up the allocated memory of the full object dictionary */
void freeOD(can_od_t* OD){
    for(int i = 0; i < OD->numberObjects; i++){
        if(OD->odObjects[i].value != NULL){
            free(OD->odObjects[i].value);
        }

    }
    free(OD->persistentObjectIds);
    free(OD->odObjects);
}

/* Retuns the pointer of the defined OD object */
can_od_object_t* getODentry(can_od_t* OD, uint16_t index, uint16_t subindex){
    int key = hash(index, subindex, OD->numberObjects);
    if(OD->odObjects[key].value){
        return &OD->odObjects[key];
    } else { 
        printf("ERROR: Object %X.%X not supported\n",index, subindex);
        return NULL;
    } 
}

/* Change the value of an object dictionary object */
void setODValue(can_od_t* OD, uint16_t index, uint16_t subindex, int value){
    int key = hash(index, subindex, OD->numberObjects);
    if(OD->odObjects[key].value == NULL){
        printf("ERROR: OD object %X.%Xh is not supported\n", index, subindex);
    }
    writeValue(OD->odObjects[key].value, OD->odObjects[key].dataType, value);
}




