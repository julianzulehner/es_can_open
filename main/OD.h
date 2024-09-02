#ifndef OD_H
#define OD_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* 
Available data types (list not extensive but sufficient form my use).
When adding new data types, be sure to also change the switch blocks in 
functions allocateValue and writeValue.
*/
typedef enum {
    BOOLEAN     = 0x1,
    INTEGER8    = 0x2,
    INTEGER16   = 0x3,
    INTEGER32   = 0x4,
    UNSIGNED8   = 0x5,
    UNSIGNED16  = 0x6,
    UNSIGNED32  = 0x7,
} CAN_DATA_TYPE;

/* 
Acces types for OD objects (constant can be changed during initialisation)
*/
typedef enum {
    READ_WRITE,  
    WRITE_ONLY,
    READ_ONLY,
    CONST,
} OD_DATA_ACCESS;

/* Options for the persistence of OD objects*/
typedef enum {
    PERSISTENT,
    VOLATILE,
} OD_DATA_PERSISTENCE;

/* Object dictionary object */
typedef struct {
    uint16_t index;
    uint8_t subindex;
    uint16_t dataType;
    uint8_t access;
    uint8_t persistence;
    void* value;
} can_od_object_t;

/* Structure that holds the full object dictionary */
typedef struct{
    can_od_object_t* odObjects;
    int16_t numberObjects;
} can_od_t;

/* 
Allocates the required memory for the object 
dictionary and returns pointer 
*/
can_od_t* initOD(int numberObjects);

/* Retuns the pointer of the defined OD object */
void* getODValue(can_od_t* OD, uint16_t index, uint16_t subindex);

/* Change the value of an object dictionary object */
void setODValue(can_od_t* OD, uint16_t index, uint16_t subindex, int value);

/* Frees up the allocated memory of the full object dictionary */
void freeOD(can_od_t* OD);

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
    int value);

/* DELETE AGAIN */
uint16_t* getSupportdedODobjects(char* filename);
uint16_t getNumberOfODObjects(char* filename);

#endif