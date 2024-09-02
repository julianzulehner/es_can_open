#ifndef OD_H
#define OD_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define OD_DEVICE_TYPE          0x1000
#define OD_ERROR_REGISTER       0x1001
#define OD_IDENTITY_OBJECT      0x1018
#define OD_TPDO1_PARAMETER      0x1800
#define OD_TPDO2_PARAMETER      0x1801
#define OD_TPDO3_PARAMETER      0x1802
#define OD_TPDO4_PARAMETER      0x1803
#define OD_TPDO1_MAPPING        0x1A00
#define OD_TPDO2_MAPPING        0x1A01
#define OD_TPDO3_MAPPING        0x1A02
#define OD_TPDO4_MAPPING        0x1A03
#define OD_MPL_VALUE_INDEX      0x2011
#define OD_NODE_ID              0x2012
#define OD_PYH_UNIT             0x6131
#define OD_DEC_POSITIONS        0x6132
#define OD_PV_DIELECTRIC        0x7100

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
    uint32_t* value;
} can_od_object_t;

/* Structure that holds the full object dictionary */
typedef struct{
    can_od_object_t* odObjects;
    uint16_t numberObjects;
    uint32_t* persistentObjectIds;
    uint16_t numberPersistentObjects;
} can_od_t;

/* 
Allocates the required memory for the object 
dictionary and returns pointer 
*/
can_od_t* initOD(int numberObjects, int numberPersistentObjects);

/* Retuns the pointer of the defined OD object */
can_od_object_t* getODentry(can_od_t* OD, uint16_t index, uint16_t subindex);

/* Change the value of an object dictionary object */
void setODValue(can_od_t* OD, uint16_t index, uint16_t subindex, int value);

/* Frees up the allocated memory of the full object dictionary */
void freeOD(can_od_t* OD);

/* Creates a hash for the OD */
unsigned int hash(uint16_t index, uint8_t subindex, unsigned int table_size);

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
    uint32_t value);

#endif