#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "OD.h"

#define SIZE 50

int main(){
    int hashValue;
    hashValue = hash(0x1018, 0x00, SIZE);
    printf("%u\n", hashValue);
    hashValue = hash(0x1018, 0x01, SIZE);
    printf("%u\n", hashValue);
    hashValue = hash(0x1018, 0x02, SIZE);
    printf("%u\n", hashValue);
    hashValue = hash(0x1018, 0x03, SIZE);
    printf("%u\n", hashValue);
    hashValue = hash(0x1018, 0x04, SIZE);
    printf("%u\n", hashValue);
    hashValue = hash(0x2011, 0x0, SIZE);
    printf("%u\n", hashValue);
    hashValue = hash(0x2011, 0x1, SIZE);
    printf("%u\n", hashValue);
    hashValue = hash(0x1800, 0x0, SIZE);
    printf("%u\n", hashValue);
    hashValue = hash(0x1800, 0x01, SIZE);
    printf("%u\n", hashValue);

    return 0;
}