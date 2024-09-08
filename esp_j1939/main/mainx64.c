#include <stdio.h>
#include <math.h> 
#include <stdint.h>
#define FAC_TEMPERATURE 0.03125
#define OFFSET_TEMPERATURE -273

int main(){
    uint32_t temp = 9376;
    float floatTemp = (float)temp;
    float floatRes = (floatTemp*(float)FAC_TEMPERATURE)+ (float)OFFSET_TEMPERATURE;
    printf("Result: %f\n", floatRes);
    return 0;
}