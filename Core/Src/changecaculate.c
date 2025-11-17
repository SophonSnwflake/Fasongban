#include "changecaculate.h"


float caculateWheelSpeed(float r,float Origin,float Mutiple)
{
    return (Origin - 128) * Mutiple * r;
}

float caculateWheelAngleSpeed(float Origin, float Mutiple)
{
    
    return  (Origin - 128) * Mutiple ;
}