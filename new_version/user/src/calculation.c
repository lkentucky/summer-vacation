#include "calculation.h"


uint8 uint8_limit(int16 value, uint8 min, uint8 max)
{
    if (value < min)
    {
        return min;
    }
    else if (value > max)
    {
        return max;
    }
    else
    {
        return (uint8)value;
    }
}

//绝对值函数
float float_abs(float value)
{
    return (value >= 0) ? value : -value;
}
