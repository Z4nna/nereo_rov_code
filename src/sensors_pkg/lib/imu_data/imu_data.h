#ifndef IMUDATA_H
#define IMUDATA_H

#include <cstdint>

class ImuData
{
private:
    int32_t Acc[3];
    int32_t Angle[3];
    int32_t AngleVel[3];
    int32_t Temp[3];
};

#endif // IMUDATA_H