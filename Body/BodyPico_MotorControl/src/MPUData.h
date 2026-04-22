#ifndef MPUDATA_H
#define MPUDATA_H

#include <Arduino.h>

struct IMUData
{
    int16_t accel_x = 0;
    int16_t accel_y = 0;
    int16_t accel_z = 0;
    uint64_t t = 0;
};

class MPUData {
public:
    MPUData();
    void setData(int16_t accel_x, int16_t accel_y, int16_t accel_z, uint64_t t);
    uint64_t getAge() const;

    IMUData data;

private:
    uint64_t last_update;
};

#endif