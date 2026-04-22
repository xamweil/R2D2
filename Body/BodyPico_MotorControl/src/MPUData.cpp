#include "MPUData.h"

MPUData::MPUData()
    : last_update(0)
{}

void MPUData::setData(int16_t accel_x, int16_t accel_y, int16_t accel_z, uint64_t t) {
    data.accel_x = accel_x;
    data.accel_y = accel_y;
    data.accel_z = accel_z;
    data.t = t;
    last_update = millis();
}

uint64_t MPUData::getAge() const {
    return millis() - last_update;
}