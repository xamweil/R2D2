#ifndef MATRIX_H
#define MATRIX_H

#include <cstdint>
class Matrix {
    public:
        Matrix();
        void randomLights(uint8_t ratOn=70, uint8_t ratChange=10, uint8_t dt = 150);
        void displayOff();
        void displayOn();
        void setLeds(bool frame[4][26]);
    private:
        int dt = 300;
        int arr[30] = {1};
        void writeToRegister(const int bits[8]);
        void resetArr();
};

#endif