#ifndef MATRIX_H
#define MATRIX_H

class Matrix {
    public:
        Matrix();
        void setLeds(bool frame[4][26]);
    private:
        int arr[30] = {1};
        void writeToRegister(const int bits[8]);
};

#endif