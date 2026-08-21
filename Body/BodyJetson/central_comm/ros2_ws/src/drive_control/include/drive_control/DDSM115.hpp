#ifndef DDSM_115_H
#define DDSM_115_H

#define MODE_CURRENT_LOOP   0x01;
#define MODE_SPEED_LOOP     0x02;
#define MODE_POSITION_LOOP  0x03;


#define MODE_FEEDBACK       0x74;


class DDSM115
{
    puublic:
        uint8_t id;
        int8_t data[9];
        void setMotorId(uint8_t);
        void 
    private:
        int16_t _mVelocity;
        int16_t _mCurrent;
        int16_t _mGivenPosition;

        void _writeVelocityInData();

        
}



#endif