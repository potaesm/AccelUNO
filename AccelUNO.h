#ifndef AccelUNO_h
#define AccelUNO_h
#include "Arduino.h"
#include "Wire.h"

class AccelUNO {
    public:
        AccelUNO();
        void Recalibrate();
        void SensitiveSet(int sense);
        void MPUAddressSet(int mpu_addr);
        void BeginMPU();
        void PrintMPU();
        void FindMPUAddress();
        int Count();
        int Ax();
        int Ay();
        int Az();
        int MPUTemp();
        int Gx();
        int Gy();
        int Gz();
        bool Detected();
    private:
        int MPU_addr = 0x68; // I2C address of the MPU-6050
        int16_t x, y, z, temp, gx, gy, gz;
        float sensitivity = 16384;
        float ax[20], ay[20], az[20];
        float dx, dy, dz;
        float dx_old, dy_old, dz_old;
        int a;
        int i = 0;
        int j = 0;
        int counter = 0;
        int return_angle = 0;
        long previousMillis = 0;
        bool checkangle = false;
        bool detected = false;
        bool I2CCheck = false;
};
#endif