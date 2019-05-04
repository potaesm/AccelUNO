#include "Arduino.h"
#include "AccelUNO.h"

AccelUNO::AccelUNO() {
    
}

void AccelUNO::BeginMPU() {
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
}

void AccelUNO::SensitiveSet(int sense) {
    sensitivity = sense;
}

void AccelUNO::MPUAddressSet(int mpu_addr) {
    MPU_addr = mpu_addr;
}

void AccelUNO::FindMPUAddress() {
    if (I2CCheck != true) {
        Wire.begin();
        Serial.println("Scanning ...");
    }
    byte error, address;
    int nDevices = 0;
    for (address = 1; address < 127; address++ )
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C Device Found At Address : 0x");
            if (address < 16)
                Serial.println("0");
            Serial.println(address, HEX);

            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown Error At Address : 0x");
            if (address < 16)
                Serial.println("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C Devices Found");
    else
        Serial.println("Done!");
}

int AccelUNO::Count() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > 100) {
        i++;
        if (i >= 20) {
            i = 1;
        }
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
        x = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
        y = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        z = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
        gx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        gy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        gz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
        ax[i] = x / sensitivity; ay[i] = y / sensitivity; az[i] = z / sensitivity;
        dx = ax[i] - ax[0];       dy = ay[i] - ay[0];       dz = az[i] - az[0];
        ax[i] = ax[i] / ax[0];    ay[i] = ay[i] / ay[0];    az[i] = az[i] / az[0];
        a = sqrt(ax[i] * ax[i] + ay[i] * ay[i] + az[i] * az[i]);
        if (a > 3 && (((dz > 0.06 || dz < -0.06) && (dx < -0.06 || dx > 0.06)) || ((dz > 0.06 || dz < -0.06) && (dy < -0.06 || dy > 0.06)) || ((dy > 0.06 || dy < -0.06) && (dx < -0.06 || dx > 0.06))) && j > 3) {
            j = 0;
            counter++;
            checkangle = true;
            dx_old = dx; dy_old = dy; dz_old = dz;
            return_angle = 0;
        }
        if (checkangle) {
            if (( dx_old - dx > 0.02) || ( dx_old - dx < -0.02)) {
                return_angle++;
            }
            if (( dy_old - dy > 0.02) || ( dy_old - dy < -0.02)) {
                return_angle++;
            }
            if (( dz_old - dz > 0.02) || ( dz_old - dz < -0.02)) {
                return_angle++;
            }
        }
        if (j == 3 && checkangle && (return_angle < 2)) {
            Recalibrate();
            checkangle = false;
        }
        j++;
        previousMillis = currentMillis;
    }
    return(counter);
}

bool AccelUNO::Detected() {
    detected = false;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > 100) {
        i++;
        if (i >= 20) {
            i = 1;
        }
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
        x = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
        y = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        z = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
        gx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        gy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        gz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
        ax[i] = x / sensitivity; ay[i] = y / sensitivity; az[i] = z / sensitivity;
        dx = ax[i] - ax[0];       dy = ay[i] - ay[0];       dz = az[i] - az[0];
        ax[i] = ax[i] / ax[0];    ay[i] = ay[i] / ay[0];    az[i] = az[i] / az[0];
        a = sqrt(ax[i] * ax[i] + ay[i] * ay[i] + az[i] * az[i]);
        if (a > 3 && (((dz > 0.06 || dz < -0.06) && (dx < -0.06 || dx > 0.06)) || ((dz > 0.06 || dz < -0.06) && (dy < -0.06 || dy > 0.06)) || ((dy > 0.06 || dy < -0.06) && (dx < -0.06 || dx > 0.06))) && j > 3) {
            j = 0;
            detected = true;
            checkangle = true;
            dx_old = dx; dy_old = dy; dz_old = dz;
            return_angle = 0;
        }
        if (checkangle) {
            if (( dx_old - dx > 0.02) || ( dx_old - dx < -0.02)) {
                return_angle++;
            }
            if (( dy_old - dy > 0.02) || ( dy_old - dy < -0.02)) {
                return_angle++;
            }
            if (( dz_old - dz > 0.02) || ( dz_old - dz < -0.02)) {
                return_angle++;
            }
        }
        if (j == 3 && checkangle && (return_angle < 2)) {
            Recalibrate();
            checkangle = false;
        }
        j++;
        previousMillis = currentMillis;
    }
    return(detected);
}

void AccelUNO::Recalibrate() {
    for ( int i = 1; i <= 50; i++) {
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
        x = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
        y = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        z = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
        gx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        gy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        gz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    }
    ax[0] = x / sensitivity; ay[0] = y / sensitivity; az[0] = z / sensitivity;
}

void AccelUNO::PrintMPU() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
    x = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    y = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    z = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    temp = temp/340.00+36.53;

    Serial.print("Ax : "); Serial.print(x); Serial.print(" || ");
    Serial.print("Ay : "); Serial.print(y); Serial.print(" || ");
    Serial.print("Az : "); Serial.print(z); Serial.print(" || ");
    Serial.print("Temp : "); Serial.print(temp); Serial.print(" || ");
    Serial.print("Gx : "); Serial.print(gx); Serial.print(" || ");
    Serial.print("Gy : "); Serial.print(gy); Serial.print(" || ");
    Serial.print("Gz : "); Serial.println(gz);
}

int AccelUNO::Ax() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
    x = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    y = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    z = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    return x;
}

int AccelUNO::Ay() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
    x = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    y = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    z = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    return y;
}

int AccelUNO::Az() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
    x = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    y = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    z = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    return z;
}

int AccelUNO::MPUTemp() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
    x = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    y = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    z = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    temp = temp/340.00+36.53;

    return temp;
}

int AccelUNO::Gx() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
    x = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    y = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    z = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    return gx;
}

int AccelUNO::Gy() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
    x = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    y = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    z = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    return gy;
}

int AccelUNO::Gz() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
    x = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    y = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    z = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    return gz;
}
