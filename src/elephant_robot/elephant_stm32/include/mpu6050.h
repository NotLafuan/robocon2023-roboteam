#include <Arduino.h>
#include <Wire.h>

class MPU6050
{
private:
    int scl, sda;
    int16_t x, y, z;

public:
    MPU6050(int, int);
    ~MPU6050();
    void begin();
    void update();
    int get_x();
    int get_y();
    int get_z();
};

MPU6050::MPU6050(int scl, int sda)
{
    this->scl = scl;
    this->sda = sda;
}

MPU6050::~MPU6050()
{
}

void MPU6050::begin()
{
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
}

void MPU6050::update()
{
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    x = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
}

int MPU6050::get_x()
{
    return x;
}

int MPU6050::get_y()
{
    return y;
}

int MPU6050::get_z()
{
    return z;
}
