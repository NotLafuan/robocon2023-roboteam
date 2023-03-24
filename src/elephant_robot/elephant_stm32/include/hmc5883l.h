#include <Arduino.h>
#include <Wire.h>

class HMC5883L
{
private:
    int16_t x, y, z;
    double heading;

public:
    HMC5883L();
    ~HMC5883L();
    void begin();
    void update();
    int get_x();
    int get_y();
    int get_z();
    double get_heading();
};

HMC5883L::HMC5883L()
{
}

HMC5883L::~HMC5883L()
{
}

void HMC5883L::begin()
{
    Wire.beginTransmission(0x1E);
    Wire.write(0x02);
    Wire.write(0x00);
    Wire.endTransmission();
}

void HMC5883L::update()
{
    Wire.beginTransmission(0x1E);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(0x1E, 6);
    if (Wire.available() <= 6)
    {
        x = Wire.read() << 8 | Wire.read();
        z = Wire.read() << 8 | Wire.read();
        y = Wire.read() << 8 | Wire.read();
    }
    heading = atan2(x, y) / 0.0174532925;
    if (heading < 0)
        heading += 360;
    heading = 360 - heading; // N=0/360, E=90, S=180, W=270
}

int HMC5883L::get_x()
{
    return x;
}

int HMC5883L::get_y()
{
    return y;
}

int HMC5883L::get_z()
{
    return z;
}

double HMC5883L::get_heading()
{
    return heading;
}
