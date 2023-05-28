#include <Arduino.h>

class Encoder
{
private:
    int pin0;
    int pin1;
    volatile int value;
    int currentState;
    int lastState;
    double distance_per_count = 0.157;

public:
    Encoder(int, int, bool);
    ~Encoder();
    double get_value();
    void encoderUpdate();
};

Encoder::Encoder(int pin0, int pin1, bool reversed = false)
{
    if (!reversed)
    {
        this->pin0 = pin0;
        this->pin1 = pin1;
    }
    else
    {
        this->pin0 = pin1;
        this->pin1 = pin0;
    }

    pinMode(this->pin0, INPUT_PULLUP);
    pinMode(this->pin1, INPUT_PULLUP);

    lastState = digitalRead(this->pin0);
}

Encoder::~Encoder()
{
}

double Encoder::get_value()
{
    double return_value = (double)value * distance_per_count;
    value = 0;
    return return_value;
}

void Encoder::encoderUpdate()
{
    currentState = digitalRead(pin0);
    if (currentState != lastState && currentState == 1)
    {
        if (digitalRead(pin1) != currentState)
            value--;
        else
            value++;
    }
    lastState = currentState;
}