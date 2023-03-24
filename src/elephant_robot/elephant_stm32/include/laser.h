// #include <Arduino.h>

// class Laser
// {
// private:
//     int pin0;
//     int pin1;
//     volatile int value;
//     int currentState;
//     int lastState;

// public:
//     Encoder(int, int);
//     ~Encoder();
//     int get_value();
//     void encoderUpdate();
// };

// Encoder::Encoder(int pin0, int pin1)
// {
//     this->pin0 = pin0;
//     this->pin1 = pin1;

//     pinMode(pin0, INPUT_PULLUP);
//     pinMode(pin1, INPUT_PULLUP);

//     lastState = digitalRead(pin0);
// }

// Encoder::~Encoder()
// {
// }

// int Encoder::get_value()
// {
//     return value;
// }

// void Encoder::encoderUpdate()
// {
//     currentState = digitalRead(pin0);
//     if (currentState != lastState && currentState == 1)
//     {
//         if (digitalRead(pin1) != currentState)
//             value--;
//         else
//             value++;
//     }
//     lastState = currentState;
// }