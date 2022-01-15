#ifndef Pot_h
#define Pot_h

#include <Arduino.h>

class Pot{
    private:
        uint8_t _pin;
        uint16_t _val;
    public:
        Pot(uint8_t aPin);
        ~Pot(){};
        uint16_t eval(uint16_t min, uint16_t max);
};

Pot::Pot(uint8_t aPin){
    _pin = aPin;
}

uint16_t Pot::eval(uint16_t min, uint16_t max){
    uint16_t curVal = analogRead(_pin);
    _val = 0.8 * _val + 0.2 * curVal;
    return map(_val, 0, 1024, min, max);
}

#endif