#ifndef Buzzer_h
#define Buzzer_h

#include <Arduino.h>

#define BEEP_INTERVAL 150

class Buzzer{
    private:
        uint8_t _pin;
        uint32_t beep_end_millis = 0;
        uint32_t last_change = 0;
        uint8_t cur_state = false;
    public:
        void beep(uint16_t beep_count=0);
        Buzzer(uint8_t pin);
        ~Buzzer(){};
};

Buzzer::Buzzer(uint8_t pin){
    _pin = pin;
    pinMode(_pin, OUTPUT);
}

void Buzzer::beep(uint16_t beep_count=0){
    if (beep_count > 0)
        beep_end_millis = BEEP_INTERVAL * beep_count * 2 + millis();
    if (millis() - last_change < BEEP_INTERVAL)
        return;
    if (beep_end_millis <= millis()){
        if (cur_state){
            cur_state = false;
            digitalWrite(_pin, LOW);
        }
        return;
    }
    cur_state = !cur_state;
    digitalWrite(_pin, cur_state);
    last_change = millis();
}
#endif