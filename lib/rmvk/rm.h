/*
    Author: Nikita Rovda
    Date: 2022.01.09
    email: nikita.rovda@gmail.com
*/

#ifndef rm_h
#define rm_h

#include <SoftwareSerial.h>
#include <Arduino.h>

#define _RM_READ_TIMEOUT 50
#define RM_MAX_V 220               // max output V
#define RM_MIN_V 40                // min output V
#define _RM_MIN_R 1000              // 10.0 Ohm
#define _RM_MAX_R 3000              // 30.0 Ohm
#define RM_STATE_ON true
#define RM_STATE_OFF false
#define RM_3KW_HEATER_R 1613        // 16.13 Ohm


uint16_t f_P(uint16_t U, uint16_t R){
    return round(U * U / (R / 100.0));
}

uint16_t f_V(uint16_t P, uint16_t R){
    return round(sqrt(P * (R / 100.0)));
}

class RMVK{
    private:
        uint8_t _rxPin, _txPin;
        SoftwareSerial *_Serial;
        char _buf[10];
        bool _state = true;
        uint16_t _vi;
        uint16_t _vo;
        uint16_t _p;
        uint32_t _energy; // counter Watt / 10ms
        uint16_t _max_p;
        uint16_t _min_p;
        uint16_t _load_resistance;
        uint16_t _pvTable[RM_MAX_V+1];
        uint32_t _last_getVo = 0; 
        uint32_t lastDimmStep = 0;
        bool _read(uint8_t timeout=_RM_READ_TIMEOUT);
    public:
        ~RMVK();
        RMVK(uint8_t rxPin, uint8_t txPin, uint16_t load_resistance=RM_3KW_HEATER_R);
        void init();
        uint16_t setLoadResistance(uint16_t new_resistance);
        uint16_t getMaxP(){return _max_p;};
        uint16_t getMinP(){return _min_p;};
        uint16_t getVi(bool update=true);
        uint16_t getVo(bool update=true);
        uint16_t getP(bool update=true);
        uint16_t getWh();
        uint16_t setV(uint16_t new_v);
        uint16_t setP(uint16_t new_p);
        uint16_t dimmV(uint16_t dstV);
        uint16_t dimmP(uint16_t dstP);
        uint16_t pvLookup(uint16_t v);
        bool setState(bool newState);
        bool getState();
};

RMVK::~RMVK(){
}

RMVK::RMVK(uint8_t rxPin, uint8_t txPin, uint16_t load_resistance=RM_3KW_HEATER_R){
    setLoadResistance(load_resistance);
    _rxPin = rxPin;
    _txPin = txPin;
}

void RMVK::init(){
    _Serial = new SoftwareSerial(_rxPin, _txPin);
    _Serial->begin(9600);
    _Serial->flush();
    setState(false);  // set initial state: off
}

uint16_t RMVK::setLoadResistance(uint16_t new_resistance){
    if ( (new_resistance < _RM_MIN_R) || (new_resistance > _RM_MAX_R) )
        return _load_resistance;

    _load_resistance = new_resistance;
    for (uint8_t v=0; v<=RM_MAX_V; v++){
        _pvTable[v] = f_P(v, _load_resistance);
    }
    _max_p = _pvTable[RM_MAX_V];
    Serial.print("_max_p: ");
    Serial.print(_max_p);
    _min_p = _pvTable[RM_MIN_V];
    return _load_resistance;
}

bool RMVK::_read(uint8_t timeout=_RM_READ_TIMEOUT){
    uint8_t k = 0;
    while (!_Serial->available()){
        delay(1);
        if(++k == _RM_READ_TIMEOUT) return false;
    }

    uint8_t n = 0;
    while (_Serial->available()){
        _buf[n++] = _Serial->read();
        if (!_Serial->available())
            delay(1);
    }
    _buf[n] = '\0';
    return true;
}

uint16_t RMVK::getVi(bool update=true){
    if (!update)
        return _vi;
    _Serial->print("AT+VI?\r");
    if (!_read()){
        _vi = 0;
    } else {
        _vi = String(_buf).toInt();
    }
    return _vi;
}

uint16_t RMVK::getVo(bool update=true){
    if (_state == RM_STATE_OFF)
        return 0;

    if (update){
        _Serial->print("AT+VO?\r");
        if (!_read()){
            _vo = 0;
        } else {
            _vo = String(_buf).toInt();
        }
    }
    _p = _pvTable[_vo];
    return _vo;
}

uint16_t RMVK::getP(bool update=true){
    getVo(update);
    // Wh counter
    static uint32_t last_calc = 0;
    _energy += (millis() - last_calc) * _p / 10;
    last_calc = millis();
    return _p;
}

uint16_t RMVK::getWh(){
    getP(false);
    return _energy /100 /60 /60;
}

uint16_t RMVK::setV(uint16_t new_v){
    if ((new_v == _vo) || (new_v > RM_MAX_V))
      return _vo;

    if (new_v < RM_MIN_V){
    //   if (_state == RM_STATE_ON)
        setState(RM_STATE_OFF);
        return _vo;
    }
        
    // set new V
    if (_state == RM_STATE_OFF)
      if (setState(RM_STATE_ON) != RM_STATE_ON)
        return _vo;

    sprintf(_buf, "AT+VS=%03d\r", new_v);
    _Serial->print(_buf);
    if ( (!_read()) || (_buf[0] == 'e'))
        return _vo;

    _vo = new_v;
    _p = _pvTable[_vo];
    return _vo;
}

uint16_t RMVK::setP(uint16_t new_p){
    if ((new_p == _p) || (new_p > _max_p))
      return _p;
    // uint16_t realP = calculateRealP_P(new_p);
    // if (realP == _p)
    //     return _p;
      
    if (new_p < _min_p){
        setState(RM_STATE_OFF);
        return _p;
    }

    uint16_t new_vo = f_V(new_p, _load_resistance);
      
    if (setV(new_vo) == new_vo){
      _vo = new_vo;
      _p = new_p;
    }
    return _p;
}

// return true while dimm in progress, false when dimm end
uint16_t RMVK::dimmV(uint16_t dstV){
    if (dstV >= 0 && _vo < RM_MIN_V){
        setV(RM_MIN_V);
        lastDimmStep = millis();
        return true;
    }
    if (_vo == RM_MAX_V || _vo >= dstV)
        return false;                       // dimm end
    if (millis() - lastDimmStep >= 600){   // next step every sec
        setV(_vo + 1);
        lastDimmStep = millis();
    }
    return true;
}

uint16_t RMVK::dimmP(uint16_t dstP){
    if (_p >= dstP)
        return false;                   // dimm end
    return dimmV(_vo + 1);
}

uint16_t RMVK::pvLookup(uint16_t v){
    if (v > RM_MAX_V)
        return 0;
    return _pvTable[v];
}


bool RMVK::getState(){
    _Serial->print("AT+ON?\r");
    if (!_read())
        return _state;

    if (_buf[1] == 'N'){
        _state = RM_STATE_ON;
    } else if (_buf[1] == 'F') {
        _state = RM_STATE_OFF;
    }
    return _state;
}

bool RMVK::setState(bool new_state){
    sprintf(_buf, "AT+ON=%d\r", new_state);
    _Serial->print(_buf);
    if (!_read())
        return _state;

    _state = new_state;
    if (_state == RM_STATE_OFF){
        _vo = 0;
        _p = 0;
    }
    return _state;
}

#endif
