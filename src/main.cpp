/*
    Author: Nikita Rovda
    Date: 2022.01.09
    email: nikita.rovda@gmail.com
*/

#include <microLiquidCrystal_I2C.h>
#include <thermistor.h>
#include <rm.h>
#include <buzzer.h>
#include <OneButton.h>
#include <pot.h>
#include "helpers.h"
// #include <avr/eeprom.h>

// input pins
#define LBTN_PIN 2
#define RBTN_PIN 3
#define POT_PIN A3

#define BUZZER_PIN 8

#define RMVK_RX 10
#define RMVK_TX 11

// temp sensors pins
#define NTC1_PIN A1
#define NTC2_PIN A2
#define NTC3_PIN A0

#define NTC3_K 1.0175


// update intervals
#define READ_NTC_INTERVAL           1000
#define READ_RMVK_INTERVAL          2000
#define INPUTS_UPDATE_INTERVAL      100
#define OPERATION_PROGRAM_INTERVAL  100
#define PLOT_INTERVAL               1000

#define HEAD_WARM_TEMP              450     // treat lower value as cold head 
#define ST1_LOW_ALC                 12      // lower alc threshold on 1st stage
#define ST2_LOW_ALC                 90      // lower alc threshold on 2nd stage

// output voltage settings
struct {
    uint16_t max = 200;
    uint16_t st1 = 180;         // 1st distillation stage
    uint16_t st2 = 127;         // 2nd distillation stage
    uint16_t t3_max1 = 90;      // when t3 > 30C
    uint16_t t3_max2 = 40;      // when t3 > 40C
    uint16_t t1_max =  40;      // when t1 > 99.8C     
} outV;

// aclcohol lookup table by head sensor
const uint8_t lut_head_size = 38;
uint16_t lut_head[lut_head_size][2] = {
    {763,950},{766,936},{768,926},{769,922},{770,916},
    {773,905},{774,901},{783,875},{789,856},{791,850},
    {798,827},{809,800},{819,778},{827,760},{834,747},
    {839,737},{846,725},{855,710},{858,706},{858,706},
    {868,688},{872,679},{876,670},{886,640},{895,615},
    {905,575},{914,533},{917,519},{926,470},{932,435},
    {938,397},{946,348},{950,320},{956,280},{961,242},
    {969,178},{971,160},{978,106}
};

LiquidCrystal_I2C lcd(0x27, 16, 2);
RMVK rmvk(RMVK_RX, RMVK_TX);

THERMISTOR ntc1(NTC1_PIN, 100000, 3950, 100000);    // tank sensor
FilterExpRunningAverage ntc1_filter;

THERMISTOR ntc2(NTC2_PIN, 100000, 3950, 100000);    // head (steam) sensor
FilterExpRunningAverage ntc2_filter;

THERMISTOR ntc3(NTC3_PIN, 100000, 3950, 100000);    // output (fridge) sensor
FilterExpRunningAverage ntc3_filter;

OneButton lBtn(LBTN_PIN, false, false);
OneButton rBtn(LBTN_PIN, false, false);
Buzzer buzzer(BUZZER_PIN);
Pot pot(POT_PIN);

enum OperationMode{
    MODE_MANUAL = 0, 
    MODE_A0 = 1,
    MODE_A1 = 2,
    MODE_A2 = 3,
    MODE_OFF = 4,
    SWITCH_NEXT = 5
} operationMode;


// runtime sensors values
struct {
    uint16_t 
        head,                       // steam phase
        tank,                       // liquid phase
        fridge;                     // coolant tmp
} tmp;

struct {
    uint16_t
        cube,                       // % alc. left in cube
        head;                       // % alc. in output
} alco;

uint32_t lastTouch = 0;                // last user input activity
uint8_t accelerate_finished = false;    // acc finished flag
uint16_t customP = 0;                    // manual specified voltage


void printSpace(int count){
    if (count < 0) return;
    for (byte n=0; n < count; n++){
        lcd.print(' ');
    }
}

void calculateAlcohol(){
    uint16_t t_min = lut_head[0][0] - 15;               // lookup table min temp
    uint16_t t_max = lut_head[lut_head_size-1][0];      // lookup table max temp
    alco.head = 0;                                      // alcohol in output
    if ((tmp.head >= t_min) && (tmp.head <= t_max)){
        uint16_t min_diff = 100;
        uint8_t min_idx = 0;
        for (uint8_t n=0; n<lut_head_size; n++){
            uint16_t diff = abs( lut_head[n][0] - tmp.head );
            if (diff < min_diff){
                min_idx = n;
                min_diff = diff;
            }
        }
        float k = (float)lut_head[min_idx][0] / (float)tmp.head;
        alco.head = round( k * lut_head[min_idx][1] / 10.0);
    }
    // ToDo:
    alco.cube = 0;                                      // % alcohol in tank
}

void readNTC() {
    tmp.head = ntc1_filter.filter((float)ntc1.read());
    tmp.tank = ntc2_filter.filter((float)ntc2.read());
    tmp.fridge = ntc3_filter.filter((float)ntc3.read() * NTC3_K);
}

void printSecondScreen(){
    static uint32_t last_out = 0;
    if (millis() - last_out < 1000)
        return;
    // energy meter
    lcd.setCursor(0,0);
    printSpace(11);
    lcd.setCursor(0,0);
    lcd.print("Wh:");
    lcd.print(rmvk.getWh());
    // uptime
    lcd.setCursor(0,1);
    printSpace(11);
    lcd.setCursor(0,1);
    uint16_t s;
    uint8_t h,m;
    s = millis() / 1000;
    h = s / 3600; s -= h * 3600;
    m = s / 60; s -= m * 60; 
    char uptimeStr[11];
    sprintf(uptimeStr, "Up:%02d:%02d:%02d", h, m, s);
    lcd.print(uptimeStr);
    last_out = millis();
}

void printCustomP(){
    lcd.setCursor(12,1);
    printSpace(4 - lcd.print(customP));
}

void printModeAlco(){
    char opModeSymb[] = {'P','0','1','2','-'};
    char maStr[5];
    sprintf(maStr, "%c %02d\0", opModeSymb[operationMode], alco.head);
    lcd.setCursor(12,0);
    lcd.print(maStr);
}

void switchMode(OperationMode newMode=SWITCH_NEXT){
    buzzer.beep(1);
    if (newMode == SWITCH_NEXT){
        switch (operationMode) {
            case MODE_OFF:
                operationMode = MODE_MANUAL;
                break;
            case MODE_MANUAL:
                operationMode = MODE_A0;
                break;
            case MODE_A0:
                operationMode = MODE_A1;
                break;
            case MODE_A1:
                operationMode = MODE_A2;
                break;
            case MODE_A2:
                operationMode = MODE_OFF;
                accelerate_finished = false;  // reset flag
                break;
        }
    } else {
        operationMode = newMode;
    }
    printModeAlco();
}

void processInput() {
    uint16_t potV = pot.eval(RM_MIN_V - 5, outV.max + 1);
    if (potV < RM_MIN_V)
        potV = 0;
    uint16_t potP = rmvk.pvLookup(potV);
    if (potP != customP){            // value changed
        lastTouch = millis();
        customP = potP;
        if (customP < rmvk.getMinP())
            customP = 0;
        printCustomP();
    }

    rBtn.tick();
    lBtn.tick();
    if (rBtn.isLongPressed()){
        printSecondScreen();
        lastTouch = millis();
    }
    else if (rBtn.getNumberClicks()==1){
        switchMode();
    }
}

// output overheat protection
bool isFridgeOk(){
    if (tmp.fridge > 400){          // t3 > 40C 
        rmvk.setV(outV.t3_max2);
        buzzer.beep(1);
        return false;
    }
    if (tmp.fridge > 300){          // t3 > 30C
        rmvk.setV(outV.t3_max1);
        buzzer.beep(1);
        return false;
    }
    return true;
}

// tank overheat protection
bool isTankOk(){
    if (tmp.tank >= 998){   // t1 > 99.8C
        buzzer.beep(1);
        rmvk.setV(outV.t1_max);
        return false;
    }
    return true;
}

// heat fluid in tank on maximum voltage while head is cold
bool accelerate(){
    if (accelerate_finished) return false;
    if (tmp.head >= 450){            // end at 45C
        accelerate_finished = true;
        return false;
    }
    rmvk.dimmV(outV.max);
    return true;
}

void routineModeManual(){
    if (tmp.tank < 600){
        if (rmvk.dimmP(customP)) // slow warm up when heater is cold
            return;
    }
    rmvk.setP(customP);
}

// accelerate and wait
void routineModeQuickHeat(){
    if (accelerate()) return;
    buzzer.beep(1);
    if (tmp.head > 600){
        rmvk.setP(200);
    } else { 
        if (tmp.head > 500)
            rmvk.setP(500);
        else
            rmvk.setP(900);
    }
}

void routineModeAuto(uint8_t stage){
    uint8_t lowThr = stage==1 ? ST1_LOW_ALC : ST2_LOW_ALC;
    if ( (alco.head > 0) && (alco.head < lowThr) ){
        rmvk.setP(customP * 0.5);
        buzzer.beep(1);
        return;
    }
    rmvk.setP(customP);
}

void loop(){
    buzzer.beep();

    static uint32_t 
        lastNTCRead,
        lastRMVKRead,
        lastInputUpdate,
        lastRoutineLoop,
        lastPlot;

    if (millis() - lastInputUpdate > INPUTS_UPDATE_INTERVAL) {
        processInput();
        lastInputUpdate = millis();
    }

    // skip processing while user inputs in use 
    if (millis() - lastTouch < 1000)
        return;

    if (millis() - lastNTCRead > READ_NTC_INTERVAL) {
        readNTC();
        calculateAlcohol();
        char tempStr[12];
        sprintf(tempStr, 
            "%03d %03d %03d ",
            tmp.tank, tmp.head, tmp.fridge
        );
        lcd.setCursor(0,0);
        lcd.print(tempStr);
    
        printModeAlco();
        lastNTCRead = millis();
    }

    if (millis() - lastRMVKRead > READ_RMVK_INTERVAL) {
        // input voltage
        char rmvkStr[11];
        sprintf(rmvkStr,
            "%3dV %4dW ",
            rmvk.getVi(), rmvk.getP());
        lcd.setCursor(0,1);
        lcd.print(rmvkStr);
        lastRMVKRead = millis();
    }

    if (millis() - lastRoutineLoop > OPERATION_PROGRAM_INTERVAL){
        if (operationMode != MODE_OFF){
            if (!isFridgeOk() || !isTankOk()){
                return;
            } else {
                switch (operationMode){
                    case MODE_MANUAL:
                        routineModeManual();
                        break;
                    case MODE_A0:
                        routineModeQuickHeat();
                        break;
                    case MODE_A1:
                        routineModeAuto(1);
                        break;
                    case MODE_A2:
                        routineModeAuto(2);
                        break;
                }
            }
        } else {
            rmvk.setState(RM_STATE_OFF);
        }
    }

    if (millis() - lastPlot > PLOT_INTERVAL){
        char plotStr[100];
        uint16_t upt =int(millis() /1000 /60);
        sprintf(plotStr, 
            "{\"t1\":%3d, \"t2\":%3d, \"t3\":%3d, \"alco\":%2d, \"p1\":%4d, \"wh\":%5d, \"up\":%3d,\"vi\":%3d}\0",
            tmp.tank, tmp.head, tmp.fridge, alco.head, rmvk.getP(false), rmvk.getWh(), upt, rmvk.getVi(false));
        Serial.println(plotStr);
        lastPlot = millis();
    }
}

void setup(){
    Serial.begin(9600);
    buzzer.beep(1);
    lcd.init();
    lcd.backlight();
    lcd.clear();
    switchMode(MODE_OFF);
    rmvk.init();
    rmvk.setState(RM_STATE_OFF);
    rBtn.setPressTicks(1000);
    lBtn.setPressTicks(1000);
}