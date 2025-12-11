#include <M5Unified.h>
#include "ADC.h"
#include "led.h"

// === 定数設定 ===
// 数字f でフロートであると示す
constexpr int PIN_ADC = 2;  // AC1 = GPIO2
constexpr float VREF = 3.3f;    // 3.3Vフルスケール
constexpr int ADC_MAX = 4095.0f;       // 12bit ADC(分解能)
constexpr float DIVIDER = 2.0f; // 電圧1/2で分圧されている
constexpr float THRESHOLD = 3.5f; // 電池電圧低下判定値[V]

static uint32_t prev_time;

void ADC_Init(){
    pinMode(2,INPUT);
    prev_time = 0;
}

bool checkBattery(){
    uint32_t now_time = millis();
    if (now_time - prev_time >= 10000){
        prev_time = now_time;

        int adcRaw = analogRead(PIN_ADC);
        float batteryVoltage = ((float)adcRaw / ADC_MAX) * VREF * DIVIDER;

        if (batteryVoltage <= THRESHOLD){
            Serial.printf("ADC raw=%d  voltage=%.3f V\n", adcRaw, batteryVoltage);
            Serial.println("vattery low");
            return BATTERY_LOW;
        }  
        return BATTERY_OK;
    }
    return BATTERY_OK;
}