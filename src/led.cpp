#include <M5Unified.h>
#include "led.h"
#include "ADC.h"

int LedState; //状態変数

bool Led_Init(){
  LedState = LED_OFF;
  M5.Display.fillScreen(TFT_WHITE);
  M5.Lcd.setBrightness(50);
  return true;
}


void Led_Recording() {
  M5.Display.setBrightness(50);      // 明るくする
  M5.Display.fillScreen(TFT_BLUE);     // 青い画面にする
}

void Led_Standby() {
  M5.Display.setBrightness(50);       // 暗めにする
  M5.Display.fillScreen(TFT_WHITE);   // 白い画面にする
}

void Led_Warning(){
  M5.Display.setBrightness(50);       // 暗めにする
  M5.Display.fillScreen(TFT_RED);   // 赤い画面にする 警告状態（終了前や5分経った後に知らせる）
  delay(WARNING_TIME); 
}

void Led_Finish(){
  M5.Display.setBrightness(0);       // 暗めにする
  M5.Display.fillScreen(TFT_BLACK);   // 黒い画面にする  終了状態
}

void Led_LowBattery(){
  M5.Display.setBrightness(50); //暗めにする
  M5.Display.fillScreen(TFT_YELLOW); //黄色にする　バッテリーが少ない状態
}

