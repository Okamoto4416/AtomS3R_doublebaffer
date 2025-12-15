#include <M5Unified.h>
#include "Triger.h"

static int prev_triger;
static bool prev_button;

bool Triger_Init(){
    pinMode(PIN_TRIGER, INPUT_PULLDOWN);
    pinMode(PIN_BUTTON, INPUT_PULLUP);  // ボタンは内部プルアップ

    //初期状態としてのtrigerの状態を保存
    prev_triger = LOW;
    prev_button = LOW;
    return true;
}

// 「LOW→HIGH」立ち上がりを検出して、1回だけTRIGER_ONを返す
// --- ユーザー要求の関数：1回だけtrueを返す ---
// bool Is_Triger() {
//   int cur_triger = digitalRead(PIN_TRIGER);
//   bool rise = TRIGER_OFF;

//   if (prev_triger == HIGH && cur_triger == LOW) {
//     rise = TRIGER_ON;
//   }

//   prev_triger = cur_triger;
//   return rise;           // それ以外は false
// }

bool Is_Triger() {
  bool trig = TRIGER_OFF;

  // HIGH → LOW のエッジを検出
  if (digitalRead(PIN_TRIGER) == HIGH) {

    // while の中身を適用
    delayMicroseconds(50);
    if (digitalRead(PIN_TRIGER) == LOW) {
      trig = TRIGER_ON;
    }
  }
  return trig;
}

bool IsButton(){
    int cur_button = digitalRead(PIN_BUTTON);
    if (cur_button == LOW && prev_button == HIGH) {  
        Serial.println("Button pusshed");
        prev_button = cur_button;
        return BUTTON_ON;
    } else {
        prev_button = cur_button;      
        return BUTTON_OFF;
    }
}
