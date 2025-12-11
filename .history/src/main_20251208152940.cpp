// ダブルバッファ + タスク優先 使用（ミューテックス版）
#include <M5Unified.h>
#include <WiFi.h>
#include <SD.h>
#include <SPI.h>
#include "led.h"
#include "Triger.h"
#include "ADC.h"
#include "ODRsetting.h"

// 共通
volatile bool newdata = false;
// TimeCount は符号付きで -1 を使えるようにする（初回ISRで0にしたい意図）
volatile int32_t TimeCount = -1;
char line[256];
int len;

// 状態
enum State { Setup, Standby, Recording, Finished };
State cur_state;
State prev_state;

// ======== ダブルバッファ関連 ========
#define BUFSIZE 512
static char dataBuf[2][BUFSIZE];   // 2つのバッファ
volatile int recBufNum = 0;        // producer が書いているバッファ番号（0 or 1）
volatile int dataIndex = 0;        // producer の書き込み位置（bytes）
volatile int bufLen[2] = {0,0};    // 各バッファの確定長（consumer 用）
volatile int writeBufNum = -1;     // consumer が書き込むべきバッファ番号（-1=なし）

// ミューテックス（共有変数保護用）
SemaphoreHandle_t bufMutex = NULL;

// ファイル
#define RECORD_INTERVAL_US 5000  // 200Hz
#define PIN_SD_CS   4
#define PIN_SD_MOSI 6
#define PIN_SD_SCK  7
#define PIN_SD_MISO 8

SPIClass spiHSPI(HSPI);
File logFile;
String logFilePath;
int measureCount = 0;

// タイマー
hw_timer_t *timer = NULL;
TaskHandle_t imuTaskHandle = nullptr;

// タイマー割り込み (5ms)
void IRAM_ATTR onTimer() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TimeCount++;  // まずインクリメント（最初は -1 -> 0）

    // IMUタスクを起こす
    vTaskNotifyGiveFromISR(imuTaskHandle, &xHigherPriorityTaskWoken);

    // より高優先度タスクが ready になったらコンテキスト切替
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

// IMU タスク（producer）
void imuTask(void *pvParameters) {
    for (;;) {
        // タイマISRからの通知を待つ（ブロック）
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // IMU読み取り（重い処理なのでタスクコンテキスト）
        M5.Imu.update();
        auto data = M5.Imu.getImuData();

        // 1行分を作る
        len = snprintf(line, sizeof(line),
                 "%ld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
                 (long)TimeCount,
                 data.accel.x, data.accel.y, data.accel.z,
                 data.gyro.x,  data.gyro.y,  data.gyro.z,
                 data.mag.x,   data.mag.y,   data.mag.z);

        if (len <= 0) continue; // 念のため
        if (len >= BUFSIZE) {
            // 1行長すぎてバッファに入らない（想定外）→無視またはログ
            Serial.println("line length too large for BUFSIZE!");
            continue;
        }

        // --- ミューテックスで共有変数を保護 ---
        if (xSemaphoreTake(bufMutex, (TickType_t)10) == pdTRUE) {
            int curBuf = recBufNum;
            int curIndex = dataIndex;

            // case A: まだ書ける
            if (curIndex + len < BUFSIZE) {
                memcpy(&dataBuf[curBuf][curIndex], line, len);
                dataIndex = curIndex + len;
                // release
                xSemaphoreGive(bufMutex);
            } else {
                // case B: 今のバッファが満杯になるので「このバッファを確定して切り替える」
                bufLen[curBuf] = curIndex;   // 確定長（今までの分。今回の行は次バッファへ）
                writeBufNum = curBuf;        // consumer に書き込みを依頼
                // 切り替え
                recBufNum ^= 1;              // producer は別バッファへ
                dataIndex = 0;               // 新しいバッファは 0 から書き始め
                // 今回の行は新しいバッファに書く
                int newBuf = recBufNum;
                memcpy(&dataBuf[newBuf][0], line, len);
                dataIndex = len;
                // release
                xSemaphoreGive(bufMutex);
                // consumer は loop() で writeBufNum を見て書き込みを行う
            }
        } else {
            // ミューテックスが取れなかった場合（ごく短時間）→行を落とすか簡易待ち
            // ここでは一行落とす（必要なら待つ処理に変更）
            Serial.println("bufMutex busy: dropped 1 line");
        }
    }
}

const char *header = "count,ax,ay,az,gx,gy,gz,mx,my,mz\r\n";
int headerLen = strlen(header);

// 新しいファイルを作成
bool makeFile() {
  logFilePath = "/OBS" + String(measureCount) + ".csv";
  logFile = SD.open(logFilePath, FILE_WRITE);
  if (!logFile) {
    Serial.println("ファイルを開けませんでした");
    return false;
  }
  logFile.write((const uint8_t*)header, headerLen);
  Serial.printf("ファイル作成: %s\n", logFilePath.c_str());
  return true;
}

// 記録終了
bool closeFile(){
  if (!logFile) {
    Serial.println("ファイルが開かれていません");
    return false;
  }

  // flush
  logFile.flush();

  logFile.close();
  Serial.println("ファイルを閉じました。");
  measureCount++;
  return true;
}

// entry 条件処理
void onEntry(State s){
  switch(s){
    case Standby:
      Led_Standby();
      if(!makeFile()){
        Led_Warning();
        Led_Finish();
        cur_state = Finished;
        return;
      }
      break;

    case Recording:
      Led_Recording();

      // タイマーカウンタを 0 に戻す
      if (timer) timerWrite(timer, 0);
      newdata = false;
      TimeCount = -1;

      timerStart(timer);
      timerAlarmEnable(timer);

      // ダブルバッファ初期化（ミューテックス保護）
      if (xSemaphoreTake(bufMutex, (TickType_t)100) == pdTRUE) {
          recBufNum = 0;
          dataIndex = 0;
          bufLen[0] = bufLen[1] = 0;
          writeBufNum = -1;
          xSemaphoreGive(bufMutex);
      }
      break;

    case Finished:
      Led_Finish();
      Serial.println("[ENTRY] Finished 実行");
      if (timer) {
        timerAlarmDisable(timer);
        timerStop(timer);
        timerEnd(timer);
        timer = NULL;
      }
      break;
  }
}

// setup
void setup() {
  Serial.begin(115200);
  prev_state = Setup;
  bool check = true;

  auto cfg = M5.config();
  cfg.internal_imu = true;
  cfg.external_imu = false;
  M5.begin(cfg);

  if (!ODRset()){
    Serial.println("ODR初期化失敗");
    check = false;
  }

  // ミューテックス作成
  bufMutex = xSemaphoreCreateMutex();
  if (bufMutex == NULL) {
    Serial.println("bufMutex create failed");
    // 致命的だが、続行しないでおく
    check = false;
  }

  // タスク作成（IMU タスク）
  xTaskCreatePinnedToCore(
      imuTask,
      "IMU_TASK",
      8192,        // snprintf を使うので余裕を増やす
      nullptr,
      5,           // 優先度
      &imuTaskHandle,
      0
  );

  // タイマー初期化
  timer = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 50000, true); // 5ms
  timerStop(timer);
  timerAlarmDisable(timer);
  if (!timer){ Serial.println("タイマー初期化失敗"); check = false; }

  if (!Led_Init()){ Serial.println("LED初期化失敗"); check = false; }
  if (!Triger_Init()){ Serial.println("トリガー初期化失敗"); check = false; }

  spiHSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  Serial.println("SDカード初期化中...");
  if (!SD.begin(PIN_SD_CS, spiHSPI, 20000000)) {
    Serial.println("SDカード初期化失敗");
    check = false;
  } else {
    Serial.println("SDカード初期化成功");
  }

  ADC_Init();
  if (!checkBattery()) { check = false; }

  // 初回の重いIMUupdateを排除
  M5.Imu.update();

  cur_state = Standby;
  if (!check){ Led_Warning(); cur_state = Finished; }
}

// loop（consumer はここで SD 書き込みを行う）
void loop(){
  // entry
  if (cur_state != prev_state){
    Serial.printf("[ENTRY] %d -> %d\n", prev_state, cur_state);
    onEntry(cur_state);
    prev_state = cur_state;
  }

  switch(cur_state){
    case Standby:
      if (Is_Triger()){
        prev_state = cur_state;
        cur_state = Recording;
        break;
      }
      if(!checkBattery()){
        Led_Warning();
        prev_state = cur_state;
        cur_state = Finished;
        break;
      }
      if (IsButton()){
        if (logFile) logFile.close();
        Led_Warning();
        prev_state = Recording;
        cur_state = Finished;
        return;
      }
      break;

    case Recording: {
        // --- consumer: バッファ書き込み処理 ---
        int toSD = -1;
        int writeLen = 0;

        if (xSemaphoreTake(bufMutex, (TickType_t)10) == pdTRUE) {
            if (writeBufNum != -1) {
                toSD = writeBufNum;
                writeLen = bufLen[toSD];
                writeBufNum = -1;   // consumer が受け取ったのでクリア
            }
            xSemaphoreGive(bufMutex);
        }

        if (toSD != -1 && writeLen > 0) {
            size_t w = logFile.write((uint8_t*)dataBuf[toSD], (size_t)writeLen);
            (void)w; // 必要ならチェック
        }

        // 記録終了判定
        if (Is_Triger() == TRIGER_ON || TimeCount == 60000) {
            Serial.println("記録終了");
            if (timer) {
              timerAlarmDisable(timer);
              timerStop(timer);
            }

            // 残りバッファの書き込み（producer が書いている方）
            int cur = 0;
            int rem = 0;
            if (xSemaphoreTake(bufMutex, (TickType_t)100) == pdTRUE) {
                cur = recBufNum;
                rem = dataIndex;
                xSemaphoreGive(bufMutex);
            }

            if (rem > 0) {
                logFile.write((uint8_t*)dataBuf[cur], (size_t)rem);
            }

            if (!closeFile()) {
                Led_Warning();
                Serial.println("ファイルを閉じれませんでした");
                prev_state = cur_state;
                cur_state = Finished;
                break;
            }

            prev_state = cur_state;
            cur_state = Standby;
        }

        break;
    }

    case Finished:
      // do nothing
      break;
  }
}




// // ダブルバッファ基本コード
// #include <M5Unified.h>
// #include <WiFi.h>
// #include <SD.h>
// #include <SPI.h>
// #include "led.h"
// #include "Triger.h"
// #include "ADC.h"
// #include "ODRsetting.h"

// //共通関数
// volatile bool newdata = false;
// volatile uint32_t count = -1; 
// //char line[256]; 

// //状態設定
// enum State{
//   Setup,
//   Standby,
//   Recording,
//   Finished
// };
// State cur_state;
// State prev_state;

// // ======== ダブルバッファ関連 ========
// #define BUFSIZE 512
// static char dataBuf[2][BUFSIZE];  // 2つのバッファ
// volatile int recBufNum = 0;       // 今書いているバッファ 0 or 1
// volatile int dataIndex = 0;       // dataBuf[recBufNum] の使用量
// volatile bool bufReady = false;   // 書き込み要求フラグ（満杯）

// //ファイル設定
// #define RECORD_INTERVAL_US 5000  // 200Hz
// #define PIN_SD_CS   4
// #define PIN_SD_MOSI 6
// #define PIN_SD_SCK  7
// #define PIN_SD_MISO 8

// SPIClass spiHSPI(HSPI);

// File logFile;
// String logFilePath;
// int measureCount = 0;

// //タイマー設定
// hw_timer_t *timer = NULL;
// volatile bool flagWrite = false;
// volatile uint32_t count = -1;   

// // タイマー割り込み (5ms)
// void IRAM_ATTR onTimer() {
//   flagWrite = true;
//   count++;   // ← サンプリング番号を1増やす
// }

// const char *header = "count,ax,ay,az,gx,gy,gz,mx,my,mz\r\n";
// int headerLen = strlen(header);

// // 新しいファイルを作成
// bool makeFile() {
//   logFilePath = "/OBS" + String(measureCount) + ".csv";
//   logFile = SD.open(logFilePath, FILE_WRITE);
//   if (!logFile) {
//     Serial.println("ファイルを開けませんでした");
//     return false;
//   }
//   logFile.write((const uint8_t*)header, headerLen);
//   Serial.printf("ファイル作成: %s\n", logFilePath.c_str());
//   return true;
// }
// // //記録終了
// bool closeFile(){
  
//   measureCount++;

//   logFile.flush();
//   if (!logFile){
//     Serial.println("flush失敗");
//     return false;
//   }
//   logFile.close();
//   if (logFile){
//     Serial.println("close失敗");
//     return false;
//   }
//   Serial.println("ファイルを閉じました。");
//   return true;
// }

// //各状態のentryで行う関数
// void onEntry(State s){
//   switch(s){
//     case Standby:
//       Led_Standby();
//       if(!makeFile()){
//         Led_Warning();
//         Led_Finish();
//         //prev_state = Standby;
//         cur_state = Finished;
//         return;
//       }
//       break;

//     case Recording:
//       Led_Recording();

//     //同じタイマーを毎回使う場合
//       timerStop(timer);
//       timerAlarmDisable(timer);  // ← 念のため
//       flagWrite = false; 
//       count = -1;
//       timerRestart(timer);

//       timerStart(timer);
//       timerAlarmEnable(timer);

//       recBufNum = 0;
//       dataIndex = 0;
//       bufReady = false;

//       break;

//     case Finished:
//       Led_Finish();
//       Serial.println("[ENTRY] Finished 実行");
//       timerEnd(timer);
//      break;
//   }
// }

// // セットアップ
// void setup() {
//   Serial.begin(115200);

//   prev_state = Setup;

//   //初期化失敗を感知するチェック
//   bool check = true;

//   //M5の初期化
//   auto cfg = M5.config();
//   cfg.internal_imu = true;
//   cfg.external_imu = false;
//   M5.begin(cfg);

//   //ODR設定
//   if (!ODRset()){
//     Serial.println("ODR初期化失敗");
//     check = false;
//   }

//   //タイマー初期化
//   //毎回同じタイマーを使う場合ここで初期化
//   timer = timerBegin(0, 80, true);
//   timerAttachInterrupt(timer, &onTimer, false); // ← level割込みに（警告対策）
//   timerAlarmWrite(timer, RECORD_INTERVAL_US, true);
//   //timerAlarmEnable(timer);
//   timerStop(timer);
//   timerAlarmDisable(timer);
//   if (!timer){
//     Serial.println("タイマー初期化失敗");
//     check = false;
//   }
  
//   //LED初期化
//   if (!Led_Init()){
//     Serial.println("LED初期化失敗");
//     check = false;
//   }

//   //GPIO1初期化
//   if (!Triger_Init()){
//     Serial.println("トリガー初期化失敗");
//     check = false;
//   };

//   //SD初期化
//   spiHSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
//   Serial.println("SDカード初期化中...");
//   if (!SD.begin(PIN_SD_CS, spiHSPI, 20000000)) {
//     Serial.println("SDカード初期化失敗");
//     check = false;
//   } else {
//     Serial.println("SDカード初期化成功");
//   }
  
//   //ADC初期化
//   ADC_Init();

//     //バッテリーチェック
//   if (!checkBattery()){
//     check = false;
//   }

//   //初回の重いIMUupdateを排除
//   M5.Imu.update();

//   cur_state = Standby;

//   if (!check){
//     Led_Warning();
//     cur_state = Finished;
//   }
// }

// // メインループ
// void loop(){

//   //entryの処理
//   if (cur_state != prev_state){
//     Serial.printf("[ENTRY] %d -> %d\n", prev_state, cur_state);
//     onEntry(cur_state);
//     prev_state = cur_state;
//   }

//   //各状態のdoの処理
//   switch(cur_state){

//     //スタンバイ
//     case Standby:
//       //トリガーシグナルのチェック
//       if (Is_Triger()){
//         prev_state = cur_state;
//         cur_state = Recording;
//         break;
//       }

//       //バッテリー低下処理
//       if(!checkBattery()){
//         Led_Warning();
//         prev_state = cur_state;
//         cur_state = Finished;
//         break;
//       }

//       //ボタン処理
//       if (IsButton()){
//         logFile.close();
//         Led_Warning();
//         prev_state = Recording;
//         cur_state = Finished;
//         return;
//       }

//       break;
    
//     //記録中
//     case Recording: {

//         // --- IMU記録（ダブルバッファ利用） ---
//         if (flagWrite) {
//             flagWrite = false;

//             uint32_t raw_count = count;

//             // ======== 70カウント捨てる処理(あなたの仕様のまま) ========
//             if (raw_count < 70) {
//                 M5.Imu.update();

//                 char line[128];
//                 int len = snprintf(line, sizeof(line),
//                     "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\r\n",
//                     count, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1
//                 );

//                 // // ---- バッファへの書き込み ----
//                 // if (dataIndex + len < BUFSIZE) {
//                 //     memcpy(&dataBuf[recBufNum][dataIndex], line, len);
//                 //     dataIndex += len;
//                 // } else {
//                 //     bufReady = true;
//                 // }

//                 // 一番最初の本物データのためにヘッダー直後に戻す仕様（従来通り）
//                 logFile.seek(headerLen);

//                 break;
//             }

//             // ======== 実際のデータ記録部分 ========
//             uint32_t rec_count = raw_count - 70;

//             M5.Imu.update();
//             float ax, ay, az, gx, gy, gz, mx, my, mz;
//             M5.Imu.getAccel(&ax, &ay, &az);
//             M5.Imu.getGyro(&gx, &gy, &gz);
//             M5.Imu.getMag(&mx, &my, &mz);

//             char line[128];
//             int len = snprintf(line, sizeof(line),
//                 "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
//                 rec_count, ax, ay, az, gx, gy, gz, mx, my, mz
//             );

//             // ---- バッファへ書き込み ----
//             if (dataIndex + len < BUFSIZE) {
//                 memcpy(&dataBuf[recBufNum][dataIndex], line, len);
//                 dataIndex += len;
//             } else {
//                 bufReady = true;
//             }
//         }

//         // ======== バッファ書き込み処理 ========
//         if (bufReady) {
//             bufReady = false;

//             // 書き込むバッファ番号を退避
//             int toSD = recBufNum;
//             int len = dataIndex;

//             // バッファ切り替え
//             recBufNum ^= 1;
//             dataIndex = 0;

//             // SD へ書き込み
//             logFile.write((uint8_t*)dataBuf[toSD], len);
//         }

//         // ======== 記録終了判定(あなたのまま) ========
//         if (Is_Triger() == TRIGER_ON || count == 60000) {
//             Serial.println("記録終了");

//             timerAlarmDisable(timer);
//             timerStop(timer);
//             flagWrite = false;

//             // バッファ残り分の書き込み
//             if (dataIndex > 0) {
//                 logFile.write((uint8_t*)dataBuf[recBufNum], dataIndex);
//             }

//             if (!closeFile()) {
//                 Led_Warning();
//                 Serial.println("ファイルを閉じれませんでした");
//                 prev_state = cur_state;
//                 cur_state = Finished;
//                 break;
//             }

//             prev_state = cur_state;
//             cur_state = Standby;
//         }

//         break;
//     }

    
//     //終了状態
//     case Finished:
//       //何もしない
//       break;
//   }
// }


