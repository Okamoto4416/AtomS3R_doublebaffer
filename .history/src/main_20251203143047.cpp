/*
makefileが失敗したときに終了しない
*/

// //70カウント捨てるもの
#include <M5Unified.h>
#include <WiFi.h>
#include <SD.h>
#include <SPI.h>
#include "led.h"
#include "Triger.h"
#include "ADC.h"
#include "ODRsetting.h"

//状態設定
enum State{
  Setup,
  Standby,
  Recording,
  Finished
};
State cur_state;
State prev_state;

// ======== ダブルバッファ関連 ========
#define BUFSIZE 512
static char dataBuf[2][BUFSIZE];  // 2つのバッファ
volatile int recBufNum = 0;       // 今書いているバッファ 0 or 1
volatile int dataIndex = 0;       // dataBuf[recBufNum] の使用量
volatile bool bufReady = false;   // 書き込み要求フラグ（満杯）

//ファイル設定
#define RECORD_INTERVAL_US 5000  // 200Hz
#define PIN_SD_CS   4
#define PIN_SD_MOSI 6
#define PIN_SD_SCK  7
#define PIN_SD_MISO 8

SPIClass spiHSPI(HSPI);

File logFile;
String logFilePath;
int measureCount = 0;

//タイマー設定
hw_timer_t *timer = NULL;
volatile bool flagWrite = false;
volatile uint32_t count = -1;   

// タイマー割り込み (5ms)
void IRAM_ATTR onTimer() {
  flagWrite = true;
  count++;   // ← サンプリング番号を1増やす
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
// //記録終了
bool closeFile(){
  
  measureCount++;

  logFile.flush();
  if (!logFile){
    Serial.println("flush失敗");
    return false;
  }
  logFile.close();
  if (logFile){
    Serial.println("close失敗");
    return false;
  }
  Serial.println("ファイルを閉じました。");
  return true;
}

//各状態のentryで行う関数
void onEntry(State s){
  switch(s){
    case Standby:
      Led_Standby();
      if(!makeFile()){
        Led_Warning();
        Led_Finish();
        //prev_state = Standby;
        cur_state = Finished;
        return;
      }
      break;

    case Recording:
      Led_Recording();

    //同じタイマーを毎回使う場合
      timerStop(timer);
      timerAlarmDisable(timer);  // ← 念のため
      flagWrite = false; 
      count = -1;
      timerRestart(timer);

      timerStart(timer);
      timerAlarmEnable(timer);

      recBufNum = 0;
      dataIndex = 0;
      bufReady = false;

      break;

    case Finished:
      Led_Finish();
      Serial.println("[ENTRY] Finished 実行");
      timerEnd(timer);
     break;
  }
}

// セットアップ
void setup() {
  Serial.begin(115200);

  prev_state = Setup;

  //初期化失敗を感知するチェック
  bool check = true;

  //M5の初期化
  auto cfg = M5.config();
  cfg.internal_imu = true;
  cfg.external_imu = false;
  M5.begin(cfg);

  //ODR設定
  if (!ODRset()){
    Serial.println("ODR初期化失敗");
    check = false;
  }

  //タイマー初期化
  //毎回同じタイマーを使う場合ここで初期化
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, false); // ← level割込みに（警告対策）
  timerAlarmWrite(timer, RECORD_INTERVAL_US, true);
  //timerAlarmEnable(timer);
  timerStop(timer);
  timerAlarmDisable(timer);
  if (!timer){
    Serial.println("タイマー初期化失敗");
    check = false;
  }
  
  //LED初期化
  if (!Led_Init()){
    Serial.println("LED初期化失敗");
    check = false;
  }

  //GPIO1初期化
  if (!Triger_Init()){
    Serial.println("トリガー初期化失敗");
    check = false;
  };

  //SD初期化
  spiHSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  Serial.println("SDカード初期化中...");
  if (!SD.begin(PIN_SD_CS, spiHSPI, 20000000)) {
    Serial.println("SDカード初期化失敗");
    check = false;
  } else {
    Serial.println("SDカード初期化成功");
  }
  
  //ADC初期化
  ADC_Init();

    //バッテリーチェック
  if (!checkBattery()){
    check = false;
  }

  //初回の重いIMUupdateを排除
  M5.Imu.update();

  cur_state = Standby;

  if (!check){
    Led_Warning();
    cur_state = Finished;
  }
}

// メインループ
void loop(){

  //entryの処理
  if (cur_state != prev_state){
    Serial.printf("[ENTRY] %d -> %d\n", prev_state, cur_state);
    onEntry(cur_state);
    prev_state = cur_state;
  }

  //各状態のdoの処理
  switch(cur_state){

    //スタンバイ
    case Standby:
      //トリガーシグナルのチェック
      if (Is_Triger()){
        prev_state = cur_state;
        cur_state = Recording;
        break;
      }

      //バッテリー低下処理
      if(!checkBattery()){
        Led_Warning();
        prev_state = cur_state;
        cur_state = Finished;
        break;
      }

      //ボタン処理
      if (IsButton()){
        logFile.close();
        Led_Warning();
        prev_state = Recording;
        cur_state = Finished;
        return;
      }

      break;
    
    //記録中
    case Recording: {

        // --- IMU記録（ダブルバッファ利用） ---
        if (flagWrite) {
            flagWrite = false;

            uint32_t raw_count = count;

            // ======== 70カウント捨てる処理(あなたの仕様のまま) ========
            if (raw_count < 70) {
                M5.Imu.update();

                char line[128];
                int len = snprintf(line, sizeof(line),
                    "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\r\n",
                    count, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1
                );

                // // ---- バッファへの書き込み ----
                // if (dataIndex + len < BUFSIZE) {
                //     memcpy(&dataBuf[recBufNum][dataIndex], line, len);
                //     dataIndex += len;
                // } else {
                //     bufReady = true;
                // }

                // 一番最初の本物データのためにヘッダー直後に戻す仕様（従来通り）
                logFile.seek(headerLen);

                break;
            }

            // ======== 実際のデータ記録部分 ========
            uint32_t rec_count = raw_count - 70;

            M5.Imu.update();
            float ax, ay, az, gx, gy, gz, mx, my, mz;
            M5.Imu.getAccel(&ax, &ay, &az);
            M5.Imu.getGyro(&gx, &gy, &gz);
            M5.Imu.getMag(&mx, &my, &mz);

            char line[128];
            int len = snprintf(line, sizeof(line),
                "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\r\n",
                rec_count, ax, ay, az, gx, gy, gz, mx, my, mz, 0
            );

            // ---- バッファへ書き込み ----
            if (dataIndex + len < BUFSIZE) {
                memcpy(&dataBuf[recBufNum][dataIndex], line, len);
                dataIndex += len;
            } else {
                bufReady = true;
            }
        }

        // ======== バッファ書き込み処理 ========
        if (bufReady) {
            bufReady = false;

            // 書き込むバッファ番号を退避
            int toSD = recBufNum;
            int len = dataIndex;

            // バッファ切り替え
            recBufNum ^= 1;
            dataIndex = 0;

            // SD へ書き込み
            logFile.write((uint8_t*)dataBuf[toSD], len);
        }

        // ======== 記録終了判定(あなたのまま) ========
        if (Is_Triger() == TRIGER_ON || count == 60000) {
            Serial.println("記録終了");

            timerAlarmDisable(timer);
            timerStop(timer);
            flagWrite = false;

            // バッファ残り分の書き込み
            if (dataIndex > 0) {
                logFile.write((uint8_t*)dataBuf[recBufNum], dataIndex);
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

    
    //終了状態
    case Finished:
      //何もしない
      break;
  }
}

//別コア
// #include <M5Unified.h>
// #include <WiFi.h>
// #include <SD.h>
// #include <SPI.h>
// #include "led.h"
// #include "Triger.h"
// #include "ADC.h"

// #define RECORD_INTERVAL_US 5000   // 200Hz (5ms)
// #define PIN_SD_CS   4
// #define PIN_SD_MOSI 6
// #define PIN_SD_SCK  7
// #define PIN_SD_MISO 8

// SPIClass spiHSPI(HSPI);

// #define BUFSIZE 512  // バッファサイズ（SDカードのセクタ単位）

// static char dataBuf[2][BUFSIZE];
// volatile int bufLen[2] = {0, 0};       // 各バッファの有効データ長
// volatile int recBufNum = 0;            // 現在書き込み中のバッファ番号 (0 or 1)
// volatile int dataIndex = 0;            // 書き込み位置
// volatile bool bufReady[2] = {false, false}; // 各バッファの書き込み要求
// volatile bool flagSample = false;      // サンプリングフラグ

// File logFile;
// String logFilePath;
// int measureCount = 0;
// bool isRecording = false;

// hw_timer_t *timer = NULL;
// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// //-------------------------------------------
// // タイマISR（5msごとに発火）
// //-------------------------------------------
// void IRAM_ATTR onTimer() {
//   portENTER_CRITICAL_ISR(&timerMux);
//   flagSample = true;
//   portEXIT_CRITICAL_ISR(&timerMux);
// }

// //-------------------------------------------
// // 新しいログファイルを作成
// //-------------------------------------------
// void makeFile() {
//   logFilePath = "/OBS" + String(measureCount) + ".csv";
//   logFile = SD.open(logFilePath, FILE_WRITE);
//   if (logFile) {
//     logFile.println("time,ax,ay,az,gx,gy,gz,mx,my,mz");
//     Serial.printf("ファイル作成: %s\n", logFilePath.c_str());
//   } else {
//     Serial.printf("ファイル作成失敗: %s\n", logFilePath.c_str());
//   }
// }

// //-------------------------------------------
// // 記録タスク（別スレッド）
// //-------------------------------------------
// void recordTask(void *pvParameters) {
//   for (;;) {
//     // サンプリング発生時
//     if (flagSample && isRecording) {
//       portENTER_CRITICAL(&timerMux);
//       flagSample = false;
//       portEXIT_CRITICAL(&timerMux);

//       // --- IMU読み取り ---
//       M5.Imu.update();
//       float ax, ay, az, gx, gy, gz, mx, my, mz;
//       M5.Imu.getAccel(&ax, &ay, &az);
//       M5.Imu.getGyro(&gx, &gy, &gz);
//       M5.Imu.getMag(&mx, &my, &mz);
//       unsigned long now = esp_timer_get_time();

//       // --- CSV行生成 ---
//       char line[128];
//       int len = snprintf(line, sizeof(line),
//                         "%08lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
//                         now, ax, ay, az, gx, gy, gz, mx, my, mz);

//       // --- バッファに格納 ---
//       portENTER_CRITICAL(&timerMux);
//       if (dataIndex + len >= BUFSIZE) {
//         // 現バッファ満杯 → 書き込み要求＋切替
//         bufLen[recBufNum] = dataIndex;  // 有効データ長を保存
//         bufReady[recBufNum] = true;
//         recBufNum ^= 1;     // バッファ切り替え
//         dataIndex = 0;
//       }

//       memcpy(&dataBuf[recBufNum][dataIndex], line, len);
//       dataIndex += len;
//       portEXIT_CRITICAL(&timerMux);
//     }

//     // --- SD書き込み（記録中のみ） ---
//     for (int i = 0; i < 2; ++i) {
//       if (bufReady[i] && isRecording) {
//         portENTER_CRITICAL(&timerMux);
//         bufReady[i] = false;
//         int len = bufLen[i];  // 実際の有効データ長
//         portEXIT_CRITICAL(&timerMux);

//         if (logFile && len > 0) {
//           logFile.write((uint8_t*)dataBuf[i], len);
//           // flush() はしない（速度優先）
//         }
//       }
//     }

//     vTaskDelay(1);
//   }
// }

// //-------------------------------------------
// // セットアップ
// //-------------------------------------------
// void setup() {
//   Serial.begin(115200);
//   M5.begin();

//   spiHSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
//   if (!SD.begin(PIN_SD_CS, spiHSPI, 20000000)) {
//     Serial.println("SD初期化失敗");
//   } else {
//     Serial.println("SD初期化成功");
//   }

//   makeFile();
//   Triger_Init();
//   Led_Init();

//   // --- タイマ設定（5msごと） ---
//   timer = timerBegin(0, 80, true); // 1us単位
//   timerAttachInterrupt(timer, &onTimer, false);  // ← LEVELモードでattach（警告回避）
//   timerAlarmWrite(timer, RECORD_INTERVAL_US, true);
//   timerAlarmEnable(timer);
//   timerStop(timer);

//   // --- 記録タスクを別コアで起動 ---
//   xTaskCreatePinnedToCore(recordTask, "recordTask", 8192, NULL, 1, NULL, 1);
// }

// //-------------------------------------------
// // メインループ
// //-------------------------------------------
// void loop() {
//   int trig = Is_Triger();

//   // 記録開始
//   if (trig == TRIGER_ON && !isRecording) {
//     Serial.println("記録開始");
//     isRecording = true;
//     dataIndex = 0;
//     recBufNum = 0;
//     bufReady[0] = bufReady[1] = false;
//     bufLen[0] = bufLen[1] = 0;
//     timerStart(timer);
//     Led_Recording();
//   }
//   // 記録終了
//   else if (trig == TRIGER_ON && isRecording) {
//     Serial.println("記録終了");
//     isRecording = false;
//     timerStop(timer);

//     // --- 残りデータを書き出す ---
//     portENTER_CRITICAL(&timerMux);
//     int len = dataIndex;
//     int bufNum = recBufNum;
//     portEXIT_CRITICAL(&timerMux);

//     if (logFile && len > 0) {
//       logFile.write((uint8_t*)dataBuf[bufNum], len);
//     }

//     // --- 最後にflush & close ---
//     if (logFile) {
//       logFile.flush();
//       logFile.close();
//       Serial.println("ファイル保存完了");
//     }

//     measureCount++;
//     makeFile();
//     Led_Standby();
//   }

//   Led_Check(checkBattery(), isRecording);
// }


//ダブルバッファ
// #include <M5Unified.h>
// #include <WiFi.h>
// #include <SD.h>
// #include <SPI.h>
// #include "led.h"
// #include "Triger.h"
// #include "ADC.h"

// #define RECORD_INTERVAL_US 5000   // 200Hz (5ms)
// #define PIN_SD_CS   4
// #define PIN_SD_MOSI 6
// #define PIN_SD_SCK  7
// #define PIN_SD_MISO 8

// SPIClass spiHSPI(HSPI);

// // ============ 記録関連 ============
// #define BUFSIZE 2048               // 1バッファあたりのサイズ
// static char dataBuf[2][BUFSIZE];   // ダブルバッファ
// volatile int recBufNum = 0;        // 現在記録中のバッファ
// volatile int dataIndex = 0;        // 現在の書き込み位置
// volatile bool bufReady = false;    // 書き込み完了フラグ
// volatile uint32_t sampleCount = 0; // ★ サンプル番号カウンタ

// File logFile;
// String logFilePath;
// int measureCount = 0;
// bool isRecording = false;

// // ============ タイマー ============
// hw_timer_t *timer = NULL;
// volatile bool flagWrite = false;

// // --------------------------------------------------------
// // タイマー割り込み：サンプルカウント増加
// // --------------------------------------------------------
// void IRAM_ATTR onTimer() {
//   flagWrite = true;
//   sampleCount++;   // ★ 割り込みごとにサンプル番号を増やす
// }

// // --------------------------------------------------------
// // ログファイル作成
// // --------------------------------------------------------
// void makeFile() {
//   logFilePath = "/OBS" + String(measureCount) + ".csv";
//   logFile = SD.open(logFilePath, FILE_WRITE);
//   if (!logFile) {
//     Serial.println("ファイルを開けませんでした");
//     return;
//   }
//   logFile.println("count,ax,ay,az,gx,gy,gz,mx,my,mz");  // ★ タイムスタンプ列名変更
//   logFile.flush();
//   Serial.printf("新しいログファイルを作成しました: %s\n", logFilePath.c_str());
// }

// // --------------------------------------------------------
// // セットアップ
// // --------------------------------------------------------
// void setup() {
//   Serial.begin(115200);
//   M5.begin();

//   pinMode(PIN_SD_CS, OUTPUT);
//   digitalWrite(PIN_SD_CS, HIGH);

//   spiHSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

//   Serial.println("SDカード初期化中...");
//   if (!SD.begin(PIN_SD_CS, spiHSPI, 20000000)) {
//     Serial.println("SDカード初期化失敗");
//   } else {
//     Serial.println("SDカード初期化成功");
//   }

//   makeFile();

//   // タイマー設定
//   timer = timerBegin(0, 80, true);
//   timerAttachInterrupt(timer, &onTimer, false); // level割込み
//   timerAlarmWrite(timer, RECORD_INTERVAL_US, true);
//   timerAlarmEnable(timer);
//   timerStop(timer);
//   timerRestart(timer);

//   Triger_Init();
//   Led_Init();
// }

// // --------------------------------------------------------
// // メインループ
// // --------------------------------------------------------
// void loop() {
//   int trig = Is_Triger();

//   // --- 記録開始 ---
//   if (trig == TRIGER_ON && !isRecording) {
//     Serial.println("GPIO1がHIGHになりました！（記録開始）");
//     isRecording = true;
//     dataIndex = 0;
//     recBufNum = 0;
//     bufReady = false;
//     sampleCount = 0; // ★ サンプル番号リセット
//     timerStart(timer);
//     Led_Recording();
//   }

//   // --- 記録終了 ---
//   else if (trig == TRIGER_ON && isRecording) {
//     Serial.println("GPIO1がHIGHになりました！（記録終了）");
//     isRecording = false;
//     timerStop(timer);

//     // バッファに残った分を書き込む
//     if (dataIndex > 0) {
//       logFile.write((uint8_t*)dataBuf[recBufNum], dataIndex);
//     }
//     logFile.flush();
//     logFile.close();
//     Serial.println("ファイルを閉じました。");

//     measureCount++;
//     timerRestart(timer);
//     makeFile();
//     Led_Standby();
//   }

//   // --- データ記録処理（200Hz） ---
//   if (flagWrite && isRecording) {
//     flagWrite = false;

//     // IMU読み取り
//     M5.Imu.update();
//     float ax, ay, az, gx, gy, gz, mx, my, mz;
//     M5.Imu.getAccel(&ax, &ay, &az);
//     M5.Imu.getGyro(&gx, &gy, &gz);
//     M5.Imu.getMag(&mx, &my, &mz);

//     uint32_t count = sampleCount;  // ★ タイムスタンプ代わりのカウンタ

//     // CSV形式に変換
//     char line[128];
//     int len = snprintf(line, sizeof(line),
//                       "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
//                       count, ax, ay, az, gx, gy, gz, mx, my, mz);

//     // バッファにコピー
//     if (dataIndex + len < BUFSIZE) {
//       memcpy(&dataBuf[recBufNum][dataIndex], line, len);
//       dataIndex += len;
//     } else {
//       // バッファが満杯 → もう一方に切り替えて書き込み
//       bufReady = true;
//     }
//   }

//   // --- バッファ書き込み処理 ---
//   if (bufReady && isRecording) {
//     bufReady = false;

//     int toTFnum = recBufNum;   // 書き込み対象を記録
//     int len = dataIndex;       // 現在のサイズを記録

//     // バッファを切り替え
//     recBufNum ^= 1;
//     dataIndex = 0;

//     // TFカード書き込み
//     logFile.write((uint8_t*)dataBuf[toTFnum], len);
//     //logFile.flush();
//   }

//   Led_Check(checkBattery(), isRecording);
// }


