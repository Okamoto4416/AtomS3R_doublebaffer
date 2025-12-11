// main.cpp
// ダブルバッファ(4096) + タスク優先化（ISR -> IMUタスク -> SDタスク）版
// - IMU.update() は ISR に入れない（クラッシュ回避）
// - IMUタスク（高優先）が必ず 200Hz のサンプルをバッファに格納する
// - SDタスク（低優先）が書き出す（遅くて良い）
// - 共有同期は最小限の原子操作で行い、imuTask はブロックしない方針

#include <M5Unified.h>
#include <WiFi.h>
#include <SD.h>
#include <SPI.h>
#include "led.h"
#include "Triger.h"
#include "ADC.h"
#include "ODRsetting.h"

// -------------------- 設定 --------------------
#define RECORD_INTERVAL_US 5000       // 200Hz = 5000us
#define BUFSIZE 4096                  // バッファ容量（バイト）1バッファあたり
#define SD_SPI_FREQ 20000000          // SD SPI クロック

// -------------------- グローバル（共有） --------------------
hw_timer_t *timer = nullptr;

// ダブルバッファ本体（2個）
static char dataBuf[2][BUFSIZE];

// どのバッファに現在書いているか（0 or 1）。producer (imuTask) がのみ変更。
volatile int recBufNum = 0;

// 現在の書き込み位置（producer がのみ更新）
volatile int dataIndex = 0;

// bufLen[i] : i 番目バッファの確定長（producer が満杯にしたときに設定）
volatile int bufLenArr[2] = {0, 0};

// bufReadyMask のビット i が立っていれば i 番目バッファが SD 書き込み待ち
volatile uint8_t bufReadyMask = 0; // ビット0 -> buf0, ビット1 -> buf1

// タスクハンドル
TaskHandle_t imuTaskHandle = nullptr;
TaskHandle_t sdTaskHandle  = nullptr;

//SD 関係
#define PIN_SD_CS   4
#define PIN_SD_MOSI 6
#define PIN_SD_SCK  7
#define PIN_SD_MISO 8

SPIClass spiHSPI(HSPI);
File logFile;
String logFilePath;
int measureCount = 0;
const char *header = "count,ax,ay,az,gx,gy,gz,mx,my,mz\r\n";
int headerLen = strlen(header);

// 状態マシン
enum State{ Setup, Standby, Recording, Finished };
State cur_state;
State prev_state;

// サンプリングカウント（ISR がインクリメント）
volatile uint32_t countSample = 0;

// -------------------- ユーティリティ --------------------
// Atomics wrapper helper (GCC builtin)
static inline uint8_t atomic_fetch_or_u8(volatile uint8_t* p, uint8_t v) {
    return __atomic_fetch_or(p, v, __ATOMIC_SEQ_CST);
}
static inline uint8_t atomic_fetch_and_u8(volatile uint8_t* p, uint8_t v) {
    return __atomic_fetch_and(p, v, __ATOMIC_SEQ_CST);
}

// -------------------- タイマーISR（超軽量） --------------------
void IRAM_ATTR onTimer() {
    // 非常に軽く：カウント増やしてIMUタスクを起こすだけ
    countSample++;  // 5ms毎にインクリメント

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // imuTask を起こす（通知）
    vTaskNotifyGiveFromISR(imuTaskHandle, &xHigherPriorityTaskWoken);

    // もし IMUタスクが高優先度で ready なら直ちにコンテキストスイッチ
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

// -------------------- ファイル操作 --------------------
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
bool closeFile(){
  if (logFile) {
    logFile.flush();
    logFile.close();
    Serial.println("ファイルを閉じました。");
  }
  measureCount++;
  return true;
}

// -------------------- IMUタスク（高優先：プロデューサ） --------------------
void imuTask(void *pvParameters) {
    (void)pvParameters;
    for (;;) {
        // ISR からの通知を待つ（5msごとに通知される）
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 状態が Recording でなければ何もしない
        if (cur_state != Recording) continue;

        // 1) サンプルカウント取得（ローカルコピー）
        uint32_t raw_count = countSample;

        // 2) 最初の70サンプルは仕様どおりダミーデータ的に扱う
        // if (raw_count < 70) {
        //     // IMU update だけ行ってヘッダ直後に戻す仕様
        //     M5.Imu.update();

        //     // 作る行（70未満用のダミー行）
        //     char line[128];
        //     int len = snprintf(line, sizeof(line),
        //         "%lu,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1\r\n",
        //         (unsigned long)raw_count
        //     );

        //     // 書き込み（producer のみが dataIndex, recBufNum を操作）
        //     if (dataIndex + len < BUFSIZE) {
        //         memcpy(&dataBuf[recBufNum][dataIndex], line, len);
        //         dataIndex += len;
        //     } else {
        //         // 現在バッファがいっぱいになった → 確定して切り替え
        //         int toSD = recBufNum;
        //         bufLenArr[toSD] = dataIndex;            // 確定長
        //         // 切替：新しいバッファへ書き続ける
        //         recBufNum ^= 1;
        //         dataIndex = 0;
        //         // 今回の行を新バッファの先頭に書く（必ず BUFSIZE > len を想定）
        //         if (len < BUFSIZE) {
        //             memcpy(&dataBuf[recBufNum][dataIndex], line, len);
        //             dataIndex = len;
        //         } else {
        //             // 異常に大きい行は無視（実運用では起きない）
        //         }
        //         // 書き込み待ちフラグを立てる（atomic）
        //         atomic_fetch_or_u8(&bufReadyMask, (1 << toSD));
        //         // sdTask を起こす（通知）
        //         xTaskNotifyGive(sdTaskHandle);
        //     }
        //     // move on
        //     continue;
        // }

        // 3) 通常サンプル処理
        uint32_t rec_count = raw_count - 70;
        // IMU読み出し（タスクコンテキストなのでOK）
        M5.Imu.update();
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        M5.Imu.getAccel(&ax, &ay, &az);
        M5.Imu.getGyro(&gx, &gy, &gz);
        M5.Imu.getMag(&mx, &my, &mz);

        // CSV1行生成
        char line[128];
        int len = snprintf(line, sizeof(line),
            "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
            (unsigned long)rec_count,
            ax, ay, az, gx, gy, gz, mx, my, mz);

        // 4) ダブルバッファへ書き込み（必ず行う設計）
        if (len <= 0) continue; // 念のため

        if (dataIndex + len < BUFSIZE) {
            // 空きがあるなら普通にコピー
            memcpy(&dataBuf[recBufNum][dataIndex], line, len);
            dataIndex += len;
        } else {
            // 今のバッファが満杯になる → 確定して切り替え（オーバーフロー回避）
            int toSD = recBufNum;
            bufLenArr[toSD] = dataIndex;  // 今までの確定長（今回の行は次バッファへ）
            // 切り替え：producer は別バッファへ
            recBufNum ^= 1;
            dataIndex = 0;
            // 今回の行を新バッファに書く（len < BUFSIZE 前提）
            if (len < BUFSIZE) {
                memcpy(&dataBuf[recBufNum][dataIndex], line, len);
                dataIndex = len;
            } else {
                // 1行が BUFSIZE を超えるのは想定外
            }
            // 書き込み待ちフラグを立てる（atomic）
            atomic_fetch_or_u8(&bufReadyMask, (1 << toSD));
            // sdTask に通知して書き込みを促す（非ブロッキング）
            xTaskNotifyGive(sdTaskHandle);
        }

        // ループ継続。imuTask は次の通知で再び動く（5msごと）
    } // for
}

// -------------------- SD書き込みタスク（低優先） --------------------
void sdTask(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        // sdTask は通知されるまで待機（通知は imuTask が設定する）
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // もし state が Recording でないなら ignore（安全）
        if (cur_state != Recording) continue;

        // 連続して書き待ちバッファがある限り処理する
        while (true) {
            uint8_t mask_snapshot = __atomic_load_n(&bufReadyMask, __ATOMIC_SEQ_CST);
            if (mask_snapshot == 0) break; // 書き待ち無し

            // find lowest ready buffer (0 or 1)
            int toWrite = -1;
            if (mask_snapshot & 0x01) toWrite = 0;
            else if (mask_snapshot & 0x02) toWrite = 1;

            if (toWrite < 0) break; // safety

            // Clear the bit atomically (we claim the slot)
            atomic_fetch_and_u8(&bufReadyMask, (uint8_t)~(1 << toWrite));

            // Read length atomically (producer wrote it before setting mask)
            int len = bufLenArr[toWrite];

            // Sanity checks
            if (len <= 0) continue;
            if (len > BUFSIZE) len = BUFSIZE;

            // Write to SD (this can be slow)
            if (logFile) {
                size_t w = logFile.write((uint8_t*)dataBuf[toWrite], (size_t)len);
                (void)w; // optional: check return value
            }
            // loop to see next ready buffer
        } // while
    } // for
}

// -------------------- onEntry（状態遷移時の処理） --------------------
void onEntry(State s) {
    switch (s) {
      case Standby:
        Led_Standby();
        if (!makeFile()) {
            Led_Warning();
            Led_Finish();
            cur_state = Finished;
        }
        break;

      case Recording:
        Led_Recording();

        // タイマーをリセットして開始
        if (timer) {
            timerAlarmDisable(timer);
            timerStop(timer);
            // timerWrite は存在する API を使う（リセット）
            timerWrite(timer, 0);
            // start
            timerAlarmWrite(timer, RECORD_INTERVAL_US, true);
            timerAlarmEnable(timer);
            timerStart(timer);
        }

        // ダブルバッファ初期化（producerだけが操作する設計）
        recBufNum = 0;
        dataIndex = 0;
        bufLenArr[0] = bufLenArr[1] = 0;
        __atomic_store_n(&bufReadyMask, 0, __ATOMIC_SEQ_CST);

        countSample = 0;
        break;

      case Finished:
        Led_Finish();
        Serial.println("[ENTRY] Finished 実行");
        if (timer) {
            timerAlarmDisable(timer);
            timerStop(timer);
        }
        break;

      default:
        break;
    }
}

// -------------------- setup --------------------
void setup() {
    Serial.begin(115200);
    prev_state = Setup;

    bool check = true;

    auto cfg = M5.config();
    cfg.internal_imu = true;
    cfg.external_imu = false;
    M5.begin(cfg);

    // ODR 設定（あなたの既存関数）
    if (!ODRset()) {
        Serial.println("ODR初期化失敗");
        check = false;
    }

    // SD 初期化（SPI）
    spiHSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    Serial.println("SDカード初期化中...");
    if (!SD.begin(PIN_SD_CS, spiHSPI, SD_SPI_FREQ)) {
        Serial.println("SDカード初期化失敗");
        check = false;
    } else {
        Serial.println("SDカード初期化成功");
    }

    if (!Led_Init()) { Serial.println("LED初期化失敗"); check = false; }
    if (!Triger_Init()) { Serial.println("トリガー初期化失敗"); check = false; }
    ADC_Init();
    if (!checkBattery()) { check = false; }

    // IMU heavy first update
    M5.Imu.update();

    // タスク生成：
    // imuTask（高優先度） — Producer
    xTaskCreatePinnedToCore(
        imuTask,
        "IMU_TASK",
        8192,           // スタック余裕を持たせる
        NULL,
        5,              // 高優先度（5）
        &imuTaskHandle,
        0
    );

    // sdTask（低優先度） — Consumer
    xTaskCreatePinnedToCore(
        sdTask,
        "SD_TASK",
        8192,
        NULL,
        1,              // 低優先度（1）
        &sdTaskHandle,
        1
    );

    // タイマー初期化（ISRは軽く）
    timer = timerBegin(0, 80, true);
    // level mode を使うと警告が出るケースがあるので環境に合わせて第3引数を調整
    timerAttachInterrupt(timer, &onTimer, false); // false=level, true=edge -> 適宜変更
    timerAlarmWrite(timer, RECORD_INTERVAL_US, true);
    timerStop(timer);
    timerAlarmDisable(timer);

    // 初期状態
    cur_state = Standby;
    if (!check) { Led_Warning(); cur_state = Finished; }
}

// -------------------- loop（状態遷移の処理） --------------------
void loop() {
    // entry処理
    if (cur_state != prev_state) {
        Serial.printf("[ENTRY] %d -> %d\n", prev_state, cur_state);
        onEntry(cur_state);
        prev_state = cur_state;
    }

    switch (cur_state) {
      case Standby:
        // トリガのチェック（あなたの既存関数）
        if (Is_Triger()) {
            prev_state = cur_state;
            cur_state = Recording;
            break;
        }
        if (!checkBattery()) {
            Led_Warning();
            prev_state = cur_state;
            cur_state = Finished;
            break;
        }
        if (IsButton()) {
            if (logFile) logFile.close();
            Led_Warning();
            prev_state = cur_state;
            cur_state = Finished;
            return;
        }
        break;

      case Recording:
        // SD 書き出しは sdTask が担当し、imuTask が bufReadyMask をセットすると通知している
        // loop は軽い状態遷移のみ行う（ここに割り込み処理を入れない）
        // 記録終了判定
        if (Is_Triger() == TRIGER_ON || countSample >= 60000) {
            Serial.println("記録終了");
            // stop timer
            if (timer) {
                timerAlarmDisable(timer);
                timerStop(timer);
            }
            // sdTask にも書き込みを促して残りを吐かせる
            xTaskNotifyGive(sdTaskHandle);
            // 少し待ってファイル閉じ（SDが遅い場合は待ち時間増やす）
            vTaskDelay(pdMS_TO_TICKS(200));
            closeFile();
            prev_state = cur_state;
            cur_state = Standby;
        }
        break;

      case Finished:
        // 何もしない
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}


// //タスク優先導入版
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

// // ================================
// //  タスクハンドラ
// // ================================
// TaskHandle_t imuTaskHandle = nullptr;   // ← IMU取得タスク（最優先）
// TaskHandle_t sdTaskHandle  = nullptr;   // ← SD書き込みタスク（低優先）

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

// const char *header =
//     "count,ax,ay,az,gx,gy,gz,mx,my,mz\r\n";
// int headerLen = strlen(header);

// //タイマー設定
// hw_timer_t *timer = NULL;
// volatile bool flagWrite = false;
// // volatile uint32_t count = -1;   

// // ================================
// //  ★ ISR（割り込み）。重い処理禁止
// // ================================
// void IRAM_ATTR onTimer() {
//     flagWrite = true;
//     count++;

//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//     // IMUタスクに通知 → すぐ実行される
//     vTaskNotifyGiveFromISR(imuTaskHandle, &xHigherPriorityTaskWoken);

//     // IMUタスクの優先度が高ければ即スイッチ
//     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
// }

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

// // =======================================
// //  ★ IMUタスク（最優先）
// //  タイマー割り込みのたびに実行される
// // =======================================
// void imuTask(void *pv) {

//     for(;;)
//     {
//         // ISR からの通知待ち（最大待ち時間 無限）
//         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//         // ---------------------
//         //   Recording中のみ処理
//         // ---------------------
//         if (cur_state != Recording) continue;

//         // flagWrite が無ければ何もしない
//         if (!flagWrite) continue;
//         flagWrite = false;

//         uint32_t raw_count = count;

//         // ===== 70カウント捨てる処理 =====
//         if (raw_count < 70) {
//             M5.Imu.update();

//             // 1行分作成
//             char line[128];
//             int len = snprintf(line, sizeof(line),
//                 "%lu,0,0,0,0,0,0,0,0,0,1\r\n", raw_count);

//             // バッファ容量をチェック
//             if (dataIndex + len < BUFSIZE){
//                 memcpy(&dataBuf[recBufNum][dataIndex], line, len);
//                 dataIndex += len;
//             } else {
//                 bufReady = true;
//                 // SDタスクに通知
//                 xTaskNotifyGive(sdTaskHandle);
//             }

//             // ヘッダー直後に戻す仕様（あなたの仕様そのまま）
//             logFile.seek(headerLen);
//             continue;
//         }

//         // ===== IMU実データ =====
//         uint32_t rec_count = raw_count - 70;

//         M5.Imu.update();
//         float ax, ay, az, gx, gy, gz, mx, my, mz;
//         M5.Imu.getAccel(&ax, &ay, &az);
//         M5.Imu.getGyro(&gx, &gy, &gz);
//         M5.Imu.getMag(&mx, &my, &mz);

//         char line[128];
//         int len = snprintf(line, sizeof(line),
//             "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
//             rec_count, ax, ay, az, gx, gy, gz, mx, my, mz);

//         // ------- バッファに書く -------
//         if (dataIndex + len < BUFSIZE) {
//             memcpy(&dataBuf[recBufNum][dataIndex], line, len);
//             dataIndex += len;
//         } else {
//             bufReady = true;
//             // 別タスクにSD書き込みを依頼
//             xTaskNotifyGive(sdTaskHandle);
//         }
//     }
// }

// // =======================================
// //  ★ SD書き込みタスク（低優先）
// // =======================================
// void sdTask(void *pv){
//     for(;;) {

//         // 通知を待つ
//         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//         if (!bufReady) continue;
//         bufReady = false;

//         // どちらのバッファを書き込むか
//         int toSD = recBufNum;
//         int len  = dataIndex;

//         // バッファ切り替え
//         recBufNum ^= 1;
//         dataIndex = 0;

//         // ---- SDに書く（ここは遅くてもOK） ----
//         if (cur_state == Recording){
//             logFile.write((uint8_t*)dataBuf[toSD], len);
//         }
//     }
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

//   // ★ IMUタスク：最優先（優先度5）
//   xTaskCreatePinnedToCore(
//       imuTask,
//       "IMU_TASK",
//       4096,
//       nullptr,
//       5,              // ←最優先
//       &imuTaskHandle,
//       0
//   );

//   // ★ SD書き込みタスク：低優先度（1）
//   xTaskCreatePinnedToCore(
//       sdTask,
//       "SD_TASK",
//       4096,
//       nullptr,
//       1,
//       &sdTaskHandle,
//       1
//   );

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


