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

// bufLen[i] : i 番目バッファの確定長（producer が満杯にしたときに設定）bufLenArr[0]→バッファ0に入っているデータの数 bufLenArr[1]→バッファ1に入っているデータの数
//→SDへの書き込みはlogFile.Write(buf,buflen)という形でbufの長さを入れる必要があるから
volatile int bufLenArr[2] = {0, 0};

// bufReadyMask のビット i が立っていれば i 番目バッファが SD 書き込み待ち(書き込み待ちなら1 →0b00=0:書き込み待ち無し 0b01=1:buf0が書き込み待ち 0b10=2:buf1が書き込み待ち)
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

// 状態
enum State{ Setup, Standby, Recording, Finished };
State cur_state;
State prev_state;

// サンプリングカウント（ISR がインクリメント）
volatile uint32_t countSample = 0;

// -------------------- ユーティリティ --------------------
// フラグを立てる関数（uini8_t版に書き直す）→SDへの書き込みフラグのため
static inline uint8_t atomic_fetch_or_u8(volatile uint8_t* p, uint8_t v) {
    return __atomic_fetch_or(p, v, __ATOMIC_SEQ_CST);
}
static inline uint8_t atomic_fetch_and_u8(volatile uint8_t* p, uint8_t v) {
    return __atomic_fetch_and(p, v, __ATOMIC_SEQ_CST);
}

// -------------------- タイマーISR（超軽量） --------------------
void IRAM_ATTR onTimer() {
    // カウント増やしてIMUタスクを起こすだけ
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
        // ISR からの通知を待つ（5msごとに通知）
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 状態が Recording でなければ何もしない
        if (cur_state != Recording) continue;

        // 1) サンプルカウント取得（ローカルコピー）
        uint32_t raw_count = countSample;

        // 2) 通常サンプル処理
        uint32_t rec_count = raw_count;

        // IMU読み出し
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

        // 3) ダブルバッファへ書き込み（必ず行う設計）
        if (len <= 0) {
            continue; // 念のため
        }

        if (dataIndex + len < BUFSIZE) {

            // 空きがあるならコピー
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
            } 

            // 書き込み待ちフラグを立てる（atomic）0b00000001ならバッファ0にフラグ、0b00000010ならバッファ1にフラグ
            //1 << toSD：2のtoSD乗→ビット移動～0001か～0010の形にする（toSDに依存）
            atomic_fetch_or_u8(&bufReadyMask, (1 << toSD));

            // sdTask に通知して書き込みを促す（非ブロッキング）
            xTaskNotifyGive(sdTaskHandle);
        }
    } 
}

// -------------------- SD書き込みタスク（低優先） --------------------
void sdTask(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        // sdTask は通知されるまで待機（通知は imuTask が設定する）
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // もし state が Recording でないなら無視
        if (cur_state != Recording){
            continue;
        } 

        // 連続して書き待ちバッファがある限り処理する
        while (true) {

            // 切り替わった瞬間のフラグをコピー→割り込みで変わってしまうから判別用のものを別に用意
            uint8_t mask_snapshot = __atomic_load_n(&bufReadyMask, __ATOMIC_SEQ_CST);
            if (mask_snapshot == 0){

                break; // 書き待ち無し
            } 

            // SDに書き込むバッファを判断する
            int toWrite = -1;
            if (mask_snapshot & 0x01){
                toWrite = 0;
            } else if(mask_snapshot & 0x02) {
                toWrite = 1;
            } 

            if (toWrite < 0){
                break;
            } 

            // フラグを下ろす（towriteのbitを0にする）
            atomic_fetch_and_u8(&bufReadyMask, (uint8_t)~(1 << toWrite));

            // Read length atomically (producer wrote it before setting mask)
            int len = bufLenArr[toWrite];

            // 安全策
            if (len <= 0){
              continue;  
            } 
            if (len > BUFSIZE){
              continue;  
            } 

            // バッファをSDに書き込み
            if (logFile) {
                size_t w = logFile.write((uint8_t*)dataBuf[toWrite], (size_t)len);
                (void)w; //wはこの分岐ないに存在しているものであり、本来ななら使ってはいけない→コンパイラに unused-variable warning を出させない処理
            }
        } 
    } 
}

// -------------------- 最後の残りデータをSDに書く関数 --------------------
void flushRemainingBuffer()
{
    //  Recording 以外では実行しない
    if (cur_state != Recording){
        return;
    } 

    // 今 producer が書いていたバッファ番号
    int lastBuf = recBufNum;

    // 今の dataIndex がそのバッファの現在サイズ
    int lastLen = dataIndex;

    // もし残りが 1byte 以上あれば、それを書き待ちにする
    if (lastLen > 0) {
        bufLenArr[lastBuf] = lastLen;

        // bufReadyMask に書き待ちビットを立てる
        atomic_fetch_or_u8(&bufReadyMask, (1 << lastBuf));

        // sdTaskをたたく
        xTaskNotifyGive(sdTaskHandle);
    }
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
            timerWrite(timer, 0);
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

    // ODR 設定
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

    if (!Led_Init()) { 
        Serial.println("LED初期化失敗"); check = false; 
    }
    if (!Triger_Init()) { 
        Serial.println("トリガー初期化失敗"); check = false; 
    }
    ADC_Init();
    if (!checkBattery()) { 
        check = false; 
    }

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

//オリジナルバージョン
// -------------------- loop（状態遷移の処理） --------------------
// void loop() {
//     // entry処理
//     if (cur_state != prev_state) {
//         Serial.printf("[ENTRY] %d -> %d\n", prev_state, cur_state);
//         onEntry(cur_state);
//         prev_state = cur_state;
//     }

//     switch (cur_state) {
//       case Standby:
//         // トリガのチェック（あなたの既存関数）
//         if (Is_Triger()) {
//             prev_state = cur_state;
//             cur_state = Recording;
//             break;
//         }
//         if (!checkBattery()) {
//             Led_Warning();
//             prev_state = cur_state;
//             cur_state = Finished;
//             break;
//         }
//         if (IsButton()) {
//             if (logFile) logFile.close();
//             Led_Warning();
//             prev_state = cur_state;
//             cur_state = Finished;
//             return;
//         }
//         break;

//       case Recording:
//         // SD 書き出しは sdTask が担当し、imuTask が bufReadyMask をセットすると通知している
//         // loop は軽い状態遷移のみ行う

//         // 記録終了判定
//         if (Is_Triger() == TRIGER_ON || countSample > 60000) {
//             Serial.println("記録終了");
//             if (timer) {
//                 timerAlarmDisable(timer);
//                 timerStop(timer);
//             }
            
//             // sdTask にも書き込みを促して残りを吐かせる
//             flushRemainingBuffer();

//             //xTaskNotifyGive(sdTaskHandle);
//             // 少し待ってファイル閉じ（SDが遅い場合は待ち時間増やす）
//             while (__atomic_load_n(&bufReadyMask, __ATOMIC_SEQ_CST) != 0) {
//                 vTaskDelay(pdMS_TO_TICKS(1));
//             }
//             closeFile();
//             prev_state = cur_state;
//             cur_state = Standby;
//         }
//         break;

//       case Finished:
//         // 何もしない
//         break;
//     }

// }


//whileバージョン
void loop() {
    // entry処理
    if (cur_state != prev_state) {
        Serial.printf("[ENTRY] %d -> %d\n", prev_state, cur_state);
        onEntry(cur_state);
        prev_state = cur_state;
    }

    switch (cur_state) {
      case Standby:
        while (true){
            // トリガのチェック（あなたの既存関数）
            if (digitalRead(PIN_TRIGER) == HIGH) {
                delayMicroseconds(50);
                if (digitalRead(PIN_TRIGER) == HIGH) {
                    prev_state = cur_state;
                    cur_state = Recording;
                    break;
                }
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
                break;
            }
        }
        break;
        // // トリガのチェック（あなたの既存関数）
        // if (Is_Triger()) {
        //     prev_state = cur_state;
        //     cur_state = Recording;
        //     break;
        // }
        // if (!checkBattery()) {
        //     Led_Warning();
        //     prev_state = cur_state;
        //     cur_state = Finished;
        //     break;
        // }
        // if (IsButton()) {
        //     if (logFile) logFile.close();
        //     Led_Warning();
        //     prev_state = cur_state;
        //     cur_state = Finished;
        //     break;
        // }
        // break;

      case Recording:
        // SD 書き出しは sdTask が担当し、imuTask が bufReadyMask をセットすると通知している
        // loop は軽い状態遷移のみ行う

        // 記録終了判定
        while(true){
            if (digitalRead(PIN_TRIGER) == HIGH) {
                delayMicroseconds(50);
                if (digitalRead(PIN_TRIGER) == HIGH) {
                    break;
                }
            }else if (countSample > 60000){
                break;
            }
        }
        Serial.println("記録終了");
        if (timer) {
            timerAlarmDisable(timer);
            timerStop(timer);
        }

        // sdTask にも書き込みを促して残りを吐かせる
        flushRemainingBuffer();

        //xTaskNotifyGive(sdTaskHandle);
        // 少し待ってファイル閉じ（SDが遅い場合は待ち時間増やす）
        while (__atomic_load_n(&bufReadyMask, __ATOMIC_SEQ_CST) != 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        closeFile();
        prev_state = cur_state;
        cur_state = Standby;        

        // if (Is_Triger() == TRIGER_ON || countSample > 60000) {
        //     Serial.println("記録終了");
        //     if (timer) {
        //         timerAlarmDisable(timer);
        //         timerStop(timer);
        //     }
            
        //     // sdTask にも書き込みを促して残りを吐かせる
        //     flushRemainingBuffer();

        //     //xTaskNotifyGive(sdTaskHandle);
        //     // 少し待ってファイル閉じ（SDが遅い場合は待ち時間増やす）
        //     while (__atomic_load_n(&bufReadyMask, __ATOMIC_SEQ_CST) != 0) {
        //         vTaskDelay(pdMS_TO_TICKS(1));
        //     }
        //     closeFile();
        //     prev_state = cur_state;
        //     cur_state = Standby;
        // }
        break;

      case Finished:
        // 何もしない
        break;
    }

}