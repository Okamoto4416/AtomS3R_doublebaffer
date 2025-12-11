#include "ODRsetting.h"
static constexpr uint32_t I2C_HZ = 100000;  // 安全に 100kHzで

static bool busWrite8(m5::I2C_Class& bus, uint8_t addr7, uint8_t reg,
                      uint8_t val) {
    uint8_t buf[2] = {reg, val};
    if (!bus.start(addr7, /*read=*/false, I2C_HZ)) return false;
    if (!bus.write(buf, 2)) {
        bus.stop();
        return false;
    }
    bus.stop();
    return true;
}

static bool busRead8(m5::I2C_Class& bus, uint8_t addr7, uint8_t reg,
                     uint8_t* out) {
    if (!bus.start(addr7, /*read=*/false, I2C_HZ)) {
        return false;
    }

    if (!bus.write(&reg, 1)) {
        bus.stop();
        return false;
    }

    bus.stop();

    if (!bus.start(addr7, /*read=*/true, I2C_HZ)) {
        return false;
    }

    if (!bus.read(out, 1, /*last_nack=*/true)) {
        bus.stop();
        return false;
    }

    bus.stop();
    return true;
}

static bool busPing(m5::I2C_Class& bus, uint8_t addr7) {
    if (!bus.start(addr7, /*read=*/false, I2C_HZ)) {
        return false;
    }

    bus.stop();
    return true;
}

// ====== BMI270 ======
namespace BMI270 {
    static constexpr uint8_t ADDR = 0x68;      // or 0x69
    static constexpr uint8_t CHIP_ID = 0x00;   // 期待値 0x24
    static constexpr uint8_t PAGE_SEL = 0x7F;  // 0x00 で page0
    static constexpr uint8_t ACC_CONF = 0x40;  // 下位4bit = ODR
    static constexpr uint8_t GYR_CONF = 0x42;  // 下位4bit = ODR

    static int decodeAcc(uint8_t v) {
        switch (v & 0x0F) {
            case 0x05:
                return 12;
            case 0x06:
                return 25;
            case 0x07:
                return 50;
            case 0x08:
                return 100;
            case 0x09:
                return 200;
            case 0x0A:
                return 400;
            case 0x0B:
                return 800;
            case 0x0C:
                return 1600;
            default:
                return -1;
        }
    }

    static int decodeGyr(uint8_t v) {
        switch (v & 0x0F) {
            case 0x06:
                return 25;
            case 0x07:
                return 50;
            case 0x08:
                return 100;
            case 0x09:
                return 200;
            case 0x0A:
                return 400;
            case 0x0B:
                return 800;
            case 0x0C:
                return 1600;
            case 0x0D:
                return 3200;
            default:
                return -1;
        }
    }

    // ODR読み
    static bool readODR(m5::I2C_Class& bus, uint8_t addr) {
        // WHO_AM_I
        uint8_t id = 0xFF;
        if (!busRead8(bus, addr, CHIP_ID, &id)) {
            return false;
        }

        if (id != 0x24) {
            return false;
        }

        // page0 固定
        if (!busWrite8(bus, addr, PAGE_SEL, 0x00)) {
            return false;
        }

        // 読み出し
        uint8_t acc_conf = 0xFF, gyr_conf = 0xFF;

        if (!busRead8(bus, addr, ACC_CONF, &acc_conf)) {
            return false;
        }

        if (!busRead8(bus, addr, GYR_CONF, &gyr_conf)) {
            return false;
        }

        Serial.printf("[BMI270@0x%02X] ACC_CONF=0x%02X -> Acc ODR=%d Hz\n", addr,
                    acc_conf, decodeAcc(acc_conf));
        Serial.printf("[BMI270@0x%02X] GYR_CONF=0x%02X -> Gyr ODR=%d Hz\n", addr,
                    gyr_conf, decodeGyr(gyr_conf));

        return true;
    }
}  // namespace BMI270


static bool BMI270_writeODR(m5::I2C_Class& bus, uint8_t addr, uint8_t acc_odr_code,
                     uint8_t gyr_odr_code) {
    uint8_t acc_conf = 0, gyr_conf = 0;
    busWrite8(bus, addr, 0x7F, 0x00);  // page 0

    if (!busRead8(bus, addr, 0x40, &acc_conf)) return false;
    if (!busRead8(bus, addr, 0x42, &gyr_conf)) return false;

    acc_conf = (acc_conf & 0xF0) | (acc_odr_code & 0x0F);
    gyr_conf = (gyr_conf & 0xF0) | (gyr_odr_code & 0x0F);

    busWrite8(bus, addr, 0x40, acc_conf);
    busWrite8(bus, addr, 0x42, gyr_conf);
    delay(5);
    return true;
}

// ====== メイン ======
static bool tryReadOnBus(const char* name, m5::I2C_Class& bus) {
    Serial.printf("Scanning %s...\n", name);

    // 候補アドレス（0x68 / 0x69）を軽く ping
    bool seen68 = busPing(bus, 0x68);
    // bool seen69 = busPing(bus, 0x69);
    //if (!seen68 && !seen69) {
    if (!seen68 ) {
        Serial.println("  no IMU address responds.");
        return false;
    }

    // BMI270 を判定（WHO_AM_I=0x24, CHIP_ID @ 0x00）
    if (seen68 && BMI270::readODR(bus, 0x68)) {
        return true;
    }

    // if (seen69 && BMI270::readODR(bus, 0x69)) {
    //     return true;
    // }

    // // 次に MPU6886（WHO_AM_I=0x19, @0x75）
    // if (seen68 && MPU6886::readODR(bus, 0x68)) {
    //     return true;
    // }

    // ここまで届かなければ失敗
    Serial.println("  IMU not identified on this bus.");
    return false;
}  // namespace BMI270

bool ODRset() {
    Serial.println("-- set 200Hz ---");
    // Acc=200Hz, Gyr=200Hz
    if (!BMI270_writeODR(M5.In_I2C, 0x68, 0x09, 0x09)) {
        Serial.println("ODR write failed");
        return false;
    }  
    
    if (tryReadOnBus("In_I2C", M5.In_I2C) == false) {
        return false;
    }

    return true;
}
