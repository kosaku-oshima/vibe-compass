#include <Wire.h>
#include <math.h>
const int MOTOR_PIN = 25;

// ==========================
// 補正・北方向登録用の変数
// ==========================
float offsetX = 0.0;
float offsetY = 0.0;

float northHeading = 0.0;
bool northHeadingSet = false;

// 北とみなす範囲
// 登録した北方向から±30度以内なら NORTH! と判定
const float NORTH_RANGE = 30.0;

// ==========================
// ESP32のI2Cピン
// ==========================
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// ==========================
// AE-BM1422AGMV の I2Cアドレス
// ==========================
const uint8_t BM1422_ADDR = 0x0E;

// ==========================
// BM1422AGMV の主なレジスタ
// ==========================
const uint8_t REG_WIA   = 0x0F; // Who am I
const uint8_t REG_DATAX = 0x10; // X/Y/Zデータ開始位置
const uint8_t REG_STA1  = 0x18; // 測定完了ステータス
const uint8_t REG_CNTL1 = 0x1B;
const uint8_t REG_CNTL2 = 0x1C;
const uint8_t REG_CNTL3 = 0x1D;
const uint8_t REG_AVE_A = 0x40;
const uint8_t REG_CNTL4 = 0x5C;

// ==========================
// 設定値
// ==========================
const uint8_t WIA_EXPECTED = 0x41;
const uint8_t CNTL1_VALUE  = 0xC2; // 14bit出力、電源ONなど
const uint8_t CNTL2_VALUE  = 0x08; // DRDY有効
const uint8_t CNTL3_FORCE  = 0x40; // 1回測定開始
const uint8_t AVE_A_VALUE  = 0x00;

// ROHMのArduinoライブラリでは14bit時に raw / 24 で uT換算している
const float SENSITIVITY_14BIT = 24.0;

// ==========================
// I2C書き込み・読み取り関数
// ==========================
bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BM1422_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool writeRegister16(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(BM1422_ADDR);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(BM1422_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received = Wire.requestFrom(BM1422_ADDR, length);

  if (received != length) {
    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }

  return true;
}

bool readRegister(uint8_t reg, uint8_t &value) {
  return readRegisters(reg, &value, 1);
}

// ==========================
// 角度を0〜360度に整える関数
// ==========================
float normalizeHeading(float angle) {
  while (angle < 0.0) {
    angle += 360.0;
  }

  while (angle >= 360.0) {
    angle -= 360.0;
  }

  return angle;
}

// ==========================
// 2つの角度の差を -180〜180度で返す関数
// 例：350度と10度の差を340度ではなく-20度として扱う
// ==========================
float angleDifference(float a, float b) {
  float diff = a - b;

  while (diff > 180.0) {
    diff -= 360.0;
  }

  while (diff < -180.0) {
    diff += 360.0;
  }

  return diff;
}

// ==========================
// BM1422AGMV初期化
// ==========================
bool initBM1422() {
  uint8_t whoAmI = 0;

  if (!readRegister(REG_WIA, whoAmI)) {
    Serial.println("BM1422AGMVにアクセスできません。");
    return false;
  }

  Serial.print("WIA register = 0x");
  Serial.println(whoAmI, HEX);

  if (whoAmI != WIA_EXPECTED) {
    Serial.println("想定したBM1422AGMVではない可能性があります。");
    return false;
  }

  if (!writeRegister(REG_CNTL1, CNTL1_VALUE)) {
    Serial.println("CNTL1の設定に失敗しました。");
    return false;
  }

  delay(1);

  if (!writeRegister16(REG_CNTL4, 0x0000)) {
    Serial.println("CNTL4の設定に失敗しました。");
    return false;
  }

  if (!writeRegister(REG_CNTL2, CNTL2_VALUE)) {
    Serial.println("CNTL2の設定に失敗しました。");
    return false;
  }

  if (!writeRegister(REG_AVE_A, AVE_A_VALUE)) {
    Serial.println("AVE_Aの設定に失敗しました。");
    return false;
  }

  Serial.println("BM1422AGMV init OK");
  return true;
}

// ==========================
// 地磁気データ読み取り
// ==========================
bool readMagneticData(float &x, float &y, float &z) {
  // 1回測定を開始
  if (!writeRegister(REG_CNTL3, CNTL3_FORCE)) {
    Serial.println("測定開始に失敗しました。");
    return false;
  }

  // 測定完了を待つ
  uint8_t status = 0;
  bool dataReady = false;

  for (int i = 0; i < 50; i++) {
    if (!readRegister(REG_STA1, status)) {
      return false;
    }

    if (status & 0x40) {
      dataReady = true;
      break;
    }

    delay(1);
  }

  if (!dataReady) {
    Serial.println("測定完了待ちでタイムアウトしました。");
    return false;
  }

  uint8_t data[6];

  if (!readRegisters(REG_DATAX, data, 6)) {
    Serial.println("磁気データの読み取りに失敗しました。");
    return false;
  }

  int16_t rawX = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
  int16_t rawY = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
  int16_t rawZ = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

  x = rawX / SENSITIVITY_14BIT;
  y = rawY / SENSITIVITY_14BIT;
  z = rawZ / SENSITIVITY_14BIT;

  return true;
}

// ==========================
// X/Yの中心ズレを補正するためのキャリブレーション
// ==========================
void calibrateMagnetometer() {
  digitalWrite(MOTOR_PIN, LOW);
  Serial.println();
  Serial.println("Calibration start");
  Serial.println("10秒間、センサーを水平にしてゆっくり一周以上回してください。");

  float minX = 99999.0;
  float maxX = -99999.0;
  float minY = 99999.0;
  float maxY = -99999.0;

  unsigned long startTime = millis();

  while (millis() - startTime < 10000) {
    float x, y, z;

    if (readMagneticData(x, y, z)) {
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;

      Serial.print("calibrating... X:");
      Serial.print(x, 2);
      Serial.print(" Y:");
      Serial.println(y, 2);
    }

    delay(100);
  }

  offsetX = (maxX + minX) / 2.0;
  offsetY = (maxY + minY) / 2.0;

  Serial.println("Calibration done");
  Serial.print("offsetX = ");
  Serial.println(offsetX, 2);
  Serial.print("offsetY = ");
  Serial.println(offsetY, 2);

  Serial.println();
  Serial.println("次に、基板の凸部を北に向けてください。");
  Serial.println("向けた状態で、Serial Monitorに n を送ると、その向きを北として登録します。");
  Serial.println("補正をやり直したい場合は c を送ってください。");
}

// ==========================
// setup
// ==========================
void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
  delay(1000);

  Serial.println();
  Serial.println("Compass motor vibration test start");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!initBM1422()) {
    Serial.println("初期化に失敗しました。配線を確認してください。");
  } else {
    calibrateMagnetometer();
  }
}

// ==========================
// loop
// ==========================
void loop() {
  float x, y, z;

  if (readMagneticData(x, y, z)) {
    float correctedX = x - offsetX;
    float correctedY = y - offsetY;

    float heading = atan2(correctedY, correctedX) * 180.0 / PI;
    heading = normalizeHeading(heading);

    // Serial Monitorからのコマンド処理
    if (Serial.available() > 0) {
      char command = Serial.read();

      // nを送ると、現在の向きを北として登録する
      if (command == 'n' || command == 'N') {
        northHeading = heading;
        northHeadingSet = true;

        Serial.println();
        Serial.println("North heading registered.");
        Serial.print("northHeading = ");
        Serial.print(northHeading, 1);
        Serial.println(" deg");
      }

      // cを送ると、キャリブレーションをやり直す
      if (command == 'c' || command == 'C') {
        calibrateMagnetometer();
        northHeadingSet = false;

        Serial.println();
        Serial.println("Calibration was redone.");
        Serial.println("基板の凸部を北に向けて、Serial Monitorに n を送ってください。");

        // キャリブレーション直後の古いheadingで判定しないように、いったんloopを抜ける
        delay(500);
        return;
      }
    }

    bool isNorth = false;
    float diffFromNorth = 0.0;

    if (northHeadingSet) {
      diffFromNorth = angleDifference(heading, northHeading);
      isNorth = fabs(diffFromNorth) <= NORTH_RANGE;
    }

    if (northHeadingSet && isNorth) {
      digitalWrite(MOTOR_PIN, HIGH);
    } else {
      digitalWrite(MOTOR_PIN, LOW);
    }

    Serial.print("X: ");
    Serial.print(x, 2);
    Serial.print(" uT, Y: ");
    Serial.print(y, 2);
    Serial.print(" uT, Z: ");
    Serial.print(z, 2);

    Serial.print(" uT, Heading(cal): ");
    Serial.print(heading, 1);
    Serial.print(" deg");

    Serial.print(", northHeading: ");
    if (northHeadingSet) {
      Serial.print(northHeading, 1);
      Serial.print(" deg");
    } else {
      Serial.print("not set");
    }

    Serial.print(", diff: ");
    Serial.print(diffFromNorth, 1);
    Serial.print(" deg, ");

    if (!northHeadingSet) {
      Serial.println("Please set north with n");
    } else if (isNorth) {
      Serial.println("NORTH!");
    } else {
      Serial.println("not north");
    }
  }

  delay(500);
}