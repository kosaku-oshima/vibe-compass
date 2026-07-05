/*
  vibe-compass v0.3.2
  BM1422AGMV対応 + nキー北登録 + パルス振動 + 反応速度改善 + 連続振動からの復帰改善版

  目的:
  - 起動後10秒間、地磁気センサーを回して簡易キャリブレーションする
  - 地磁気センサー基板の「凸部分」を北へ向けた状態で n + Enter を送る
  - その瞬間の方位を「北」として登録する
  - 以後、登録した北方向に近づくほど振動間隔を短くする
  - 真北付近では連続振動する
  - 真北付近から外れたら、すぐ断続振動へ戻る

  想定配線:
  - AE-BM1422AGMV VIN -> 3V3
  - AE-BM1422AGMV GND -> GND
  - AE-BM1422AGMV SDA -> GPIO21
  - AE-BM1422AGMV SCL -> GPIO22

  - ESP32 GPIO25 -> R1 1kΩ -> 2SC1815 ベース
  - モーターは2SC1815でローサイド駆動
  - フライバックダイオードあり

  Serial Monitor:
  - 通信速度: 115200
  - n + Enter: 現在の向きを「北」として登録
  - r + Enter: 北登録をリセット
  - c + Enter: 10秒キャリブレーションをやり直す

  振動パターン:
  - 0〜5度: 連続振動
  - 5〜10度: かなり小刻みに振動
  - 10〜20度: 中くらいの間隔で振動
  - 20〜30度: ゆっくり振動
  - 35度以上: 停止

  v0.3.2での変更:
  - 振動ゾーンを明示的に管理
  - CONTINUOUS から PULSE_FAST / PULSE_MEDIUM / PULSE_SLOW に移った瞬間、すぐ断続振動を再開
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// =====================
// ピン設定
// =====================
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int MOTOR_PIN = 25;

// =====================
// PWM設定
// =====================
const int PWM_FREQ = 20000;      // 20kHz
const int PWM_RESOLUTION = 8;    // 0〜255
const int PWM_MAX = 255;
const int MOTOR_PWM_CHANNEL = 0; // Arduino-ESP32 2.x系用

// =====================
// BM1422AGMV設定
// =====================
const uint8_t BM1422_ADDR_0E = 0x0E;
const uint8_t BM1422_ADDR_0F = 0x0F;

uint8_t bm1422Address = 0x00;

// レジスタ
const uint8_t BM1422_WIA   = 0x0F;
const uint8_t BM1422_DATAX = 0x10;
const uint8_t BM1422_CNTL1 = 0x1B;
const uint8_t BM1422_CNTL2 = 0x1C;
const uint8_t BM1422_CNTL3 = 0x1D;
const uint8_t BM1422_AVE_A = 0x40;
const uint8_t BM1422_CNTL4 = 0x5C;

// WHO AM I の期待値
const uint8_t BM1422_WIA_VAL = 0x41;

// 制御値
const uint8_t BM1422_CNTL1_FS1     = (1 << 1);
const uint8_t BM1422_CNTL1_OUT_BIT = (1 << 6);
const uint8_t BM1422_CNTL1_PC1     = (1 << 7);
const uint8_t BM1422_CNTL2_DREN    = (1 << 3);
const uint8_t BM1422_CNTL3_FORCE   = (1 << 6);

const uint8_t BM1422_CNTL1_VAL = BM1422_CNTL1_FS1 | BM1422_CNTL1_OUT_BIT | BM1422_CNTL1_PC1;
const uint8_t BM1422_CNTL2_VAL = BM1422_CNTL2_DREN;
const uint8_t BM1422_CNTL3_VAL = BM1422_CNTL3_FORCE;
const uint8_t BM1422_AVE_A_VAL = 0x00;

// 14bit出力時の感度。
// raw / 24 = uT
const float BM1422_14BIT_SENS = 24.0;

// =====================
// 方位・振動範囲設定
// =====================

// 登録した北から30度以内で振動開始
const float VIBRATION_ON_DEGREE = 30.0;

// 一度振動した後は、35度以上離れたら停止
// 30度付近でON/OFFが細かく切り替わるのを防ぐ
const float VIBRATION_OFF_DEGREE = 35.0;

// 真北とみなして連続振動する範囲
const float CONTINUOUS_VIBRATION_DEGREE = 5.0;

// =====================
// 振動パターン設定
// =====================

// 強弱ではなく「振動の間隔」で方向を伝える。
// ON時の強さは基本的に一定にする。
const int DUTY_STOP = 0;
const int DUTY_ON = 220;

// パルス振動のON時間。
const unsigned long PULSE_ON_MS = 100;

// 北からの距離に応じたOFF時間。
// 北に近づくほどOFF時間を短くし、小刻みに振動させる。
const unsigned long PULSE_OFF_SLOW_MS = 450;   // 20〜30度
const unsigned long PULSE_OFF_MEDIUM_MS = 220; // 10〜20度
const unsigned long PULSE_OFF_FAST_MS = 80;    // 5〜10度

// =====================
// センサー向き補正
// =====================
// n登録で「凸部分を北に向けた姿勢」を基準化するので、通常はこのままでOKです。
// ただし、方位の増減方向が明らかにおかしい場合だけ調整します。
const bool SWAP_XY = false;
const int X_SIGN = 1;
const int Y_SIGN = 1;

// =====================
// 簡易キャリブレーション設定
// =====================
const unsigned long CALIBRATION_TIME_MS = 10000;

float xOffset = 0.0;
float yOffset = 0.0;
float xScale = 1.0;
float yScale = 1.0;

// =====================
// 北登録
// =====================
float registeredNorthHeading = 0.0;
bool northRegistered = false;

// =====================
// 移動平均
// =====================
// 反応速度を優先して4回平均
const int HEADING_HISTORY_SIZE = 4;
float headingHistory[HEADING_HISTORY_SIZE];
int headingHistoryIndex = 0;
bool headingHistoryFilled = false;

// =====================
// 振動ゾーン
// =====================
enum VibrationZone {
  ZONE_OFF,
  ZONE_SLOW,
  ZONE_MEDIUM,
  ZONE_FAST,
  ZONE_CONTINUOUS
};

// =====================
// 状態管理
// =====================
bool vibrationActive = false;

// パルス振動制御用
bool pulseMotorOn = false;
unsigned long lastPulseSwitchMillis = 0;

// 前回の振動ゾーン。
// CONTINUOUSからPULSEへ戻る瞬間を検出するために使う。
VibrationZone lastVibrationZone = ZONE_OFF;

unsigned long lastPrintMillis = 0;
const unsigned long PRINT_INTERVAL_MS = 300;

unsigned long lastGuideMillis = 0;
const unsigned long GUIDE_INTERVAL_MS = 1000;

// 直近の方位。n登録時に使う。
float latestHeading = 0.0;
float latestSmoothedHeading = 0.0;
bool latestHeadingValid = false;


// =====================
// I2C基本処理
// =====================

bool i2cDeviceExists(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool writeRegister8(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool writeRegisterBytes(uint8_t address, uint8_t reg, const uint8_t *data, int size) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  for (int i = 0; i < size; i++) {
    Wire.write(data[i]);
  }
  return Wire.endTransmission() == 0;
}

bool readRegisterBytes(uint8_t address, uint8_t reg, uint8_t *data, int size) {
  Wire.beginTransmission(address);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  int bytesRead = Wire.requestFrom((int)address, size, true);
  if (bytesRead != size) {
    return false;
  }

  for (int i = 0; i < size; i++) {
    data[i] = Wire.read();
  }

  return true;
}

bool readRegister8(uint8_t address, uint8_t reg, uint8_t &value) {
  return readRegisterBytes(address, reg, &value, 1);
}


// =====================
// BM1422AGMV初期化
// =====================

bool tryInitBM1422AtAddress(uint8_t address) {
  if (!i2cDeviceExists(address)) {
    return false;
  }

  uint8_t whoAmI = 0x00;
  if (!readRegister8(address, BM1422_WIA, whoAmI)) {
    return false;
  }

  Serial.print("I2C address 0x");
  Serial.print(address, HEX);
  Serial.print(" WHO_AM_I = 0x");
  Serial.println(whoAmI, HEX);

  if (whoAmI != BM1422_WIA_VAL) {
    return false;
  }

  if (!writeRegister8(address, BM1422_CNTL1, BM1422_CNTL1_VAL)) {
    Serial.println("ERROR: CNTL1 write failed");
    return false;
  }

  delay(1);

  uint8_t cntl4Data[2] = {0x00, 0x00};
  if (!writeRegisterBytes(address, BM1422_CNTL4, cntl4Data, 2)) {
    Serial.println("ERROR: CNTL4 write failed");
    return false;
  }

  if (!writeRegister8(address, BM1422_CNTL2, BM1422_CNTL2_VAL)) {
    Serial.println("ERROR: CNTL2 write failed");
    return false;
  }

  if (!writeRegister8(address, BM1422_AVE_A, BM1422_AVE_A_VAL)) {
    Serial.println("ERROR: AVE_A write failed");
    return false;
  }

  bm1422Address = address;
  return true;
}

bool initBM1422() {
  Serial.println("Searching BM1422AGMV...");

  if (tryInitBM1422AtAddress(BM1422_ADDR_0F)) {
    Serial.println("BM1422AGMV detected at 0x0F");
    return true;
  }

  if (tryInitBM1422AtAddress(BM1422_ADDR_0E)) {
    Serial.println("BM1422AGMV detected at 0x0E");
    return true;
  }

  Serial.println("ERROR: BM1422AGMV not found");
  return false;
}


// =====================
// BM1422AGMV読み取り
// =====================

bool readBM1422(float &x_uT, float &y_uT, float &z_uT) {
  if (bm1422Address == 0x00) {
    return false;
  }

  if (!writeRegister8(bm1422Address, BM1422_CNTL3, BM1422_CNTL3_VAL)) {
    return false;
  }

  delay(2);

  uint8_t data[6];
  if (!readRegisterBytes(bm1422Address, BM1422_DATAX, data, 6)) {
    return false;
  }

  int16_t rawX = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
  int16_t rawY = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
  int16_t rawZ = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

  x_uT = (float)rawX / BM1422_14BIT_SENS;
  y_uT = (float)rawY / BM1422_14BIT_SENS;
  z_uT = (float)rawZ / BM1422_14BIT_SENS;

  return true;
}


// =====================
// PWM / モーター制御
// =====================

void setupMotorPwm() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  bool pwmOk = ledcAttach(MOTOR_PIN, PWM_FREQ, PWM_RESOLUTION);
  if (!pwmOk) {
    Serial.println("ERROR: ledcAttach failed");
  }
#else
  ledcSetup(MOTOR_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_PIN, MOTOR_PWM_CHANNEL);
#endif
}

void setMotorDuty(int duty) {
  duty = constrain(duty, 0, PWM_MAX);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PIN, duty);
#else
  ledcWrite(MOTOR_PWM_CHANNEL, duty);
#endif
}


// =====================
// 方位計算
// =====================

float normalizeHeading(float heading) {
  while (heading < 0.0) {
    heading += 360.0;
  }

  while (heading >= 360.0) {
    heading -= 360.0;
  }

  return heading;
}

float calculateHeadingDegree(float x, float y) {
  // 簡易キャリブレーション補正
  x = (x - xOffset) * xScale;
  y = (y - yOffset) * yScale;

  // 基板の向き調整
  if (SWAP_XY) {
    float temp = x;
    x = y;
    y = temp;
  }

  x = X_SIGN * x;
  y = Y_SIGN * y;

  float heading = atan2(y, x) * 180.0 / PI;
  return normalizeHeading(heading);
}

// 角度の平均化。
// 359度と1度の平均が180度にならないように、sin/cosで平均する。
float smoothHeading(float newHeading) {
  headingHistory[headingHistoryIndex] = newHeading;
  headingHistoryIndex++;

  if (headingHistoryIndex >= HEADING_HISTORY_SIZE) {
    headingHistoryIndex = 0;
    headingHistoryFilled = true;
  }

  int count = headingHistoryFilled ? HEADING_HISTORY_SIZE : headingHistoryIndex;

  float sinSum = 0.0;
  float cosSum = 0.0;

  for (int i = 0; i < count; i++) {
    float rad = headingHistory[i] * PI / 180.0;
    sinSum += sin(rad);
    cosSum += cos(rad);
  }

  float avgRad = atan2(sinSum / count, cosSum / count);
  float avgHeading = avgRad * 180.0 / PI;

  return normalizeHeading(avgHeading);
}

void resetHeadingHistory() {
  for (int i = 0; i < HEADING_HISTORY_SIZE; i++) {
    headingHistory[i] = 0.0;
  }

  headingHistoryIndex = 0;
  headingHistoryFilled = false;
  latestHeadingValid = false;
}

float angleDifference(float a, float b) {
  float diff = fabs(a - b);

  if (diff > 180.0) {
    diff = 360.0 - diff;
  }

  return diff;
}


// =====================
// 状態リセット
// =====================

void stopVibration() {
  vibrationActive = false;
  pulseMotorOn = false;
  lastVibrationZone = ZONE_OFF;
  setMotorDuty(DUTY_STOP);
}

void resetPulseTiming() {
  pulseMotorOn = false;
  lastPulseSwitchMillis = millis();
  lastVibrationZone = ZONE_OFF;
}


// =====================
// キャリブレーション
// =====================

void calibrateMagnetometer() {
  stopVibration();
  northRegistered = false;
  resetHeadingHistory();
  resetPulseTiming();

  Serial.println();
  Serial.println("=== Calibration start ===");
  Serial.println("10秒間、センサーを水平に保ったまま360度以上ゆっくり回してください。");
  Serial.println("できればPC、金属、スマホ、モーターから少し離して行ってください。");
  Serial.println("キャリブレーション中、モーターは停止します。");

  float xMin = 1000000.0;
  float xMax = -1000000.0;
  float yMin = 1000000.0;
  float yMax = -1000000.0;

  unsigned long startMillis = millis();
  unsigned long lastStatusMillis = 0;

  while (millis() - startMillis < CALIBRATION_TIME_MS) {
    float x, y, z;

    if (readBM1422(x, y, z)) {
      if (x < xMin) xMin = x;
      if (x > xMax) xMax = x;
      if (y < yMin) yMin = y;
      if (y > yMax) yMax = y;
    }

    unsigned long now = millis();
    if (now - lastStatusMillis >= 1000) {
      lastStatusMillis = now;

      int remaining = (CALIBRATION_TIME_MS - (now - startMillis)) / 1000;
      Serial.print("calibrating... remaining ");
      Serial.print(remaining);
      Serial.println(" sec");
    }

    delay(30);
  }

  float xRange = xMax - xMin;
  float yRange = yMax - yMin;

  xOffset = (xMax + xMin) / 2.0;
  yOffset = (yMax + yMin) / 2.0;

  float xRadius = xRange / 2.0;
  float yRadius = yRange / 2.0;
  float averageRadius = (xRadius + yRadius) / 2.0;

  if (xRadius > 0.001 && yRadius > 0.001) {
    xScale = averageRadius / xRadius;
    yScale = averageRadius / yRadius;
  } else {
    xScale = 1.0;
    yScale = 1.0;
    Serial.println("WARNING: キャリブレーション範囲が小さすぎます。補正倍率は1.0にしました。");
  }

  Serial.println("=== Calibration complete ===");

  Serial.print("xMin=");
  Serial.print(xMin, 2);
  Serial.print(", xMax=");
  Serial.print(xMax, 2);
  Serial.print(", xOffset=");
  Serial.print(xOffset, 2);
  Serial.print(", xScale=");
  Serial.println(xScale, 3);

  Serial.print("yMin=");
  Serial.print(yMin, 2);
  Serial.print(", yMax=");
  Serial.print(yMax, 2);
  Serial.print(", yOffset=");
  Serial.print(yOffset, 2);
  Serial.print(", yScale=");
  Serial.println(yScale, 3);

  if (xRange < 5.0 || yRange < 5.0) {
    Serial.println("WARNING: 回転が足りない可能性があります。次回は360度以上ゆっくり回してください。");
  }

  Serial.println();
  Serial.println("次に、地磁気センサー基板の凸部分を実際の北へ向けてください。");
  Serial.println("向きを固定したまま Serial Monitor から n を入力して Enter を押してください。");
  Serial.println("nを押した瞬間の向きを、この試作品上の「北」として登録します。");
  Serial.println();
}


// =====================
// 振動パターン制御
// =====================

bool updateVibrationActive(float diffFromRegisteredNorth) {
  // 停止中なら30度以内で振動開始
  if (!vibrationActive && diffFromRegisteredNorth <= VIBRATION_ON_DEGREE) {
    vibrationActive = true;

    // 振動範囲に入ったら、まず一度すぐ振動を感じられるようにする
    pulseMotorOn = true;
    lastPulseSwitchMillis = millis();
  }

  // 振動中なら35度以上で停止
  if (vibrationActive && diffFromRegisteredNorth >= VIBRATION_OFF_DEGREE) {
    stopVibration();
  }

  return vibrationActive;
}

VibrationZone getVibrationZone(float diffFromRegisteredNorth) {
  if (!vibrationActive) {
    return ZONE_OFF;
  }

  if (diffFromRegisteredNorth <= CONTINUOUS_VIBRATION_DEGREE) {
    return ZONE_CONTINUOUS;
  }

  if (diffFromRegisteredNorth <= 10.0) {
    return ZONE_FAST;
  }

  if (diffFromRegisteredNorth <= 20.0) {
    return ZONE_MEDIUM;
  }

  return ZONE_SLOW;
}

unsigned long getPulseOffTime(float diffFromRegisteredNorth) {
  if (diffFromRegisteredNorth <= 10.0) {
    return PULSE_OFF_FAST_MS;
  }

  if (diffFromRegisteredNorth <= 20.0) {
    return PULSE_OFF_MEDIUM_MS;
  }

  return PULSE_OFF_SLOW_MS;
}

const char* getVibrationPatternName(float diffFromRegisteredNorth) {
  VibrationZone zone = getVibrationZone(diffFromRegisteredNorth);

  switch (zone) {
    case ZONE_CONTINUOUS:
      return "CONTINUOUS";
    case ZONE_FAST:
      return "PULSE_FAST";
    case ZONE_MEDIUM:
      return "PULSE_MEDIUM";
    case ZONE_SLOW:
      return "PULSE_SLOW";
    case ZONE_OFF:
    default:
      return "OFF";
  }
}

int updateMotorPattern(float diffFromRegisteredNorth) {
  bool active = updateVibrationActive(diffFromRegisteredNorth);
  unsigned long now = millis();

  if (!active) {
    stopVibration();
    return DUTY_STOP;
  }

  VibrationZone currentZone = getVibrationZone(diffFromRegisteredNorth);

  // ゾーンが変わった瞬間の処理。
  // 特に CONTINUOUS から PULSE に戻るとき、すぐ断続振動を再開させる。
  if (currentZone != lastVibrationZone) {
    lastVibrationZone = currentZone;

    if (currentZone == ZONE_OFF) {
      stopVibration();
      return DUTY_STOP;
    }

    if (currentZone == ZONE_CONTINUOUS) {
      pulseMotorOn = true;
      lastPulseSwitchMillis = now;
      setMotorDuty(DUTY_ON);
      return DUTY_ON;
    }

    // ここが今回の重要修正。
    // FAST / MEDIUM / SLOW に入った瞬間、待たずに一度ONにする。
    pulseMotorOn = true;
    lastPulseSwitchMillis = now;
    setMotorDuty(DUTY_ON);
    return DUTY_ON;
  }

  // 真北付近なら連続振動
  if (currentZone == ZONE_CONTINUOUS) {
    pulseMotorOn = true;
    lastPulseSwitchMillis = now;
    setMotorDuty(DUTY_ON);
    return DUTY_ON;
  }

  // パルス振動
  unsigned long offTime = getPulseOffTime(diffFromRegisteredNorth);

  if (pulseMotorOn) {
    // ON状態が一定時間続いたらOFFにする
    if (now - lastPulseSwitchMillis >= PULSE_ON_MS) {
      pulseMotorOn = false;
      lastPulseSwitchMillis = now;
      setMotorDuty(DUTY_STOP);
      return DUTY_STOP;
    }

    setMotorDuty(DUTY_ON);
    return DUTY_ON;
  }

  // OFF状態が距離に応じた時間続いたらONにする
  if (now - lastPulseSwitchMillis >= offTime) {
    pulseMotorOn = true;
    lastPulseSwitchMillis = now;
    setMotorDuty(DUTY_ON);
    return DUTY_ON;
  }

  setMotorDuty(DUTY_STOP);
  return DUTY_STOP;
}


// =====================
// シリアルコマンド
// =====================

void registerNorth() {
  if (!latestHeadingValid) {
    Serial.println("ERROR: まだ方位が取得できていません。少し待ってから n を送ってください。");
    return;
  }

  registeredNorthHeading = latestSmoothedHeading;
  northRegistered = true;
  stopVibration();
  resetPulseTiming();

  Serial.println();
  Serial.println("=== North registered ===");
  Serial.print("登録した北のheading: ");
  Serial.print(registeredNorthHeading, 1);
  Serial.println(" deg");
  Serial.println("以後、この向きに近づくほど振動間隔が短くなり、真北付近では連続振動します。");
  Serial.println();
}

void resetNorthRegistration() {
  northRegistered = false;
  stopVibration();
  resetPulseTiming();

  Serial.println();
  Serial.println("=== North registration reset ===");
  Serial.println("凸部分を北へ向けて、n + Enter で再登録してください。");
  Serial.println();
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == 'n' || c == 'N') {
      registerNorth();
    } else if (c == 'r' || c == 'R') {
      resetNorthRegistration();
    } else if (c == 'c' || c == 'C') {
      calibrateMagnetometer();
    } else if (c == '\n' || c == '\r' || c == ' ') {
      // Enterや空白は無視
    } else {
      Serial.print("Unknown command: ");
      Serial.println(c);
      Serial.println("使用可能: n=北登録, r=北登録リセット, c=再キャリブレーション");
    }
  }
}


// =====================
// Arduino標準関数
// =====================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("vibe-compass v0.3.2 BM1422AGMV");
  Serial.println("nキー北登録 + パルス振動 + 連続振動復帰改善版");
  Serial.println();

  Wire.begin(SDA_PIN, SCL_PIN);

  bool sensorOk = initBM1422();
  if (!sensorOk) {
    Serial.println("配線、電源、SDA/SCL、I2Cアドレスを確認してください。");
    Serial.println("この状態では処理を続けられません。");

    setupMotorPwm();
    stopVibration();

    while (true) {
      stopVibration();
      delay(1000);
    }
  }

  setupMotorPwm();
  stopVibration();

  calibrateMagnetometer();

  Serial.println("setup complete");
}

void loop() {
  handleSerialCommands();

  float x, y, z;

  bool readOk = readBM1422(x, y, z);

  if (!readOk) {
    stopVibration();
    Serial.println("ERROR: BM1422AGMVのセンサー値を読めませんでした。");
    delay(500);
    return;
  }

  float heading = calculateHeadingDegree(x, y);
  float smoothedHeading = smoothHeading(heading);

  latestHeading = heading;
  latestSmoothedHeading = smoothedHeading;
  latestHeadingValid = true;

  unsigned long now = millis();

  // 北登録前は、必ずモーター停止
  if (!northRegistered) {
    stopVibration();

    if (now - lastGuideMillis >= GUIDE_INTERVAL_MS) {
      lastGuideMillis = now;

      Serial.print("WAITING NORTH REGISTER: heading=");
      Serial.print(smoothedHeading, 1);
      Serial.println(" deg");
      Serial.println("凸部分を北へ向けて n + Enter を送ってください。");
    }

    delay(20);
    return;
  }

  float diffFromRegisteredNorth = angleDifference(smoothedHeading, registeredNorthHeading);

  int duty = updateMotorPattern(diffFromRegisteredNorth);

  if (now - lastPrintMillis >= PRINT_INTERVAL_MS) {
    lastPrintMillis = now;

    Serial.print("X=");
    Serial.print(x, 2);
    Serial.print(" uT, Y=");
    Serial.print(y, 2);
    Serial.print(" uT, Z=");
    Serial.print(z, 2);

    Serial.print(", heading=");
    Serial.print(smoothedHeading, 1);

    Serial.print(" deg, registeredNorth=");
    Serial.print(registeredNorthHeading, 1);

    Serial.print(" deg, diff=");
    Serial.print(diffFromRegisteredNorth, 1);

    Serial.print(" deg, duty=");
    Serial.print(duty);

    Serial.print(", pattern=");
    Serial.print(getVibrationPatternName(diffFromRegisteredNorth));

    Serial.print(", vibration=");
    Serial.println(vibrationActive ? "ON" : "OFF");
  }

  delay(20);
}