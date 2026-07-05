#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ============================================================
// v0.3 自律北検知版 改良版
// ------------------------------------------------------------
// 目的：
// 外部コンパスやSerial Monitorで北を登録せず、
// 地磁気センサー自身の方位計算を使って北を検知する。
//
// 操作：
// 1. 電源を入れる
// 2. 起動直後に10秒間、センサーを水平にしてゆっくり一周以上回す
// 3. キャリブレーション完了後、基板の「前」印が北を向くと振動する
//
// 今回の改良：
// ・millis() % cycleMs による断続振動を廃止
// ・モーターのON/OFF状態と次回切り替え時刻を管理
// ・北からのズレ量を平滑化
// ・新しいセンサー値が来たときだけ平滑化する
// ・中心付近の連続振動にヒステリシスを追加
//
// 重要：
// SENSOR_FORWARD_OFFSET は、
// 「基板の前方向を北に向けたときの Heading 値」を入れる。
// 今回の測定では、凸部分を北に向けたとき 121〜124度だったため、
// まずは 123.0 を使用する。
// ============================================================


// ============================================================
// ピン設定
// ============================================================

// 振動モーター制御ピン
const int MOTOR_PIN = 25;

// ESP32 の I2C ピン
const int SDA_PIN = 21;
const int SCL_PIN = 22;


// ============================================================
// 北判定・方位補正設定
// ============================================================

// センサー基板の「前」方向を北に向けたときの Heading 値。
// 今回の実測値：北 121〜124度 → 中央値として 99.3度を採用。
const float SENSOR_FORWARD_OFFSET = 99.3;

// 北とみなす範囲。
// 最初は誤反応を避けるため、やや狭めの ±25度。
// 反応が厳しすぎる場合は 30.0 に広げてもよい。
const float NORTH_RANGE = 25.0;

// 中心付近とみなす範囲。
// この範囲内に入ると連続振動にする。
const float CENTER_ENTER_RANGE = 5.0;

// 連続振動から断続振動へ戻る範囲。
// ENTERより少し広くすることで、5度付近でON/OFFが揺れるのを防ぐ。
const float CENTER_EXIT_RANGE = 7.0;


// ============================================================
// 読み取り・表示間隔
// ============================================================

const unsigned long SENSOR_READ_INTERVAL_MS = 200;
const unsigned long DEBUG_PRINT_INTERVAL_MS = 500;


// ============================================================
// モーター断続振動設定
// ============================================================

// 北からのズレを平滑化する強さ。
// 大きいほど反応は速いが揺れやすい。
// 小さいほど安定するが反応は少し遅くなる。
const float DIFF_SMOOTHING_ALPHA = 0.25;

// 北に近いときの断続振動周期
const unsigned long PULSE_CYCLE_MIN_MS = 220;

// 北判定範囲の端に近いときの断続振動周期
const unsigned long PULSE_CYCLE_MAX_MS = 900;

// 北に近いときのON時間
const unsigned long PULSE_ON_MAX_MS = 150;

// 北判定範囲の端に近いときのON時間
const unsigned long PULSE_ON_MIN_MS = 60;

// OFF時間が短すぎると断続感が分かりづらいため、最低値を持たせる
const unsigned long PULSE_OFF_MIN_MS = 40;


// ============================================================
// AE-BM1422AGMV の I2C設定
// ============================================================

const uint8_t BM1422_ADDR = 0x0E;

// BM1422AGMV の主なレジスタ
const uint8_t REG_WIA = 0x0F;     // Who am I
const uint8_t REG_DATAX = 0x10;   // X/Y/Zデータ開始位置
const uint8_t REG_STA1 = 0x18;    // 測定完了ステータス
const uint8_t REG_CNTL1 = 0x1B;
const uint8_t REG_CNTL2 = 0x1C;
const uint8_t REG_CNTL3 = 0x1D;
const uint8_t REG_AVE_A = 0x40;
const uint8_t REG_CNTL4 = 0x5C;

// 設定値
const uint8_t WIA_EXPECTED = 0x41;
const uint8_t CNTL1_VALUE = 0xC2;     // 14bit出力、電源ONなど
const uint8_t CNTL2_VALUE = 0x08;     // DRDY有効
const uint8_t CNTL3_FORCE = 0x40;     // 1回測定開始
const uint8_t AVE_A_VALUE = 0x00;

// ROHMのArduinoライブラリでは14bit時に raw / 24 で uT換算している
const float SENSITIVITY_14BIT = 24.0;


// ============================================================
// グローバル変数
// ============================================================

// X/Yの中心ズレ補正値
float offsetX = 0.0;
float offsetY = 0.0;

// 最新の測定値・計算値
float latestX = 0.0;
float latestY = 0.0;
float latestZ = 0.0;

float latestHeading = 0.0;          // センサー計算上の方位
float latestForwardHeading = 0.0;   // 前方向オフセット補正後の方位
float latestDiffFromNorth = 0.0;    // 北からのズレ（-180〜180度）

bool bm1422Ready = false;
bool calibrationReady = false;
bool latestIsNorth = false;

unsigned long lastSensorReadTime = 0;
unsigned long lastDebugPrintTime = 0;

// センサー値が更新されるたびに増やす番号。
// モーター制御側で「新しいセンサー値が来たか」を判断するために使う。
unsigned long latestSensorUpdateId = 0;


// ============================================================
// モーター断続振動の状態管理
// ============================================================

// 北からのズレを少しなめらかにした値
float smoothedAbsDiff = 0.0;
bool smoothedAbsDiffReady = false;

// 最後にモーター制御へ反映したセンサー更新番号
unsigned long lastMotorFeedbackSensorUpdateId = 0;

// 断続振動のON/OFF状態
bool motorPulseOn = false;

// 次にON/OFFを切り替える時刻
unsigned long nextMotorToggleTime = 0;

// 中心付近の連続振動モード
bool centerVibrationMode = false;


// ============================================================
// 基本ユーティリティ
// ============================================================

float normalizeHeading(float angle) {
  while (angle < 0.0) {
    angle += 360.0;
  }

  while (angle >= 360.0) {
    angle -= 360.0;
  }

  return angle;
}


// 2つの角度の差を -180〜180度で返す。
// 例：350度と10度の差を340度ではなく -20度として扱う。
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


float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }

  if (value > maxValue) {
    return maxValue;
  }

  return value;
}


float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
  if (inMax == inMin) {
    return outMin;
  }

  float ratio = (value - inMin) / (inMax - inMin);
  ratio = clampFloat(ratio, 0.0, 1.0);

  return outMin + (outMax - outMin) * ratio;
}


// millis() のオーバーフローにも比較的強い時刻判定
bool timeReached(unsigned long now, unsigned long targetTime) {
  return (int32_t)(now - targetTime) >= 0;
}


// ============================================================
// モーター制御
// ============================================================

void motorOn() {
  digitalWrite(MOTOR_PIN, HIGH);
}


void motorOff() {
  digitalWrite(MOTOR_PIN, LOW);
}


// モーター断続振動の状態をリセットする。
// 北範囲外に出たとき、キャリブレーション開始時、エラー時に使う。
void resetMotorFeedbackState() {
  motorPulseOn = false;
  nextMotorToggleTime = 0;
  centerVibrationMode = false;

  smoothedAbsDiff = 0.0;
  smoothedAbsDiffReady = false;
  lastMotorFeedbackSensorUpdateId = 0;

  motorOff();
}


// 状態通知用の短い振動。
// setupやキャリブレーション時だけで使う。
void pulseMotor(unsigned long durationMs) {
  motorOn();
  delay(durationMs);
  motorOff();
}


void vibrationStartupSignal() {
  // 起動したことを知らせる短い2回振動
  pulseMotor(250);
  delay(250);
  pulseMotor(250);
  delay(500);
}


void vibrationCalibrationDoneSignal() {
  // キャリブレーション完了を知らせる、長めの2回振動
  pulseMotor(700);
  delay(300);
  pulseMotor(700);
  delay(500);
}


void vibrationErrorSignal() {
  // エラーを知らせる短めの4回振動
  for (int i = 0; i < 4; i++) {
    pulseMotor(180);
    delay(180);
  }
}


// 新しいセンサー値が来たときだけ、北からのズレ量を平滑化する。
void updateSmoothedDiffIfNeeded() {
  if (lastMotorFeedbackSensorUpdateId == latestSensorUpdateId) {
    return;
  }

  lastMotorFeedbackSensorUpdateId = latestSensorUpdateId;

  float rawAbsDiff = fabs(latestDiffFromNorth);

  if (!smoothedAbsDiffReady) {
    smoothedAbsDiff = rawAbsDiff;
    smoothedAbsDiffReady = true;
    return;
  }

  smoothedAbsDiff =
    DIFF_SMOOTHING_ALPHA * rawAbsDiff +
    (1.0 - DIFF_SMOOTHING_ALPHA) * smoothedAbsDiff;
}


// 北に近いほど振動が密になる。
// 中心付近では連続振動。
// 範囲外では停止。
void updateMotorFeedback() {
  if (!bm1422Ready || !calibrationReady || !latestIsNorth) {
    resetMotorFeedbackState();
    return;
  }

  updateSmoothedDiffIfNeeded();

  if (!smoothedAbsDiffReady) {
    resetMotorFeedbackState();
    return;
  }

  float absDiff = smoothedAbsDiff;

  // 中心付近では連続振動。
  // ただし、境界付近で連続/断続が細かく切り替わらないようにヒステリシスを入れる。
  if (centerVibrationMode) {
    if (absDiff <= CENTER_EXIT_RANGE) {
      motorPulseOn = false;
      nextMotorToggleTime = 0;
      motorOn();
      return;
    }

    centerVibrationMode = false;
  }

  if (absDiff <= CENTER_ENTER_RANGE) {
    centerVibrationMode = true;
    motorPulseOn = false;
    nextMotorToggleTime = 0;
    motorOn();
    return;
  }

  // 念のため、北判定範囲外なら停止
  if (absDiff > NORTH_RANGE) {
    resetMotorFeedbackState();
    return;
  }

  // 連続振動範囲のすぐ外から、北判定範囲の端までを断続振動にする。
  // 北に近いほど周期を短くする。
  unsigned long cycleMs = (unsigned long)mapFloat(
    absDiff,
    CENTER_EXIT_RANGE,
    NORTH_RANGE,
    (float)PULSE_CYCLE_MIN_MS,
    (float)PULSE_CYCLE_MAX_MS
  );

  // 北に近いほどON時間を少し長くする。
  unsigned long onMs = (unsigned long)mapFloat(
    absDiff,
    CENTER_EXIT_RANGE,
    NORTH_RANGE,
    (float)PULSE_ON_MAX_MS,
    (float)PULSE_ON_MIN_MS
  );

  if (onMs >= cycleMs) {
    onMs = cycleMs - PULSE_OFF_MIN_MS;
  }

  unsigned long offMs = cycleMs - onMs;

  if (offMs < PULSE_OFF_MIN_MS) {
    offMs = PULSE_OFF_MIN_MS;
  }

  unsigned long now = millis();

  // 北範囲に入った直後は、まずONから始める
  if (nextMotorToggleTime == 0) {
    motorPulseOn = true;
    motorOn();
    nextMotorToggleTime = now + onMs;
    return;
  }

  // まだ切り替え時刻でなければ何もしない
  if (!timeReached(now, nextMotorToggleTime)) {
    return;
  }

  // 切り替え時刻になったら、ON/OFFを反転する
  if (motorPulseOn) {
    motorPulseOn = false;
    motorOff();
    nextMotorToggleTime = now + offMs;
  } else {
    motorPulseOn = true;
    motorOn();
    nextMotorToggleTime = now + onMs;
  }
}


// ============================================================
// I2C書き込み・読み取り
// ============================================================

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

  int received = Wire.requestFrom((int)BM1422_ADDR, (int)length);

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


// ============================================================
// BM1422AGMV 初期化
// ============================================================

bool initBM1422() {
  uint8_t whoAmI = 0;

  if (!readRegister(REG_WIA, whoAmI)) {
    Serial.println("BM1422AGMVにアクセスできません。配線を確認してください。");
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


// ============================================================
// 地磁気データ読み取り
// ============================================================

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


// ============================================================
// 方位計算
// ============================================================

void calculateHeading(float x, float y, float z) {
  latestX = x;
  latestY = y;
  latestZ = z;

  float correctedX = x - offsetX;
  float correctedY = y - offsetY;

  // X/Yから方位を計算
  float heading = atan2(correctedY, correctedX) * 180.0 / PI;
  heading = normalizeHeading(heading);

  // 基板の「前」印が0度になるよう補正
  float forwardHeading = normalizeHeading(heading - SENSOR_FORWARD_OFFSET);

  // 0度、つまり北からのズレを求める
  float diffFromNorth = angleDifference(forwardHeading, 0.0);

  latestHeading = heading;
  latestForwardHeading = forwardHeading;
  latestDiffFromNorth = diffFromNorth;
  latestIsNorth = fabs(diffFromNorth) <= NORTH_RANGE;

  // 新しいセンサー値が反映されたことを記録
  latestSensorUpdateId++;
}


// ============================================================
// X/Yの中心ズレを補正するキャリブレーション
// ============================================================

bool calibrateMagnetometer() {
  calibrationReady = false;
  latestIsNorth = false;
  resetMotorFeedbackState();

  Serial.println();
  Serial.println("Calibration start");
  Serial.println("10秒間、センサーを水平にしてゆっくり一周以上回してください。");
  Serial.println("Serial Monitorなしで使う場合も、この10秒間の回転操作は必要です。");

  float minX = 99999.0;
  float maxX = -99999.0;
  float minY = 99999.0;
  float maxY = -99999.0;

  int sampleCount = 0;

  unsigned long startTime = millis();
  unsigned long lastCalibrationPrintTime = 0;

  while (millis() - startTime < 10000) {
    float x, y, z;

    if (readMagneticData(x, y, z)) {
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;

      sampleCount++;

      if (millis() - lastCalibrationPrintTime >= 500) {
        lastCalibrationPrintTime = millis();

        Serial.print("calibrating... ");
        Serial.print("X: ");
        Serial.print(x, 2);
        Serial.print(" uT, Y: ");
        Serial.print(y, 2);
        Serial.print(" uT, Z: ");
        Serial.print(z, 2);
        Serial.print(" uT, samples: ");
        Serial.println(sampleCount);
      }
    }

    delay(100);
  }

  if (sampleCount < 10) {
    Serial.println("Calibration failed: サンプル数が不足しています。");
    Serial.println("配線、センサー、I2C接続を確認してください。");
    calibrationReady = false;
    vibrationErrorSignal();
    return false;
  }

  offsetX = (maxX + minX) / 2.0;
  offsetY = (maxY + minY) / 2.0;

  calibrationReady = true;

  // キャリブレーション直後の古いモーター状態を使わないようにする
  resetMotorFeedbackState();

  Serial.println("Calibration done");
  Serial.print("sampleCount = ");
  Serial.println(sampleCount);
  Serial.print("offsetX = ");
  Serial.println(offsetX, 2);
  Serial.print("offsetY = ");
  Serial.println(offsetY, 2);

  Serial.println();
  Serial.println("v0.3 autonomous north detection mode - improved motor feedback");
  Serial.print("SENSOR_FORWARD_OFFSET = ");
  Serial.print(SENSOR_FORWARD_OFFSET, 1);
  Serial.println(" deg");
  Serial.print("NORTH_RANGE = ±");
  Serial.print(NORTH_RANGE, 1);
  Serial.println(" deg");
  Serial.print("CENTER_ENTER_RANGE = ±");
  Serial.print(CENTER_ENTER_RANGE, 1);
  Serial.println(" deg");
  Serial.print("CENTER_EXIT_RANGE = ±");
  Serial.print(CENTER_EXIT_RANGE, 1);
  Serial.println(" deg");

  Serial.println();
  Serial.println("以後、基板の「前」印が北を向くと振動します。");
  Serial.println("c を送ると、キャリブレーションをやり直します。");
  Serial.println("n による北登録は v0.3 では使用しません。");

  vibrationCalibrationDoneSignal();

  return true;
}


// ============================================================
// デバッグ表示
// ============================================================

void printDebugInfo() {
  Serial.print("X: ");
  Serial.print(latestX, 2);
  Serial.print(" uT, Y: ");
  Serial.print(latestY, 2);
  Serial.print(" uT, Z: ");
  Serial.print(latestZ, 2);

  Serial.print(" uT, Heading: ");
  Serial.print(latestHeading, 1);
  Serial.print(" deg");

  Serial.print(", Forward: ");
  Serial.print(latestForwardHeading, 1);
  Serial.print(" deg");

  Serial.print(", DiffFromNorth: ");
  Serial.print(latestDiffFromNorth, 1);
  Serial.print(" deg");

  Serial.print(", SmoothedAbsDiff: ");

  if (smoothedAbsDiffReady) {
    Serial.print(smoothedAbsDiff, 1);
    Serial.print(" deg");
  } else {
    Serial.print("-");
  }

  Serial.print(", State: ");

  if (!calibrationReady) {
    Serial.println("not calibrated");
  } else if (latestIsNorth) {
    if (centerVibrationMode) {
      Serial.println("NORTH! center");
    } else {
      Serial.println("NORTH! pulse");
    }
  } else {
    Serial.println("not north");
  }
}


void printHelp() {
  Serial.println();
  Serial.println("===== v0.3 help =====");
  Serial.println("c : キャリブレーションをやり直す");
  Serial.println("i : この説明を表示する");
  Serial.println("n : v0.3では使用しない。北登録は不要");
  Serial.println();
  Serial.print("SENSOR_FORWARD_OFFSET = ");
  Serial.print(SENSOR_FORWARD_OFFSET, 1);
  Serial.println(" deg");
  Serial.print("NORTH_RANGE = ±");
  Serial.print(NORTH_RANGE, 1);
  Serial.println(" deg");
  Serial.print("CENTER_ENTER_RANGE = ±");
  Serial.print(CENTER_ENTER_RANGE, 1);
  Serial.println(" deg");
  Serial.print("CENTER_EXIT_RANGE = ±");
  Serial.print(CENTER_EXIT_RANGE, 1);
  Serial.println(" deg");
  Serial.println("=====================");
  Serial.println();
}


// ============================================================
// Serialコマンド処理
// ============================================================

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char command = Serial.read();

    if (command == '\n' || command == '\r' || command == ' ') {
      continue;
    }

    if (command == 'c' || command == 'C') {
      Serial.println();
      Serial.println("Command: c");
      Serial.println("キャリブレーションをやり直します。");
      calibrateMagnetometer();

      // キャリブレーション直後の古い値で判定しないようにする
      lastSensorReadTime = 0;
      lastDebugPrintTime = 0;
      return;
    }

    if (command == 'i' || command == 'I') {
      printHelp();
      continue;
    }

    if (command == 'n' || command == 'N') {
      Serial.println();
      Serial.println("Command: n");
      Serial.println("v0.3では n による北登録は使用しません。");
      Serial.println("現在は SENSOR_FORWARD_OFFSET により、センサーの方位を直接補正しています。");
      Serial.print("SENSOR_FORWARD_OFFSET = ");
      Serial.print(SENSOR_FORWARD_OFFSET, 1);
      Serial.println(" deg");
      continue;
    }

    Serial.println();
    Serial.print("Unknown command: ");
    Serial.println(command);
    Serial.println("i を送るとヘルプを表示します。");
  }
}


// ============================================================
// センサー読み取り更新
// ============================================================

void updateSensorIfNeeded() {
  unsigned long now = millis();

  if (now - lastSensorReadTime < SENSOR_READ_INTERVAL_MS) {
    return;
  }

  lastSensorReadTime = now;

  float x, y, z;

  if (!readMagneticData(x, y, z)) {
    latestIsNorth = false;
    resetMotorFeedbackState();
    return;
  }

  calculateHeading(x, y, z);

  if (now - lastDebugPrintTime >= DEBUG_PRINT_INTERVAL_MS) {
    lastDebugPrintTime = now;
    printDebugInfo();
  }
}


// ============================================================
// setup
// ============================================================

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_PIN, OUTPUT);
  resetMotorFeedbackState();

  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println("Vibe Compass v0.3");
  Serial.println("Autonomous north detection mode");
  Serial.println("Improved motor feedback");
  Serial.println("======================================");
  Serial.println();

  vibrationStartupSignal();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  bm1422Ready = initBM1422();

  if (!bm1422Ready) {
    Serial.println("初期化に失敗しました。配線を確認してください。");
    vibrationErrorSignal();
    return;
  }

  calibrateMagnetometer();
}


// ============================================================
// loop
// ============================================================

void loop() {
  handleSerialCommands();

  if (!bm1422Ready) {
    resetMotorFeedbackState();
    delay(1000);
    return;
  }

  if (!calibrationReady) {
    resetMotorFeedbackState();
    delay(100);
    return;
  }

  updateSensorIfNeeded();
  updateMotorFeedback();

  delay(10);
}