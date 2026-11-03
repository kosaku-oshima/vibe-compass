#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ============================================================
// v0.4 補正値変更版（v0.3 自律北検知版 - 屋外試験 診断版ベース）
// ------------------------------------------------------------
// ベース:
// GitHub
// v0.3_autonomous_north_detection.ino/
// v0.3_autonomous_north_detection.ino.ino
//
// v0.4での変更（2026-11-03）:
// ・SENSOR_FORWARD_OFFSET を 99.3 → 89.3 に変更
//   屋外・電池駆動の実測で、連続振動が毎回コンパスの北より
//   東に約5〜20°（平均約11°）ずれていたため、西へ10°ずらした
// ・起動時・ヘルプの表示をv0.4に変更
//
// 診断版の目的（v0.3から引き続き）:
// ・方位ズレの原因調査
// ・南東など誤った方向で振動する現象の原因調査
// ・北を向いたまま振動が弱まる / 止まる現象の原因調査
// ・キャリブレーション状態の確認
// ・センサー読み取り失敗の確認
//
// ★重要
// 北判定・振動パターン・平滑化・ヒステリシスなど、
// v0.3本来の動作ロジックは変更しない。
// 変更は SENSOR_FORWARD_OFFSET と表示のみ。
// ============================================================


// ============================================================
// ピン設定
// ============================================================

const int MOTOR_PIN = 25;

const int SDA_PIN = 21;
const int SCL_PIN = 22;


// ============================================================
// 北判定・方位補正設定
// ============================================================

// v0.4で変更（2026-11-03）
// 99.3 → 89.3
// 屋外・電池駆動の実測で、連続振動が毎回コンパスの北より
// 東に約5〜20°（平均約11°）ずれていたため、西へ10°ずらした
const float SENSOR_FORWARD_OFFSET = 89.3;

// 北とみなす範囲
const float NORTH_RANGE = 25.0;

// 中心付近：連続振動へ入る範囲
const float CENTER_ENTER_RANGE = 5.0;

// 連続振動から抜ける範囲
const float CENTER_EXIT_RANGE = 7.0;


// ============================================================
// 読み取り・表示間隔
// ============================================================

const unsigned long SENSOR_READ_INTERVAL_MS = 200;

// 診断版では少し細かく観察するため500msのまま
const unsigned long DEBUG_PRINT_INTERVAL_MS = 500;


// ============================================================
// モーター断続振動設定
// ============================================================

const float DIFF_SMOOTHING_ALPHA = 0.25;

const unsigned long PULSE_CYCLE_MIN_MS = 220;
const unsigned long PULSE_CYCLE_MAX_MS = 900;

const unsigned long PULSE_ON_MAX_MS = 150;
const unsigned long PULSE_ON_MIN_MS = 60;

const unsigned long PULSE_OFF_MIN_MS = 40;


// ============================================================
// AE-BM1422AGMV I2C設定
// ============================================================

const uint8_t BM1422_ADDR = 0x0E;

const uint8_t REG_WIA   = 0x0F;
const uint8_t REG_DATAX = 0x10;
const uint8_t REG_STA1  = 0x18;
const uint8_t REG_CNTL1 = 0x1B;
const uint8_t REG_CNTL2 = 0x1C;
const uint8_t REG_CNTL3 = 0x1D;
const uint8_t REG_AVE_A = 0x40;
const uint8_t REG_CNTL4 = 0x5C;

const uint8_t WIA_EXPECTED = 0x41;

const uint8_t CNTL1_VALUE = 0xC2;
const uint8_t CNTL2_VALUE = 0x08;
const uint8_t CNTL3_FORCE = 0x40;
const uint8_t AVE_A_VALUE = 0x00;

const float SENSITIVITY_14BIT = 24.0;


// ============================================================
// グローバル変数
// ============================================================

// キャリブレーション補正値
float offsetX = 0.0;
float offsetY = 0.0;


// 最新のμT値
float latestX = 0.0;
float latestY = 0.0;
float latestZ = 0.0;


// ★診断追加：生のセンサー値
int16_t latestRawX = 0;
int16_t latestRawY = 0;
int16_t latestRawZ = 0;


// 方位
float latestHeading = 0.0;
float latestForwardHeading = 0.0;
float latestDiffFromNorth = 0.0;


// ★診断追加：補正後X/Y
float latestCorrectedX = 0.0;
float latestCorrectedY = 0.0;


// ★診断追加：磁場の大きさ
float latestFieldStrength = 0.0;


// 状態
bool bm1422Ready = false;
bool calibrationReady = false;
bool latestIsNorth = false;


// 時刻
unsigned long lastSensorReadTime = 0;
unsigned long lastDebugPrintTime = 0;


// センサー更新番号
unsigned long latestSensorUpdateId = 0;


// ============================================================
// ★診断用状態
// ============================================================

// ESP32がモーターへ出している命令
// ※実際に物理モーターが回っていることを意味するわけではない
bool motorCommandOn = false;


// センサー読み取り成功・失敗回数
unsigned long sensorReadSuccessCount = 0;
unsigned long sensorReadFailCount = 0;


// 最後にセンサー読み取り成功した時刻
unsigned long lastSensorSuccessTime = 0;


// キャリブレーション時のmin/max
float calibrationMinX = 0.0;
float calibrationMaxX = 0.0;
float calibrationMinY = 0.0;
float calibrationMaxY = 0.0;

float calibrationRangeX = 0.0;
float calibrationRangeY = 0.0;

int calibrationSampleCount = 0;


// ============================================================
// モーター断続振動 状態管理
// ============================================================

float smoothedAbsDiff = 0.0;
bool smoothedAbsDiffReady = false;

unsigned long lastMotorFeedbackSensorUpdateId = 0;

bool motorPulseOn = false;
unsigned long nextMotorToggleTime = 0;

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


float mapFloat(
  float value,
  float inMin,
  float inMax,
  float outMin,
  float outMax
) {
  if (inMax == inMin) {
    return outMin;
  }

  float ratio =
    (value - inMin) /
    (inMax - inMin);

  ratio =
    clampFloat(
      ratio,
      0.0,
      1.0
    );

  return
    outMin +
    (outMax - outMin) * ratio;
}


bool timeReached(
  unsigned long now,
  unsigned long targetTime
) {
  return
    (int32_t)(now - targetTime) >= 0;
}


// ============================================================
// モーター制御
// ============================================================

void motorOn() {
  digitalWrite(MOTOR_PIN, HIGH);

  // ★診断
  motorCommandOn = true;
}


void motorOff() {
  digitalWrite(MOTOR_PIN, LOW);

  // ★診断
  motorCommandOn = false;
}


void resetMotorFeedbackState() {
  motorPulseOn = false;
  nextMotorToggleTime = 0;

  centerVibrationMode = false;

  smoothedAbsDiff = 0.0;
  smoothedAbsDiffReady = false;

  lastMotorFeedbackSensorUpdateId = 0;

  motorOff();
}


void pulseMotor(unsigned long durationMs) {
  motorOn();

  delay(durationMs);

  motorOff();
}


// ============================================================
// 起動・状態通知振動
// ============================================================

void vibrationStartupSignal() {
  Serial.println("[BOOT] Startup vibration 1/2");

  pulseMotor(250);

  delay(250);

  Serial.println("[BOOT] Startup vibration 2/2");

  pulseMotor(250);

  delay(500);

  Serial.println("[BOOT] Startup vibration finished");
}


void vibrationCalibrationDoneSignal() {
  Serial.println("[CAL] Completion vibration 1/2");

  pulseMotor(700);

  delay(300);

  Serial.println("[CAL] Completion vibration 2/2");

  pulseMotor(700);

  delay(500);
}


void vibrationErrorSignal() {
  Serial.println("[ERROR] Error vibration");

  for (int i = 0; i < 4; i++) {
    pulseMotor(180);
    delay(180);
  }
}


// ============================================================
// 平滑化
// ============================================================

void updateSmoothedDiffIfNeeded() {
  if (
    lastMotorFeedbackSensorUpdateId
    ==
    latestSensorUpdateId
  ) {
    return;
  }


  lastMotorFeedbackSensorUpdateId =
    latestSensorUpdateId;


  float rawAbsDiff =
    fabs(latestDiffFromNorth);


  if (!smoothedAbsDiffReady) {
    smoothedAbsDiff = rawAbsDiff;
    smoothedAbsDiffReady = true;

    return;
  }


  smoothedAbsDiff =
    DIFF_SMOOTHING_ALPHA * rawAbsDiff
    +
    (1.0 - DIFF_SMOOTHING_ALPHA)
    * smoothedAbsDiff;
}


// ============================================================
// モーターフィードバック
// ============================================================

void updateMotorFeedback() {

  if (
    !bm1422Ready
    ||
    !calibrationReady
    ||
    !latestIsNorth
  ) {
    resetMotorFeedbackState();

    return;
  }


  updateSmoothedDiffIfNeeded();


  if (!smoothedAbsDiffReady) {
    resetMotorFeedbackState();

    return;
  }


  float absDiff = smoothedAbsDiff;


  // ----------------------------------------------------------
  // 中心連続振動
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // 北範囲外
  // ----------------------------------------------------------

  if (absDiff > NORTH_RANGE) {
    resetMotorFeedbackState();

    return;
  }


  // ----------------------------------------------------------
  // 断続振動
  // ----------------------------------------------------------

  unsigned long cycleMs =
    (unsigned long)mapFloat(
      absDiff,
      CENTER_EXIT_RANGE,
      NORTH_RANGE,
      (float)PULSE_CYCLE_MIN_MS,
      (float)PULSE_CYCLE_MAX_MS
    );


  unsigned long onMs =
    (unsigned long)mapFloat(
      absDiff,
      CENTER_EXIT_RANGE,
      NORTH_RANGE,
      (float)PULSE_ON_MAX_MS,
      (float)PULSE_ON_MIN_MS
    );


  if (onMs >= cycleMs) {
    onMs =
      cycleMs -
      PULSE_OFF_MIN_MS;
  }


  unsigned long offMs =
    cycleMs -
    onMs;


  if (offMs < PULSE_OFF_MIN_MS) {
    offMs =
      PULSE_OFF_MIN_MS;
  }


  unsigned long now =
    millis();


  // 北範囲に入った直後
  if (nextMotorToggleTime == 0) {

    motorPulseOn = true;

    motorOn();

    nextMotorToggleTime =
      now +
      onMs;

    return;
  }


  if (
    !timeReached(
      now,
      nextMotorToggleTime
    )
  ) {
    return;
  }


  if (motorPulseOn) {

    motorPulseOn = false;

    motorOff();

    nextMotorToggleTime =
      now +
      offMs;

  } else {

    motorPulseOn = true;

    motorOn();

    nextMotorToggleTime =
      now +
      onMs;
  }
}


// ============================================================
// I2C
// ============================================================

bool writeRegister(
  uint8_t reg,
  uint8_t value
) {
  Wire.beginTransmission(BM1422_ADDR);

  Wire.write(reg);
  Wire.write(value);

  return
    Wire.endTransmission() == 0;
}


bool writeRegister16(
  uint8_t reg,
  uint16_t value
) {
  Wire.beginTransmission(BM1422_ADDR);

  Wire.write(reg);

  Wire.write(
    (value >> 8) & 0xFF
  );

  Wire.write(
    value & 0xFF
  );

  return
    Wire.endTransmission() == 0;
}


bool readRegisters(
  uint8_t reg,
  uint8_t *buffer,
  uint8_t length
) {
  Wire.beginTransmission(BM1422_ADDR);

  Wire.write(reg);


  if (
    Wire.endTransmission(false)
    !=
    0
  ) {
    return false;
  }


  int received =
    Wire.requestFrom(
      (int)BM1422_ADDR,
      (int)length
    );


  if (received != length) {
    return false;
  }


  for (
    uint8_t i = 0;
    i < length;
    i++
  ) {
    buffer[i] = Wire.read();
  }


  return true;
}


bool readRegister(
  uint8_t reg,
  uint8_t &value
) {
  return
    readRegisters(
      reg,
      &value,
      1
    );
}


// ============================================================
// BM1422初期化
// ============================================================

bool initBM1422() {

  Serial.println(
    "[INIT] BM1422 initialization start"
  );


  uint8_t whoAmI = 0;


  if (
    !readRegister(
      REG_WIA,
      whoAmI
    )
  ) {
    Serial.println(
      "[ERROR] BM1422AGMVにアクセスできません。"
    );

    return false;
  }


  Serial.print(
    "[INIT] WIA register = 0x"
  );

  Serial.println(
    whoAmI,
    HEX
  );


  if (
    whoAmI
    !=
    WIA_EXPECTED
  ) {
    Serial.println(
      "[ERROR] 想定したBM1422AGMVではない可能性があります。"
    );

    return false;
  }


  if (
    !writeRegister(
      REG_CNTL1,
      CNTL1_VALUE
    )
  ) {
    Serial.println(
      "[ERROR] CNTL1設定失敗"
    );

    return false;
  }


  delay(1);


  if (
    !writeRegister16(
      REG_CNTL4,
      0x0000
    )
  ) {
    Serial.println(
      "[ERROR] CNTL4設定失敗"
    );

    return false;
  }


  if (
    !writeRegister(
      REG_CNTL2,
      CNTL2_VALUE
    )
  ) {
    Serial.println(
      "[ERROR] CNTL2設定失敗"
    );

    return false;
  }


  if (
    !writeRegister(
      REG_AVE_A,
      AVE_A_VALUE
    )
  ) {
    Serial.println(
      "[ERROR] AVE_A設定失敗"
    );

    return false;
  }


  Serial.println(
    "[INIT] BM1422AGMV init OK"
  );


  return true;
}


// ============================================================
// 地磁気データ読み取り
// ============================================================

bool readMagneticData(
  float &x,
  float &y,
  float &z
) {

  if (
    !writeRegister(
      REG_CNTL3,
      CNTL3_FORCE
    )
  ) {
    sensorReadFailCount++;

    Serial.println(
      "[ERROR] 測定開始失敗"
    );

    return false;
  }


  uint8_t status = 0;

  bool dataReady = false;


  for (
    int i = 0;
    i < 50;
    i++
  ) {

    if (
      !readRegister(
        REG_STA1,
        status
      )
    ) {
      sensorReadFailCount++;

      Serial.println(
        "[ERROR] STA1読み取り失敗"
      );

      return false;
    }


    if (status & 0x40) {
      dataReady = true;
      break;
    }


    delay(1);
  }


  if (!dataReady) {
    sensorReadFailCount++;

    Serial.println(
      "[ERROR] 測定完了待ちタイムアウト"
    );

    return false;
  }


  uint8_t data[6];


  if (
    !readRegisters(
      REG_DATAX,
      data,
      6
    )
  ) {
    sensorReadFailCount++;

    Serial.println(
      "[ERROR] 磁気データ読み取り失敗"
    );

    return false;
  }


  int16_t rawX =
    (int16_t)(
      ((uint16_t)data[1] << 8)
      |
      data[0]
    );


  int16_t rawY =
    (int16_t)(
      ((uint16_t)data[3] << 8)
      |
      data[2]
    );


  int16_t rawZ =
    (int16_t)(
      ((uint16_t)data[5] << 8)
      |
      data[4]
    );


  // ★診断保存
  latestRawX = rawX;
  latestRawY = rawY;
  latestRawZ = rawZ;


  x =
    rawX /
    SENSITIVITY_14BIT;


  y =
    rawY /
    SENSITIVITY_14BIT;


  z =
    rawZ /
    SENSITIVITY_14BIT;


  sensorReadSuccessCount++;

  lastSensorSuccessTime =
    millis();


  return true;
}


// ============================================================
// 方位計算
// ============================================================

void calculateHeading(
  float x,
  float y,
  float z
) {

  latestX = x;
  latestY = y;
  latestZ = z;


  float correctedX =
    x -
    offsetX;


  float correctedY =
    y -
    offsetY;


  // ★診断保存
  latestCorrectedX =
    correctedX;

  latestCorrectedY =
    correctedY;


  // 磁場の大きさ
  latestFieldStrength =
    sqrt(
      x * x
      +
      y * y
      +
      z * z
    );


  // v0.3本来の方位計算
  float heading =
    atan2(
      correctedY,
      correctedX
    )
    *
    180.0
    /
    PI;


  heading =
    normalizeHeading(
      heading
    );


  // 基板「前」方向補正
  float forwardHeading =
    normalizeHeading(
      heading
      -
      SENSOR_FORWARD_OFFSET
    );


  float diffFromNorth =
    angleDifference(
      forwardHeading,
      0.0
    );


  latestHeading =
    heading;


  latestForwardHeading =
    forwardHeading;


  latestDiffFromNorth =
    diffFromNorth;


  latestIsNorth =
    fabs(diffFromNorth)
    <=
    NORTH_RANGE;


  latestSensorUpdateId++;
}


// ============================================================
// キャリブレーション
// ============================================================

bool calibrateMagnetometer() {

  calibrationReady = false;
  latestIsNorth = false;

  resetMotorFeedbackState();


  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "[CAL] Calibration start"
  );

  Serial.println(
    "[CAL] 10秒間、水平にしてゆっくり一周以上回してください。"
  );

  Serial.println(
    "[CAL] コンパス・スマホ・鍵・磁石などを近づけないでください。"
  );

  Serial.println(
    "======================================"
  );


  float minX = 99999.0;
  float maxX = -99999.0;

  float minY = 99999.0;
  float maxY = -99999.0;


  int sampleCount = 0;


  unsigned long startTime =
    millis();


  unsigned long lastCalibrationPrintTime =
    0;


  while (
    millis() -
    startTime
    <
    10000
  ) {

    float x;
    float y;
    float z;


    if (
      readMagneticData(
        x,
        y,
        z
      )
    ) {

      if (x < minX) {
        minX = x;
      }

      if (x > maxX) {
        maxX = x;
      }

      if (y < minY) {
        minY = y;
      }

      if (y > maxY) {
        maxY = y;
      }


      sampleCount++;


      if (
        millis()
        -
        lastCalibrationPrintTime
        >=
        500
      ) {

        lastCalibrationPrintTime =
          millis();


        Serial.print(
          "[CAL] X="
        );

        Serial.print(
          x,
          2
        );


        Serial.print(
          " Y="
        );

        Serial.print(
          y,
          2
        );


        Serial.print(
          " Z="
        );

        Serial.print(
          z,
          2
        );


        Serial.print(
          " samples="
        );

        Serial.println(
          sampleCount
        );
      }
    }


    delay(100);
  }


  if (sampleCount < 10) {

    Serial.println(
      "[CAL][ERROR] サンプル数不足"
    );

    calibrationReady = false;

    vibrationErrorSignal();

    return false;
  }


  offsetX =
    (maxX + minX)
    /
    2.0;


  offsetY =
    (maxY + minY)
    /
    2.0;


  // ★診断保存
  calibrationMinX = minX;
  calibrationMaxX = maxX;

  calibrationMinY = minY;
  calibrationMaxY = maxY;

  calibrationRangeX =
    maxX -
    minX;

  calibrationRangeY =
    maxY -
    minY;

  calibrationSampleCount =
    sampleCount;


  calibrationReady =
    true;


  resetMotorFeedbackState();


  Serial.println();
  Serial.println(
    "========== CALIBRATION RESULT =========="
  );


  Serial.print(
    "samples="
  );

  Serial.println(
    calibrationSampleCount
  );


  Serial.print(
    "minX="
  );

  Serial.print(
    calibrationMinX,
    2
  );


  Serial.print(
    " maxX="
  );

  Serial.print(
    calibrationMaxX,
    2
  );


  Serial.print(
    " rangeX="
  );

  Serial.println(
    calibrationRangeX,
    2
  );


  Serial.print(
    "minY="
  );

  Serial.print(
    calibrationMinY,
    2
  );


  Serial.print(
    " maxY="
  );

  Serial.print(
    calibrationMaxY,
    2
  );


  Serial.print(
    " rangeY="
  );

  Serial.println(
    calibrationRangeY,
    2
  );


  Serial.print(
    "offsetX="
  );

  Serial.println(
    offsetX,
    2
  );


  Serial.print(
    "offsetY="
  );

  Serial.println(
    offsetY,
    2
  );


  Serial.print(
    "SENSOR_FORWARD_OFFSET="
  );

  Serial.println(
    SENSOR_FORWARD_OFFSET,
    1
  );


  Serial.print(
    "NORTH_RANGE=±"
  );

  Serial.println(
    NORTH_RANGE,
    1
  );


  Serial.print(
    "CENTER_ENTER_RANGE=±"
  );

  Serial.println(
    CENTER_ENTER_RANGE,
    1
  );


  Serial.print(
    "CENTER_EXIT_RANGE=±"
  );

  Serial.println(
    CENTER_EXIT_RANGE,
    1
  );


  Serial.println(
    "========================================"
  );

  Serial.println();


  vibrationCalibrationDoneSignal();


  return true;
}


// ============================================================
// ★診断表示
// ============================================================

void printDebugInfo() {

  unsigned long now =
    millis();


  unsigned long sensorAge =
    now -
    lastSensorSuccessTime;


  Serial.print(
    "[DIAG]"
  );


  // ----------------------------------------------------------
  // 時刻
  // ----------------------------------------------------------

  Serial.print(
    " t="
  );

  Serial.print(now);


  // ----------------------------------------------------------
  // 生データ
  // ----------------------------------------------------------

  Serial.print(
    " rawX="
  );

  Serial.print(
    latestRawX
  );


  Serial.print(
    " rawY="
  );

  Serial.print(
    latestRawY
  );


  Serial.print(
    " rawZ="
  );

  Serial.print(
    latestRawZ
  );


  // ----------------------------------------------------------
  // μT値
  // ----------------------------------------------------------

  Serial.print(
    " X="
  );

  Serial.print(
    latestX,
    2
  );


  Serial.print(
    " Y="
  );

  Serial.print(
    latestY,
    2
  );


  Serial.print(
    " Z="
  );

  Serial.print(
    latestZ,
    2
  );


  // ----------------------------------------------------------
  // キャリブレーション補正後
  // ----------------------------------------------------------

  Serial.print(
    " corrX="
  );

  Serial.print(
    latestCorrectedX,
    2
  );


  Serial.print(
    " corrY="
  );

  Serial.print(
    latestCorrectedY,
    2
  );


  // ----------------------------------------------------------
  // 方位
  // ----------------------------------------------------------

  Serial.print(
    " Heading="
  );

  Serial.print(
    latestHeading,
    1
  );


  Serial.print(
    " Forward="
  );

  Serial.print(
    latestForwardHeading,
    1
  );


  Serial.print(
    " Diff="
  );

  Serial.print(
    latestDiffFromNorth,
    1
  );


  Serial.print(
    " AbsDiff="
  );

  Serial.print(
    fabs(latestDiffFromNorth),
    1
  );


  // ----------------------------------------------------------
  // 平滑化
  // ----------------------------------------------------------

  Serial.print(
    " Smooth="
  );


  if (smoothedAbsDiffReady) {

    Serial.print(
      smoothedAbsDiff,
      1
    );

  } else {

    Serial.print("-");
  }


  // ----------------------------------------------------------
  // 磁場強度
  // ----------------------------------------------------------

  Serial.print(
    " Field="
  );

  Serial.print(
    latestFieldStrength,
    1
  );


  // ----------------------------------------------------------
  // 北判定
  // ----------------------------------------------------------

  Serial.print(
    " IsNorth="
  );

  Serial.print(
    latestIsNorth
    ?
    "YES"
    :
    "NO"
  );


  // ----------------------------------------------------------
  // モーター指示
  // ----------------------------------------------------------

  Serial.print(
    " MotorCmd="
  );

  Serial.print(
    motorCommandOn
    ?
    "ON"
    :
    "OFF"
  );


  // ----------------------------------------------------------
  // モード
  // ----------------------------------------------------------

  Serial.print(
    " Mode="
  );


  if (!calibrationReady) {

    Serial.print(
      "NOT_CALIBRATED"
    );

  } else if (!latestIsNorth) {

    Serial.print(
      "OUTSIDE"
    );

  } else if (centerVibrationMode) {

    Serial.print(
      "CONTINUOUS"
    );

  } else {

    Serial.print(
      motorPulseOn
      ?
      "PULSE_ON"
      :
      "PULSE_OFF"
    );
  }


  // ----------------------------------------------------------
  // センサー状態
  // ----------------------------------------------------------

  Serial.print(
    " ReadOK="
  );

  Serial.print(
    sensorReadSuccessCount
  );


  Serial.print(
    " ReadFail="
  );

  Serial.print(
    sensorReadFailCount
  );


  Serial.print(
    " SensorAge="
  );

  Serial.print(
    sensorAge
  );


  Serial.println(
    "ms"
  );
}


// ============================================================
// ヘルプ
// ============================================================

void printHelp() {

  Serial.println();

  Serial.println(
    "===== v0.4 diagnostic help ====="
  );


  Serial.println(
    "c : キャリブレーションをやり直す"
  );


  Serial.println(
    "i : この説明を表示する"
  );


  Serial.println(
    "n : v0.4では使用しない"
  );


  Serial.println();


  Serial.print(
    "SENSOR_FORWARD_OFFSET = "
  );

  Serial.println(
    SENSOR_FORWARD_OFFSET,
    1
  );


  Serial.print(
    "NORTH_RANGE = ±"
  );

  Serial.println(
    NORTH_RANGE,
    1
  );


  Serial.print(
    "CENTER_ENTER_RANGE = ±"
  );

  Serial.println(
    CENTER_ENTER_RANGE,
    1
  );


  Serial.print(
    "CENTER_EXIT_RANGE = ±"
  );

  Serial.println(
    CENTER_EXIT_RANGE,
    1
  );


  Serial.println(
    "==============================="
  );

  Serial.println();
}


// ============================================================
// Serialコマンド
// ============================================================

void handleSerialCommands() {

  while (
    Serial.available()
    >
    0
  ) {

    char command =
      Serial.read();


    if (
      command == '\n'
      ||
      command == '\r'
      ||
      command == ' '
    ) {
      continue;
    }


    if (
      command == 'c'
      ||
      command == 'C'
    ) {

      Serial.println();

      Serial.println(
        "Command: c"
      );

      Serial.println(
        "キャリブレーションをやり直します。"
      );


      calibrateMagnetometer();


      lastSensorReadTime = 0;
      lastDebugPrintTime = 0;

      return;
    }


    if (
      command == 'i'
      ||
      command == 'I'
    ) {

      printHelp();

      continue;
    }


    if (
      command == 'n'
      ||
      command == 'N'
    ) {

      Serial.println();

      Serial.println(
        "v0.4では n による北登録は使用しません。"
      );

      Serial.print(
        "SENSOR_FORWARD_OFFSET = "
      );

      Serial.println(
        SENSOR_FORWARD_OFFSET,
        1
      );

      continue;
    }


    Serial.print(
      "Unknown command: "
    );

    Serial.println(
      command
    );
  }
}


// ============================================================
// センサー更新
// ============================================================

void updateSensorIfNeeded() {

  unsigned long now =
    millis();


  if (
    now -
    lastSensorReadTime
    <
    SENSOR_READ_INTERVAL_MS
  ) {
    return;
  }


  lastSensorReadTime =
    now;


  float x;
  float y;
  float z;


  if (
    !readMagneticData(
      x,
      y,
      z
    )
  ) {

    latestIsNorth = false;

    resetMotorFeedbackState();

    return;
  }


  calculateHeading(
    x,
    y,
    z
  );


  if (
    now -
    lastDebugPrintTime
    >=
    DEBUG_PRINT_INTERVAL_MS
  ) {

    lastDebugPrintTime =
      now;

    printDebugInfo();
  }
}


// ============================================================
// setup
// ============================================================

void setup() {

  Serial.begin(115200);


  pinMode(
    MOTOR_PIN,
    OUTPUT
  );


  resetMotorFeedbackState();


  delay(1000);


  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "Vibe Compass v0.4 DIAGNOSTIC (OFFSET 89.3)"
  );

  Serial.println(
    "Autonomous north detection"
  );

  Serial.println(
    "Outdoor test diagnostic version"
  );

  Serial.println(
    "======================================"
  );

  Serial.println();


  Serial.println(
    "[BOOT] setup started"
  );


  vibrationStartupSignal();


  Serial.println(
    "[BOOT] Wire.begin"
  );


  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );


  Wire.setClock(
    100000
  );


  Serial.println(
    "[BOOT] sensor initialization"
  );


  bm1422Ready =
    initBM1422();


  if (!bm1422Ready) {

    Serial.println(
      "[FATAL] 初期化失敗"
    );

    vibrationErrorSignal();

    return;
  }


  Serial.println(
    "[BOOT] sensor initialized"
  );


  calibrateMagnetometer();


  Serial.println(
    "[READY] diagnostic operation started"
  );

  Serial.println(
    "[READY] Serial Monitor = 115200"
  );

  Serial.println();
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