/*
 * =========================================================
 * AIR MOUSE v2  |  Feather 32u4 Bluefruit LE + GY-BMI160
 * =========================================================
 * Two movement modes, picked automatically at startup:
 *   GYRO mode : turn/tilt your wrist like a laser pointer
 *   TILT mode : used if the gyro will not start. Hold the mouse
 *               flat at boot; then tilting it moves the cursor
 *               like a joystick (more tilt = faster).
 *
 * Wiring (as on your board):
 *   BMI160 3V3 -> Feather 3V      BMI160 GND -> Feather GND
 *   BMI160 SCL -> Feather pin 3   BMI160 SDA -> Feather pin 5
 *   BMI160 CS  -> 3V              Buttons: A0 (left), A1 (right) -> GND
 *
 * Library: "Adafruit BluefruitLE nRF51"   Board: Adafruit Feather 32u4
 * =========================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"

// ---------------- Pins ----------------
#define LEFT_BUTTON   A0
#define RIGHT_BUTTON  A1
#define LED_PIN       13
#define VBAT_PIN      A9
#define SDA_PIN       5
#define SCL_PIN       3

#define BLE_CS   8
#define BLE_IRQ  7
#define BLE_RST  4
Adafruit_BluefruitLE_SPI ble(BLE_CS, BLE_IRQ, BLE_RST);
#define DEVICE_NAME "COMET AIRMOUSE"

// ---------------- BMI160 registers ----------------
#define REG_CHIP_ID     0x00
#define REG_ERR         0x02
#define REG_PMU_STATUS  0x03
#define REG_GYR_DATA    0x0C
#define REG_ACC_DATA    0x12
#define REG_ACC_CONF    0x40
#define REG_ACC_RANGE   0x41
#define REG_GYR_CONF    0x42
#define REG_GYR_RANGE   0x43
#define REG_CMD         0x7E
const float GYRO_LSB_PER_DPS = 65.6;    // +/-500 deg/s
const float ACC_LSB_PER_G    = 16384.0; // +/-2 g
uint8_t imuAddr = 0x68;

// ---------------- Tuning: GYRO mode ----------------
float cursorSpeed  = 20.0;   // cursor counts per degree of rotation
float smoothing    = 0.45;   // lower = snappier, higher = smoother
float deadzoneDps  = 2.5;
// Sensor mounted with X pointing LEFT, Y pointing to the BACK, Z UP.
//   Left/right = yaw   (turn about Z): turning left gives +Z -> cursor left
//   Up/down    = pitch (turn about X): nose up gives -X     -> cursor up
// If one direction is still reversed, flip just that SIGN.
#define MOUSE_X_AXIS   2
#define MOUSE_X_SIGN  -1
#define MOUSE_Y_AXIS   0
#define MOUSE_Y_SIGN   -1

// ---------------- Tuning: TILT mode ----------------
float tiltDeadzone = 0.08;   // g of tilt ignored around flat (~5 degrees)
float tiltSpeed    = 900.0;  // cursor counts per second at full tilt
float tiltSmooth   = 0.80;
// Tilt right side down -> +X -> cursor right; nose down -> +Y -> cursor down
#define TILT_X_AXIS    0     // 0 = accel X, 1 = accel Y
#define TILT_X_SIGN    1     // flip if left/right is reversed
#define TILT_Y_AXIS    1
#define TILT_Y_SIGN    1     // flip if up/down is reversed

// ---------------- Speed settings ----------------
#define DEBUG_PRINT        0      // 1 = print live sensor values (slows things down)
#define CONN_CHECK_MS      3000   // how often to ask "still connected?" (each ask stalls)
#define MIN_SEND_MS        8      // minimum time between cursor updates

// ---------------- Buttons ----------------
const unsigned long clickThreshold = 180;
const unsigned long scrollDelay    = 120;
const unsigned long debounceMs     = 15;
struct Button {
  uint8_t pin; char hidName; int scrollDir;
  bool down; bool held; unsigned long pressStart; unsigned long lastChange;
};
Button leftBtn  = { LEFT_BUTTON,  'L', -1, false, false, 0, 0 };
Button rightBtn = { RIGHT_BUTTON, 'R', +1, false, false, 0, 0 };

// ---------------- State ----------------
bool gyroMode = true;
float bias[3]   = { 0, 0, 0 };   // gyro bias (deg/s) or accel rest offset (g)
float fX = 0, fY = 0, accX = 0, accY = 0;
unsigned long lastMicros = 0, lastScrollTime = 0, lastConnCheck = 0, lastBattUpdate = 0;
bool connected = false, battServiceOn = false;

// =====================================================
// Software I2C with timeouts
// =====================================================
#define HALF_US 5      // raise to 10-20 if sensor reads start failing
bool busError = false;
void sdaLow()  { digitalWrite(SDA_PIN, LOW); pinMode(SDA_PIN, OUTPUT); }
void sdaFree() { pinMode(SDA_PIN, INPUT_PULLUP); }
void sclLow()  { digitalWrite(SCL_PIN, LOW); pinMode(SCL_PIN, OUTPUT); }
void sclFree() {
  pinMode(SCL_PIN, INPUT_PULLUP);
  unsigned long t = micros();
  while (digitalRead(SCL_PIN) == LOW) {
    if (micros() - t > 2000) { busError = true; return; }
  }
}
bool sdaHigh() { return digitalRead(SDA_PIN) == HIGH; }
void i2cStart() { sdaFree(); sclFree(); delayMicroseconds(HALF_US); sdaLow(); delayMicroseconds(HALF_US); sclLow(); delayMicroseconds(HALF_US); }
void i2cStop()  { sdaLow(); delayMicroseconds(HALF_US); sclFree(); delayMicroseconds(HALF_US); sdaFree(); delayMicroseconds(HALF_US); }
bool i2cWrite(uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    if (b & (1 << i)) sdaFree(); else sdaLow();
    delayMicroseconds(HALF_US); sclFree(); delayMicroseconds(HALF_US); sclLow();
  }
  sdaFree(); delayMicroseconds(HALF_US); sclFree();
  bool ack = !sdaHigh();
  delayMicroseconds(HALF_US); sclLow();
  return ack;
}
uint8_t i2cRead(bool ack) {
  uint8_t b = 0; sdaFree();
  for (int i = 7; i >= 0; i--) {
    delayMicroseconds(HALF_US); sclFree();
    if (sdaHigh()) b |= (1 << i);
    delayMicroseconds(HALF_US); sclLow();
  }
  if (ack) sdaLow(); else sdaFree();
  delayMicroseconds(HALF_US); sclFree(); delayMicroseconds(HALF_US); sclLow(); sdaFree();
  return b;
}
void i2cRecover() {
  sdaFree();
  for (int i = 0; i < 9; i++) { sclLow(); delayMicroseconds(HALF_US); sclFree(); delayMicroseconds(HALF_US); }
  i2cStop();
}
bool regWrite(uint8_t reg, uint8_t val) {
  busError = false; i2cStart();
  bool ok = i2cWrite(imuAddr << 1) && i2cWrite(reg) && i2cWrite(val);
  i2cStop(); return ok && !busError;
}
bool regReadN(uint8_t reg, uint8_t *buf, uint8_t n) {
  busError = false; i2cStart();
  if (!i2cWrite(imuAddr << 1) || !i2cWrite(reg)) { i2cStop(); return false; }
  i2cStart();
  if (!i2cWrite((imuAddr << 1) | 1)) { i2cStop(); return false; }
  for (uint8_t i = 0; i < n; i++) buf[i] = i2cRead(i < n - 1);
  i2cStop(); return !busError;
}
int regRead(uint8_t reg) { uint8_t v; return regReadN(reg, &v, 1) ? v : -1; }

// =====================================================
// BMI160
// =====================================================
bool imuFind() {
  sdaFree(); pinMode(SCL_PIN, INPUT_PULLUP); delay(5);
  Serial.print(F("SDA idle: ")); Serial.println(sdaHigh() ? F("high (good)") : F("LOW (bad)"));
  Serial.print(F("SCL idle: ")); Serial.println(digitalRead(SCL_PIN) ? F("high (good)") : F("LOW (bad)"));
  i2cRecover();
  for (uint8_t a = 0x68; a <= 0x69; a++) {
    imuAddr = a;
    int id = regRead(REG_CHIP_ID);
    Serial.print(F("CHIP_ID at 0x")); Serial.print(a, HEX); Serial.print(F(": "));
    if (id < 0) Serial.println(F("no answer")); else { Serial.print(F("0x")); Serial.println(id, HEX); }
    if (id == 0xD1) return true;
  }
  return false;
}

bool gyroIsOn() {
  int pmu = regRead(REG_PMU_STATUS);
  return pmu >= 0 && ((pmu >> 2) & 0x03) == 0x01;
}

void imuStart() {
  // Fresh start: soft reset, then accel, then gyro on its own
  regWrite(REG_CMD, 0xB6); delay(150);
  regRead(REG_CHIP_ID);
  regWrite(REG_CMD, 0x11); delay(50);                 // accel normal
  for (int attempt = 1; attempt <= 3 && !gyroIsOn(); attempt++) {
    regRead(REG_ERR);                                  // reading clears old flags
    regWrite(REG_CMD, 0x15); delay(250);               // gyro normal
    Serial.print(F("Gyro start attempt ")); Serial.print(attempt);
    Serial.print(F(": PMU 0x")); Serial.print(regRead(REG_PMU_STATUS), HEX);
    Serial.print(F(", ERR 0x")); Serial.println(regRead(REG_ERR), HEX);
  }
  regWrite(REG_ACC_CONF,  0x28);   // 100 Hz
  regWrite(REG_ACC_RANGE, 0x03);   // +/-2 g
  regWrite(REG_GYR_CONF,  0x29);   // 200 Hz
  regWrite(REG_GYR_RANGE, 0x02);   // +/-500 deg/s
  delay(20);
  gyroMode = gyroIsOn();
  Serial.println(gyroMode ? F("MODE: GYRO (rotate to move)")
                          : F("MODE: TILT (gyro would not start - tilt to move)"));
}

// reads gyro (deg/s) in gyro mode, accel (g) in tilt mode
bool imuRead(float v[3]) {
  uint8_t b[6];
  if (!regReadN(gyroMode ? REG_GYR_DATA : REG_ACC_DATA, b, 6)) return false;
  float scale = gyroMode ? GYRO_LSB_PER_DPS : ACC_LSB_PER_G;
  for (uint8_t i = 0; i < 3; i++) {
    int16_t raw = (int16_t)((b[2 * i + 1] << 8) | b[2 * i]);
    v[i] = raw / scale;
  }
  return true;
}

void calibrate() {
  Serial.println(F("Calibrating - keep the mouse flat and still!"));
  digitalWrite(LED_PIN, HIGH);
  float sum[3] = { 0, 0, 0 }, v[3];
  int good = 0;
  for (int i = 0; i < 300; i++) {
    if (imuRead(v)) { for (uint8_t a = 0; a < 3; a++) sum[a] += v[a]; good++; }
    delay(5);
  }
  if (good) for (uint8_t a = 0; a < 3; a++) bias[a] = sum[a] / good;
  digitalWrite(LED_PIN, LOW);
  Serial.print(F("Good samples: ")); Serial.println(good);
  Serial.print(gyroMode ? F("Gyro bias X/Y/Z: ") : F("Accel rest X/Y/Z (g): "));
  Serial.print(bias[0], 2); Serial.print(' ');
  Serial.print(bias[1], 2); Serial.print(' ');
  Serial.println(bias[2], 2);
}

// =====================================================
// BLE helpers
// =====================================================
void bleMove(int x, int y, int wheel) {
  ble.print(F("AT+BLEHIDMOUSEMOVE="));
  ble.print(x); ble.print(','); ble.print(y);
  if (wheel != 0) { ble.print(','); ble.print(wheel); }
  ble.println(); ble.waitForOK();
}
void bleClick(char button) {
  ble.print(F("AT+BLEHIDMOUSEBUTTON=")); ble.print(button); ble.println(F(",click"));
  ble.waitForOK();
}
int batteryPercent() {
  float v = analogRead(VBAT_PIN) * 2.0 * 3.3 / 1024.0;
  return constrain((int)((v - 3.3) / 0.9 * 100.0), 0, 100);
}
void updateBattery() {
  if (!battServiceOn) return;
  ble.print(F("AT+BLEBATTVAL=")); ble.println(batteryPercent()); ble.waitForOK();
}
void fail(const __FlashStringHelper *msg) {
  while (true) {
    Serial.println(msg);
    for (int i = 0; i < 10; i++) { digitalWrite(LED_PIN, HIGH); delay(100); digitalWrite(LED_PIN, LOW); delay(100); }
  }
}

// =====================================================
// Buttons
// =====================================================
void handleButton(Button &b, unsigned long now) {
  bool pressed = (digitalRead(b.pin) == LOW);
  if (pressed != b.down) {
    if (now - b.lastChange < debounceMs) return;
    b.lastChange = now; b.down = pressed;
    if (pressed) { b.pressStart = now; b.held = false; }
    else { if (!b.held && connected) bleClick(b.hidName); b.held = false; }
    return;
  }
  if (b.down) {
    if (!b.held && now - b.pressStart >= clickThreshold) b.held = true;
    if (b.held && connected && now - lastScrollTime >= scrollDelay) {
      bleMove(0, 0, b.scrollDir); lastScrollTime = now;
    }
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 8000) { }   // wait for Serial Monitor (max 8 s)
  delay(200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);

  Serial.println(F("=== AIR MOUSE v2: Feather 32u4 Bluefruit + BMI160 ==="));
  if (!imuFind()) fail(F("BMI160 not found. Check 3V3/GND, SDA->pin 5, SCL->pin 3."));
  imuStart();
  calibrate();

  Serial.println(F("Starting Bluefruit..."));
  if (!ble.begin(false)) fail(F("Bluefruit module not responding."));
  ble.echo(false);
  if (!ble.isVersionAtLeast("0.6.6"))
    fail(F("Bluefruit firmware too old for HID. Update it with the Bluefruit Connect app."));
  ble.sendCommandCheckOK(F("AT+GAPDEVNAME=" DEVICE_NAME));
  if (!ble.sendCommandCheckOK(F("AT+BleHIDEn=On"))) fail(F("Could not enable HID mode."));
  battServiceOn = ble.sendCommandCheckOK(F("AT+BLEBATTEN=1"));
  // Ask for a short BLE connection interval (min 10 ms, max 20 ms) for snappier updates
  if (ble.sendCommandCheckOK(F("AT+GAPINTERVALS=10,20,100,30")))
    Serial.println(F("Fast connection interval set."));
  else
    Serial.println(F("Could not set connection interval (continuing)."));
  if (!ble.reset()) fail(F("Bluefruit reset failed."));
  ble.echo(false);

  Serial.println(F("Ready. Pair \"" DEVICE_NAME "\" in your Bluetooth settings."));
  lastMicros = micros();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  unsigned long now = millis();

  if (now - lastConnCheck >= CONN_CHECK_MS || (!connected && now - lastConnCheck >= 500)) {
    lastConnCheck = now;
    bool c = ble.isConnected();
    if (c != connected) {
      connected = c;
      Serial.println(connected ? F("Connected!") : F("Disconnected."));
      if (connected) updateBattery();
    }
    digitalWrite(LED_PIN, connected ? LOW : !digitalRead(LED_PIN));
  }
  if (connected && now - lastBattUpdate >= 60000UL) { lastBattUpdate = now; updateBattery(); }

  handleButton(leftBtn, now);
  handleButton(rightBtn, now);

  unsigned long nowUs = micros();
  float dt = (nowUs - lastMicros) / 1000000.0;
  lastMicros = nowUs;
  if (dt > 0.10) dt = 0.10;

  float v[3];
  bool ok = imuRead(v);
#if DEBUG_PRINT
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {               // live readout for tuning
    lastPrint = millis();
    if (!ok) Serial.println(F("sensor read FAILED"));
    else {
      Serial.print(gyroMode ? F("gyro deg/s ") : F("tilt g "));
      Serial.print(F("X: ")); Serial.print(v[0] - bias[0], 2);
      Serial.print(F("  Y: ")); Serial.print(v[1] - bias[1], 2);
      Serial.print(F("  Z: ")); Serial.println(v[2] - bias[2], 2);
    }
  }
#endif
  if (!ok) return;
  for (uint8_t a = 0; a < 3; a++) v[a] -= bias[a];

  float moveX, moveY;   // cursor counts this update (before rounding)
  if (gyroMode) {
    float rX = MOUSE_X_SIGN * v[MOUSE_X_AXIS];
    float rY = MOUSE_Y_SIGN * v[MOUSE_Y_AXIS];
    fX = fX * smoothing + rX * (1.0 - smoothing);
    fY = fY * smoothing + rY * (1.0 - smoothing);
    moveX = (fabs(fX) < deadzoneDps ? 0 : fX) * dt * cursorSpeed;
    moveY = (fabs(fY) < deadzoneDps ? 0 : fY) * dt * cursorSpeed;
  } else {
    float tX = TILT_X_SIGN * v[TILT_X_AXIS];
    float tY = TILT_Y_SIGN * v[TILT_Y_AXIS];
    fX = fX * tiltSmooth + tX * (1.0 - tiltSmooth);
    fY = fY * tiltSmooth + tY * (1.0 - tiltSmooth);
    // outside the deadzone, speed grows with the square of the tilt (fine control)
    float ex = fabs(fX) > tiltDeadzone ? (fabs(fX) - tiltDeadzone) : 0;
    float ey = fabs(fY) > tiltDeadzone ? (fabs(fY) - tiltDeadzone) : 0;
    moveX = (fX > 0 ? 1 : -1) * ex * ex * 4.0 * tiltSpeed * dt;
    moveY = (fY > 0 ? 1 : -1) * ey * ey * 4.0 * tiltSpeed * dt;
  }

  if (!connected) { accX = accY = 0; delay(5); return; }

  // keep integrating every sensor sample; send the total in one go
  accX += moveX; accY += moveY;
  static unsigned long lastSend = 0;
  if (millis() - lastSend < MIN_SEND_MS) return;
  int mx = constrain((int)accX, -127, 127);
  int my = constrain((int)accY, -127, 127);
  if (mx != 0 || my != 0) {
    accX -= mx; accY -= my;
    bleMove(mx, my, 0);
    lastSend = millis();
  }
}
