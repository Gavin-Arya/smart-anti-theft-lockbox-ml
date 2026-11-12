#pragma once
// =====================================================================
//  features.h  -  SHARED by logger.ino, logger_wifi.ino and lockbox.ino
//  All sketch folders must contain an IDENTICAL copy, so the features recorded
//  for training are computed exactly like the ones the model sees on the box.
//
//  Sampling: MPU6050 accel+gyro and the sound sensor are read at 500 Hz.
//  A window is 2 s long and a NEW window is produced every 1 s (50 % overlap).
//  Call mlPoll() on every loop(); it takes a sample whenever 2 ms have passed.
// =====================================================================
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

#define MIC_PIN       33     // sound sensor AO
#define MPU_ADDR      0x68   // MPU6050 on SDA 21 / SCL 22
#define ML_N_FEATURES 15

const unsigned long ML_TICK_US  = 2000;   // 500 samples per second
const unsigned long ML_BLOCK_MS = 1000;   // window = last 2 blocks, one window per block
const int   ML_MIC_HIGH   = 80;           // sound level that starts a peak (a knock)
const int   ML_MIC_LOW    = 35;
const float ML_ACC_HIGH   = 0.30f;        // accel deviation (g) that starts a peak
const float ML_ACC_LOW    = 0.15f;
const float ML_CROSS_BAND = 0.03f;        // dead band (g) for the oscillation counter
const float ACC_LSB  = 4096.0f;           // +-8 g range
const float GYRO_LSB = 65.5f;             // +-500 deg/s range

int   micMid = 2048;                      // mic idle level (calibrateSensors)
float gyroBias[3] = {0, 0, 0};            // gyro offset (calibrateSensors)
float mlLastDev  = 0;                     // latest accel deviation from 1 g (dashboard)
int   mlMicLevel = 0;                     // sound level of the latest sample
bool  mlSampled  = false;                 // true when the last mlPoll() took a sample

// ---------- MPU6050 (raw I2C, no library) ----------
static void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void mpuInit() {
  Wire.begin(21, 22);
  Wire.setClock(400000);
  mpuWrite(0x6B, 0x00); delay(50);   // wake up
  mpuWrite(0x6B, 0x01);              // clock from the gyro PLL
  mpuWrite(0x1A, 0x01);              // low-pass 184 Hz
  mpuWrite(0x19, 0x01);              // 1 kHz / 2 = 500 Hz output rate
  mpuWrite(0x1C, 0x10);              // accel +-8 g
  mpuWrite(0x1B, 0x08);              // gyro  +-500 deg/s
}

// a[3] in g, g[3] in deg/s (bias removed)
static bool readImu(float *a, float *g) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (bool)true) != 14) return false;
  int16_t v[7];
  for (int i = 0; i < 7; i++) {
    int hi = Wire.read();
    int lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  for (int k = 0; k < 3; k++) {
    a[k] = v[k] / ACC_LSB;
    g[k] = v[4 + k] / GYRO_LSB - gyroBias[k];
  }
  return true;
}

// keep the box still and quiet for ~1.3 s
void calibrateSensors() {
  long m = 0;
  for (int i = 0; i < 300; i++) { m += analogRead(MIC_PIN); delay(2); }
  micMid = m / 300;
  gyroBias[0] = gyroBias[1] = gyroBias[2] = 0;
  float s[3] = {0, 0, 0}, a[3], g[3];
  int n = 0;
  for (int i = 0; i < 300; i++) {
    if (readImu(a, g)) { s[0] += g[0]; s[1] += g[1]; s[2] += g[2]; n++; }
    delay(2);
  }
  if (n) for (int k = 0; k < 3; k++) gyroBias[k] = s[k] / n;
}

// ---------- One-second block of statistics ----------
struct Blk {
  int n, dpeaks, jn, mn, mpeak, mpeaks, cross;
  double sd, sd2, sx, sx2, sy, sy2, sz, sz2;
  double dmax, jsum, gsum, gmax, gv[3], msum;
};

static Blk wPrev, wCur;
static bool wHavePrev = false, wHavePrevSample = false;
static unsigned long wBlockStartMs = 0, wLastTickUs = 0;
static float wPrevMag = 1.0f, wEma = 1.0f;
static int   wCrossState = 0;
static bool  wAccIn = false, wMicIn = false;

void mlReset(unsigned long nowMs) {
  memset(&wPrev, 0, sizeof(wPrev));
  memset(&wCur, 0, sizeof(wCur));
  wHavePrev = false; wHavePrevSample = false;
  wBlockStartMs = nowMs; wLastTickUs = micros();
  wPrevMag = 1.0f; wEma = 1.0f; wCrossState = 0;
  wAccIn = wMicIn = false;
}

// f[0] acc_mean  f[1] acc_std  f[2] acc_max  f[3] acc_peaks
// f[4] mic_peak  f[5] mic_mean f[6] mic_peaks
// f[7] jerk_mean f[8] ax_std   f[9] ay_std   f[10] az_std
// f[11] gyro_mean f[12] gyro_max f[13] gyro_net (net rotation, deg) f[14] acc_cross
static bool combineBlocks(const Blk &p, const Blk &c, float *f) {
  int n = p.n + c.n, mn = p.mn + c.mn;
  if (n < 100 || mn < 100) return false;
  double mean = (p.sd + c.sd) / n;
  double var  = (p.sd2 + c.sd2) / n - mean * mean;
  f[0] = (float)mean;
  f[1] = (float)(var > 0 ? sqrt(var) : 0);
  f[2] = (float)(p.dmax > c.dmax ? p.dmax : c.dmax);
  f[3] = (float)(p.dpeaks + c.dpeaks);
  f[4] = (float)(p.mpeak > c.mpeak ? p.mpeak : c.mpeak);
  f[5] = (float)((p.msum + c.msum) / mn);
  f[6] = (float)(p.mpeaks + c.mpeaks);
  int jn = p.jn + c.jn;
  f[7] = jn ? (float)((p.jsum + c.jsum) / jn) : 0;
  double mx = (p.sx + c.sx) / n, my = (p.sy + c.sy) / n, mz = (p.sz + c.sz) / n;
  double vx = (p.sx2 + c.sx2) / n - mx * mx;
  double vy = (p.sy2 + c.sy2) / n - my * my;
  double vz = (p.sz2 + c.sz2) / n - mz * mz;
  f[8]  = (float)(vx > 0 ? sqrt(vx) : 0);
  f[9]  = (float)(vy > 0 ? sqrt(vy) : 0);
  f[10] = (float)(vz > 0 ? sqrt(vz) : 0);
  f[11] = (float)((p.gsum + c.gsum) / n);
  f[12] = (float)(p.gmax > c.gmax ? p.gmax : c.gmax);
  double g0 = p.gv[0] + c.gv[0], g1 = p.gv[1] + c.gv[1], g2 = p.gv[2] + c.gv[2];
  f[13] = (float)sqrt(g0 * g0 + g1 * g1 + g2 * g2);
  f[14] = (float)(p.cross + c.cross);
  return true;
}

// Call on EVERY loop(). Returns true when a new window is ready (about once per second).
bool mlPoll(float *f) {
  mlSampled = false;
  unsigned long nowUs = micros();
  if (nowUs - wLastTickUs < ML_TICK_US) return false;
  float dt = (nowUs - wLastTickUs) / 1e6f;
  if (dt > 0.01f) { dt = 0.01f; wHavePrevSample = false; }   // the loop was blocked: no jerk this time
  wLastTickUs = nowUs;

  // sound: strongest deviation in a few quick readings
  int lvl = 0;
  for (int i = 0; i < 3; i++) { int v = abs(analogRead(MIC_PIN) - micMid); if (v > lvl) lvl = v; }
  mlMicLevel = lvl; mlSampled = true;

  Blk &b = wCur;
  b.mn++; b.msum += lvl;
  if (lvl > b.mpeak) b.mpeak = lvl;
  if (lvl > ML_MIC_HIGH && !wMicIn) { b.mpeaks++; wMicIn = true; }
  else if (lvl < ML_MIC_LOW) wMicIn = false;

  float a[3], g[3];
  if (readImu(a, g)) {
    float mag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    float dev = fabsf(mag - 1.0f);
    mlLastDev = dev;
    b.n++;
    b.sd += dev; b.sd2 += (double)dev * dev;
    if (dev > b.dmax) b.dmax = dev;
    if (dev > ML_ACC_HIGH && !wAccIn) { b.dpeaks++; wAccIn = true; }
    else if (dev < ML_ACC_LOW) wAccIn = false;
    if (wHavePrevSample) { b.jsum += fabsf(mag - wPrevMag); b.jn++; }
    wPrevMag = mag; wHavePrevSample = true;
    b.sx += a[0]; b.sx2 += (double)a[0] * a[0];
    b.sy += a[1]; b.sy2 += (double)a[1] * a[1];
    b.sz += a[2]; b.sz2 += (double)a[2] * a[2];
    float gm = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
    b.gsum += gm; if (gm > b.gmax) b.gmax = gm;
    for (int k = 0; k < 3; k++) b.gv[k] += g[k] * dt;
    // oscillation counter: crossings of the magnitude around its slow average
    wEma += 0.01f * (mag - wEma);
    float hp = mag - wEma;
    if (hp > ML_CROSS_BAND && wCrossState != 1)        { if (wCrossState == -1) b.cross++; wCrossState = 1; }
    else if (hp < -ML_CROSS_BAND && wCrossState != -1) { if (wCrossState == 1) b.cross++; wCrossState = -1; }
  }

  unsigned long nowMs = millis();
  if (nowMs - wBlockStartMs >= ML_BLOCK_MS) {
    bool ready = false;
    if (wHavePrev && nowMs - wBlockStartMs < 2 * ML_BLOCK_MS) ready = combineBlocks(wPrev, wCur, f);
    wPrev = wCur; wHavePrev = true;
    memset(&wCur, 0, sizeof(wCur));
    wBlockStartMs = nowMs;
    return ready;
  }
  return false;
}
