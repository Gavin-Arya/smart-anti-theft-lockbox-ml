#include <Wire.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include "features.h"   
#include "model.h"      

// ---------- Pins ----------
#define TRIG_PIN   25
#define ECHO_PIN   34
#define SERVO_PIN  13
#define BUZZER_PIN 27

// ---------- Buzzer ----------
const int BUZZER_FREQ = 2700;   
const int BUZZER_DUTY = 16;     
const int WRONG_FREQ  = 1000;   
const int WRONG_DUTY  = 1;      
const int BUZZER_RES  = 12;
const int BUZZER_CH   = 4;      
const unsigned long ALARM_TIME = 8000;   


const int   VOL_DUTY[3] = {4, 16, 300};
const char* VOL_NAME[3] = {"low", "medium", "high"};
int volLevel   = 1;
int pendingVol = -1;
Preferences prefs;

// ---------- Servo ----------
const int LOCK_ANGLE   = 0;     
const int UNLOCK_ANGLE = 90;
const unsigned long UNLOCK_TIME = 15000;  

// ---------- ML ----------
const int   ML_CONFIRM      = 2;      
const float ML_MIN_ACC_MAX = 0.35f;  
                                    
int mlBadCount = 0;
int lastCls    = -1;
unsigned long mlBlockUntil = 0; 

// ---------- Lid (ultrasonic) ----------
const float DIST_MAX_CM = 10.0; 

// ---------- Secret knock ----------
const int   KNOCK_THRESHOLD = 80;
const int   KNOCK_RELEASE   = 35;
const unsigned long KNOCK_QUIET_MS = 80;
const unsigned long SHORT_GAP_MIN = 150;
const unsigned long SHORT_GAP_MAX = 1100;
const unsigned long LONG_GAP_MIN  = 1300;
const unsigned long LONG_GAP_MAX  = 6000;
const unsigned long KNOCK_MIN_GAP = 120;
const unsigned long KNOCK_TIMEOUT = 8000;
const bool  DEBUG_MIC = true;

// ---------- State ----------
enum State { LOCKED, ALARM, UNLOCKED };
State state = LOCKED;

Servo lockServo;

float lastDist = -1;      
float distBase = -1;

unsigned long knockTimes[3];
int  knockCount = 0;
unsigned long lastKnock = 0;
bool wrongKnock = false;
bool micArmed = true;
unsigned long quietSince = 0;

unsigned long micIgnoreUntil = 0;
unsigned long unlockedAt     = 0;
unsigned long alarmStart     = 0;
unsigned long lidSince       = 0;
unsigned long lastDistRead   = 0;

// ---------- WiFi + MQTT ----------
const char* WIFI_SSID = "GAGA";
const char* WIFI_PASS = "Gege0507";
const char* MQTT_HOST = "broker.hivemq.com";
const int   MQTT_PORT = 1883;
#define TOPIC_BASE "lockbox_gavin_7f3a"   

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
unsigned long lastPublish = 0, lastMqttTry = 0;
int servoAngle = LOCK_ANGLE;
int micPeakMax = 0;
int pendingCmd = 0;   

const char* stateName() {
  return state == LOCKED ? "LOCKED" : state == ALARM ? "ALARM" : "UNLOCKED";
}

void publishTelemetry() {
  if (!mqtt.connected()) return;
  char buf[230];
  snprintf(buf, sizeof(buf),
    "{\"state\":\"%s\",\"servo\":%d,\"buzzer\":%d,\"dist\":%.1f,\"mic\":%d,\"shake\":%.2f,\"vol\":\"%s\",\"cls\":\"%s\"}",
    stateName(), servoAngle, state == ALARM ? 1 : 0,
    lastDist, micPeakMax, mlLastDev, VOL_NAME[volLevel],
    lastCls >= 0 ? CLASS_NAMES[lastCls] : "-");
  mqtt.publish(TOPIC_BASE "/telemetry", buf);
  micPeakMax = 0;
}

void logEvent(const char* msg) {
  Serial.println(msg);
  if (mqtt.connected()) {
    mqtt.publish(TOPIC_BASE "/event", msg);
    publishTelemetry();
  }
}

void onMessage(char* topic, byte* payload, unsigned int len) {
  String m;
  for (unsigned int i = 0; i < len; i++) m += (char)payload[i];
  Serial.printf("MQTT cmd: %s\n", m.c_str());
  if (m == "unlock") pendingCmd = 1;
  else if (m == "lock") pendingCmd = 2;
  else if (m == "vol:low")    pendingVol = 0;
  else if (m == "vol:medium") pendingVol = 1;
  else if (m == "vol:high")   pendingVol = 2;
}

void mqttLoop() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!mqtt.connected()) {
    if (millis() - lastMqttTry > 5000) {      
      lastMqttTry = millis();
      String id = "lockbox-" + String((uint32_t)ESP.getEfuseMac(), HEX);
      if (mqtt.connect(id.c_str(), nullptr, nullptr, TOPIC_BASE "/status", 0, true, "offline")) {
        mqtt.publish(TOPIC_BASE "/status", "online", true);
        mqtt.subscribe(TOPIC_BASE "/command");
      }
    }
    return;
  }
  mqtt.loop();
  if (millis() - lastPublish >= 1000) {
    lastPublish = millis();
    publishTelemetry();
  }
}

// ---------- Buzzer ----------
void buzzerInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(BUZZER_PIN, BUZZER_FREQ, BUZZER_RES);
  ledcWrite(BUZZER_PIN, 0);
#else
  ledcSetup(BUZZER_CH, BUZZER_FREQ, BUZZER_RES);
  ledcAttachPin(BUZZER_PIN, BUZZER_CH);
  ledcWrite(BUZZER_CH, 0);
#endif
}

void buzzerSetFreq(int freq) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcChangeFrequency(BUZZER_PIN, freq, BUZZER_RES);
#else
  ledcSetup(BUZZER_CH, freq, BUZZER_RES);
#endif
}

void buzzer(bool on, int dutyOn = BUZZER_DUTY) {
  int duty = on ? dutyOn : 0;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(BUZZER_PIN, duty);
#else
  ledcWrite(BUZZER_CH, duty);
#endif
}

void beep(int ms) {
  buzzer(true);
  delay(ms);
  buzzer(false);
  micIgnoreUntil = millis() + 150;
  mlReset(millis());
}

// two very short quiet ticks = wrong knock pattern
void wrongBeep() {
  buzzerSetFreq(WRONG_FREQ);
  for (int i = 0; i < 2; i++) {
    buzzer(true, WRONG_DUTY);
    delay(40);
    buzzer(false);
    delay(100);
  }
  buzzerSetFreq(BUZZER_FREQ);
  micIgnoreUntil = millis() + 400;
  mlReset(millis());
}

// ---------- Servo (stays attached so its PWM channel is never re-allocated) ----------
void moveServo(int angle, unsigned long holdMs = 1000) {
  if (!lockServo.attached()) lockServo.attach(SERVO_PIN);
  unsigned long t = millis();
  while (millis() - t < holdMs) {
    lockServo.write(angle);
    delay(20);
  }
  servoAngle = angle;
  mlBlockUntil = millis() + 1500;   
}

// ---------- Ultrasonic ----------
float readDistanceCm() {          
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 12000);
  if (us == 0) return -1;
  return us * 0.0343f / 2.0f;
}

void updateDistance(unsigned long now) {
  if (now - lastDistRead < 100) return;     
  lastDistRead = now;
  lastDist = readDistanceCm();
}

bool detectLid(unsigned long now) {
  bool open = (lastDist < 0) || (lastDist > DIST_MAX_CM);
  if (open) {
    if (lidSince == 0) lidSince = now;
    if (now - lidSince > 300) return true;  
  } else {
    lidSince = 0;
  }
  return false;
}

// ---------- Calibration ----------
void calibrate() {
  calibrateSensors();                        
  float sum = 0; int n = 0;
  for (int i = 0; i < 15; i++) {
    float d = readDistanceCm();
    if (d > 0) { sum += d; n++; }
    delay(60);
  }
  distBase = n ? sum / n : -1;
  lastDist = distBase;
  Serial.printf("Calibrated: micMid=%d distBase=%.1f cm\n", micMid, distBase);
  if (distBase < 0) Serial.println("WARNING: no ultrasonic echo, check wiring and aim");
}

// ---------- Secret knock (rule based) ----------
bool detectKnockPattern(unsigned long now, int level) {
  if (knockCount > 0 && now - lastKnock > KNOCK_TIMEOUT) knockCount = 0;

  if (now < micIgnoreUntil) {
    micArmed = false;
    quietSince = 0;
    return false;
  }

  if (level > micPeakMax) micPeakMax = level;

  if (!micArmed) {                           
    if (level < KNOCK_RELEASE) {
      if (quietSince == 0) quietSince = now;
      if (now - quietSince >= KNOCK_QUIET_MS) { micArmed = true; quietSince = 0; }
    } else {
      quietSince = 0;
    }
    return false;
  }

  if (level > KNOCK_THRESHOLD && now - lastKnock > KNOCK_MIN_GAP) {
    micArmed = false;
    quietSince = 0;
    lastKnock = now;
    knockTimes[knockCount++] = now;
    if (DEBUG_MIC) Serial.printf("Knock %d (level %d)\n", knockCount, level);

    if (knockCount == 3) {
      unsigned long g1 = knockTimes[1] - knockTimes[0];
      unsigned long g2 = knockTimes[2] - knockTimes[1];
      knockCount = 0;
      Serial.printf("Gaps: %lu, %lu\n", g1, g2);
      bool shortOk = (g1 >= SHORT_GAP_MIN && g1 <= SHORT_GAP_MAX);
      bool longOk  = (g2 >= LONG_GAP_MIN  && g2 <= LONG_GAP_MAX);
      if (shortOk && longOk) return true;
      logEvent("Wrong pattern");
      wrongKnock = true;
    }
  }
  return false;
}

// ---------- State changes ----------
void enterLocked(unsigned long now) {
  state = LOCKED;
  lidSince = 0; knockCount = 0; mlBadCount = 0;
  micIgnoreUntil = now + 300;
  mlReset(now);
}

void enterUnlocked() {
  buzzer(false);
  moveServo(UNLOCK_ANGLE, 1200);
  unlockedAt = millis();
  state = UNLOCKED;
}

void relock() {
  moveServo(LOCK_ANGLE, 1200);
  delay(1500);
  calibrate();                               
  enterLocked(millis());
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  // 1) Servo FIRST so the box locks immediately
  lockServo.attach(SERVO_PIN);
  moveServo(LOCK_ANGLE, 1500);

  // 2) Everything else afterwards
  prefs.begin("lockbox", false);
  volLevel = prefs.getUChar("vol", 1);
  if (volLevel > 2) volLevel = 1;

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  mpuInit();
  buzzerInit();

  // WiFi gives up after 8 s, so the lock still works offline
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long w = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - w < 8000) delay(200);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);

  delay(1500);                               // let the box settle
  calibrate();                               // keep the box closed and still
  enterLocked(millis());
  Serial.println("LOCKED and armed (ML active)");
}

// ---------- Loop ----------
void loop() {
  unsigned long now = millis();
  mqttLoop();
  wrongKnock = false;
  float f[ML_N_FEATURES];
  bool win = mlPoll(f);                      
  bool correctKnock = mlSampled ? detectKnockPattern(now, mlMicLevel) : false;
  updateDistance(now);

  // alarm volume change from the dashboard
  if (pendingVol >= 0) {
    volLevel = pendingVol;
    pendingVol = -1;
    prefs.putUChar("vol", volLevel);
    if (state != ALARM) {                    
      buzzer(true, VOL_DUTY[volLevel]);
      delay(300);
      buzzer(false);
      micIgnoreUntil = millis() + 400;
      mlReset(millis());
    }
    char msg[32];
    snprintf(msg, sizeof(msg), "Alarm volume: %s", VOL_NAME[volLevel]);
    logEvent(msg);
  }

  // remote lock / unlock from the dashboard
  if (pendingCmd) {
    int cmd = pendingCmd;
    pendingCmd = 0;
    if (cmd == 1 && state != UNLOCKED) {
      enterUnlocked();
      logEvent("Remote UNLOCK (dashboard)");
    } else if (cmd == 2 && state == UNLOCKED) {
      relock();
      logEvent("Remote LOCK (dashboard)");
    }
    return;
  }

  switch (state) {
    case LOCKED: {
      // --- ML: one prediction every 2 s ---
      int cls = -1;
      float accMaxWin = 0;
      if (now < mlBlockUntil) {
        mlReset(now);                        
      } else if (win) {
        cls = mlPredict(f);
        lastCls = cls;
        accMaxWin = f[2];
        Serial.printf("ML -> %-8s | acc_max %.2f  gyro %.0f  mic_peak %.0f\n",
                      CLASS_NAMES[cls], f[2], f[12], f[4]);
      }

      if (correctKnock) {
        beep(100);
        enterUnlocked();
        logEvent("UNLOCKED");
        break;
      }
      if (wrongKnock) wrongBeep();

      bool mlAlarm = false;
      if (cls == CLS_SHAKE || cls == CLS_CARRIED) {
        if (accMaxWin < ML_MIN_ACC_MAX) {            
          Serial.println("   (ignored: motion too weak)");
          mlBadCount = 0;
        } else if (++mlBadCount >= ML_CONFIRM) {
          mlAlarm = true;
        } else {
          Serial.println("   (shake suspected, waiting for the next window)");
        }
      } else if (cls >= 0) {
        mlBadCount = 0;                      
      }

      if (mlAlarm) {
        alarmStart = now;
        state = ALARM;
        logEvent(cls == CLS_SHAKE ? "ALARM: shake (ML)" : "ALARM: box moved (ML)");
      } else if (detectLid(now)) {
        alarmStart = now;
        state = ALARM;
        logEvent("ALARM: lid opened");
      }
      break;
    }

    case ALARM:
      if (correctKnock) {
        enterUnlocked();
        logEvent("Alarm stopped, UNLOCKED");
      } else if (now - alarmStart >= ALARM_TIME) {
        buzzer(false);                       
        enterLocked(now);
        logEvent("Alarm timed out, LOCKED again");
      } else {
        bool on = (now % 500) < 150;
        buzzer(on, VOL_DUTY[volLevel]);
        if (on) micIgnoreUntil = now + 100;  
      }
      break;

    case UNLOCKED:
      if (now - unlockedAt > UNLOCK_TIME) {
        relock();
        logEvent("LOCKED again");
      }
      break;
  }
}
