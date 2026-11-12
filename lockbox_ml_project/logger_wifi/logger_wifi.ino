// =====================================================================
//  WIRELESS DATA LOGGER - the ESP32 runs from a power bank and sends each
//  2 s window (every 1 s, 500 Hz sampling) to your computer over Wi-Fi + MQTT. Run collect_mqtt.py.
//  Wiring: MPU6050 SDA 21 / SCL 22, mic AO on GPIO 33.
//  Keep the servo/buzzer idle while recording (the servo shakes the box).
// =====================================================================
#include <WiFi.h>
#include <PubSubClient.h>
#include "features.h"

const char* WIFI_SSID = "GAGA";
const char* WIFI_PASS = "Gege0507";
const char* MQTT_HOST = "broker.hivemq.com";
const int   MQTT_PORT = 1883;
#define REC_TOPIC "lockbox_gavin_7f3a_rec/window"   // must match collect_mqtt.py

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
unsigned long lastTry = 0;

void connectMqtt() {                       // non-blocking: one attempt every 3 s
  if (mqtt.connected() || millis() - lastTry < 3000) return;
  lastTry = millis();
  String id = "logger-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  if (mqtt.connect(id.c_str())) Serial.println("# MQTT connected");
  else Serial.printf("# MQTT failed, rc=%d\n", mqtt.state());
}

void setup() {
  Serial.begin(115200);
  mpuInit();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("# connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println("\n# WiFi connected");
  mqtt.setServer(MQTT_HOST, MQTT_PORT);

  calibrateSensors();                          // keep the box still and quiet for ~1 s
  mlReset(millis());
  Serial.println("# logger ready");
}

void loop() {
  connectMqtt();
  mqtt.loop();

  float f[ML_N_FEATURES];
  if (mlPoll(f)) {
    char buf[200];
    snprintf(buf, sizeof(buf), "%.4f,%.4f,%.3f,%.0f,%.0f,%.1f,%.0f,%.4f,%.4f,%.4f,%.4f,%.2f,%.1f,%.2f,%.0f",
             f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7],
             f[8], f[9], f[10], f[11], f[12], f[13], f[14]);
    Serial.println(buf);
    if (mqtt.connected()) mqtt.publish(REC_TOPIC, buf);   // lost windows are simply not recorded
  }
}
