#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// 使用你提供的 MQTT 库（内部维持 WiFi/MQTT 连接，并将订阅消息回调给你）
#include "FangmuMqttLib.h"

/* ===================== 引脚分配 ===================== */
static const int PIN_MQ7_AO      = 1;    // ADC
static const int PIN_FLAME_AO    = 2;    // ADC
static const int PIN_I2C_SDA     = 15;
static const int PIN_I2C_SCL     = 16;

static const int PIN_BUZZER      = 4;    // 蜂鸣器（低电平响）
static const int PIN_PUMP_RELAY  = 6;    // 水泵继电器控制脚
static const int PIN_KEY_TOGGLE  = 17;   // 按键：翻转水泵开关

/* ===================== 继电器有效电平配置 ===================== */
// true : GPIO=HIGH 表示开泵
// false: GPIO=LOW  表示开泵（很多继电器模块是低电平吸合）
static const bool RELAY_ACTIVE_HIGH = false;

/* ===================== MQTT 连接信息（请你替换为你的实际参数） ===================== */
static const char* WIFI_SSID   = "YOUR_WIFI_SSID";
static const char* WIFI_PASS   = "YOUR_WIFI_PASS";
static const char* MQTT_HOST   = "YOUR_MQTT_HOST";
static const uint16_t MQTT_PORT = 1883;
static const char* MQTT_CLIENT_ID = "esp32s3_001";

// 固定 Topic：
// - PUB：下位机 -> 上位机，只发 2 个数据 + 2 个阈值
// - SUB：上位机 -> 下位机，下发阈值设置 & 水泵开关
static const char* MQTT_PUB_TOPIC = "fangmu/dev/esp32s3_001/up";
static const char* MQTT_SUB_TOPIC = "fangmu/dev/esp32s3_001/down";

// 固定 Payload（建议 JSON，字段固定）：
// 1) 上报（PUB）：
//    {"mq7_v":1.23,"flame_v":0.45,"mq7_th":1.20,"flame_th":1.00}
// 2) 下发（SUB）阈值设置：
//    {"cmd":"set_th","mq7_th":1.30,"flame_th":0.90}
// 3) 下发（SUB）水泵控制：
//    {"cmd":"pump","state":1}   // 1=开泵,0=关泵
//    {"cmd":"pump","toggle":1}  // 翻转手动状态

static FangmuMqttClient g_mqtt;

/* ===================== OLED 参数 ===================== */
static const int OLED_W = 128;
static const int OLED_H = 64;
static const uint8_t OLED_ADDR = 0x3C; // 不亮改 0x3D
Adafruit_SSD1306 g_oled(OLED_W, OLED_H, &Wire, -1);

/* ===================== 传感器结构 ===================== */
struct SensorItem {
  const char* name;
  int pin;
  float thresholdV;
  float lastV;
};

static SensorItem g_sensors[] = {
  {"MQ7",   PIN_MQ7_AO,   1.20f, 0.0f},
  {"Flame", PIN_FLAME_AO, 1.00f, 0.0f},
};
static const int SENSOR_COUNT = sizeof(g_sensors) / sizeof(g_sensors[0]);

/* ===================== 状态变量（设备内部用，不上传） ===================== */
static bool g_alarm = false;           // 是否超阈值
static bool g_pumpManual = false;      // 手动期望水泵状态（不报警时生效）
static bool g_pumpOut = false;         // 最终开/关状态
static bool g_buzzerOut = false;       // 最终蜂鸣器状态（响/不响）

/* ===================== 时间控制：上报节流 ===================== */
static uint32_t g_lastPubMs = 0;
static const uint32_t PUB_INTERVAL_MS = 1000; // 1s 上报一次（可改）

/* ===================== 工具：ADC 读电压 ===================== */
static float readVoltageV(int pin) {
  uint32_t mv = analogReadMilliVolts(pin);
  return mv / 1000.0f;
}

/* ===================== 初始化：ADC ===================== */
static void sensorsInit() {
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MQ7_AO,   ADC_11db);
  analogSetPinAttenuation(PIN_FLAME_AO, ADC_11db);
}

/* ===================== 初始化：OLED ===================== */
static bool oledInit() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  if (!g_oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) return false;

  g_oled.clearDisplay();
  g_oled.setTextColor(SSD1306_WHITE);
  g_oled.setTextSize(1);
  g_oled.setCursor(0, 0);
  g_oled.println("Sensors Monitor");
  g_oled.display();
  delay(200);
  return true;
}

/* ===================== 初始化：IO（蜂鸣器/继电器/按键） ===================== */
static void ioInit() {
  pinMode(PIN_BUZZER, OUTPUT);
  // 蜂鸣器：低电平响，所以默认先拉高（不响）
  digitalWrite(PIN_BUZZER, HIGH);

  pinMode(PIN_PUMP_RELAY, OUTPUT);
  // 默认关泵
  if (RELAY_ACTIVE_HIGH) digitalWrite(PIN_PUMP_RELAY, LOW);
  else                  digitalWrite(PIN_PUMP_RELAY, HIGH);

  pinMode(PIN_KEY_TOGGLE, INPUT_PULLUP); // 按下=LOW
}

/* ===================== 读取所有传感器 ===================== */
static void sensorsReadAll() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    g_sensors[i].lastV = readVoltageV(g_sensors[i].pin);
  }
}

/* ===================== 计算报警（任意一个超阈值即报警） ===================== */
static bool computeAlarm() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (g_sensors[i].lastV >= g_sensors[i].thresholdV) return true;
  }
  return false;
}

/* ===================== 按键扫描：翻转水泵手动状态（带去抖） ===================== */
static void keyScanTogglePump() {
  static int lastStable = HIGH;
  static int lastRead = HIGH;
  static uint32_t lastChangeMs = 0;

  int r = digitalRead(PIN_KEY_TOGGLE);
  if (r != lastRead) {
    lastRead = r;
    lastChangeMs = millis();
  }

  if (millis() - lastChangeMs > 25) {
    if (lastStable != lastRead) {
      lastStable = lastRead;
      if (lastStable == LOW) {
        g_pumpManual = !g_pumpManual;
      }
    }
  }
}

/* ===================== 输出控制：报警优先，其次手动 ===================== */
static void outputsUpdate() {
  g_alarm = computeAlarm();

  // 报警时强制开泵+蜂鸣器响；不报警时蜂鸣器关，水泵按手动
  g_buzzerOut = g_alarm;
  g_pumpOut = g_alarm ? true : g_pumpManual;

  // 蜂鸣器：低电平响
  digitalWrite(PIN_BUZZER, g_buzzerOut ? LOW : HIGH);

  // 继电器输出
  if (RELAY_ACTIVE_HIGH) {
    digitalWrite(PIN_PUMP_RELAY, g_pumpOut ? HIGH : LOW);
  } else {
    digitalWrite(PIN_PUMP_RELAY, g_pumpOut ? LOW : HIGH);
  }
}

/* ===================== OLED 显示（本地显示，MQTT 不上传状态） ===================== */
static void oledRender() {
  g_oled.clearDisplay();
  g_oled.setTextSize(1);
  g_oled.setTextColor(SSD1306_WHITE);

  g_oled.setCursor(0, 0);
  g_oled.print("Name   V     Th");

  int y = 12;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    const auto& s = g_sensors[i];

    g_oled.setCursor(0, y);
    g_oled.print(s.name);

    g_oled.setCursor(40, y);
    g_oled.print(s.lastV, 2);

    g_oled.setCursor(80, y);
    g_oled.print(s.thresholdV, 2);

    if (s.lastV >= s.thresholdV) {
      g_oled.setCursor(115, y);
      g_oled.print("!");
    }
    y += 10;
  }

  // 仅本地显示水泵/蜂鸣器（不上传）
  g_oled.setCursor(0, 44);
  g_oled.print("Pump:");
  g_oled.print(g_pumpOut ? "ON " : "OFF");
  g_oled.print(" Buzz:");
  g_oled.print(g_buzzerOut ? "ON" : "OFF");

  g_oled.setCursor(0, 56);
  g_oled.print("Alarm:");
  g_oled.print(g_alarm ? "YES" : "NO ");
  g_oled.print(" Man:");
  g_oled.print(g_pumpManual ? "ON" : "OFF");

  g_oled.display();
}

/* ===================== MQTT：轻量 JSON 解析（固定字段） ===================== */
static bool jsonGetString(const char* json, const char* key, char* out, size_t outLen) {
  if (!json || !key || !out || outLen == 0) return false;
  out[0] = '\0';

  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char* p = strstr(json, pattern);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
  if (*p != '\"') return false;
  p++;
  size_t i = 0;
  while (*p && *p != '\"' && i + 1 < outLen) {
    out[i++] = *p++;
  }
  out[i] = '\0';
  return i > 0;
}

static bool jsonGetFloat(const char* json, const char* key, float* out) {
  if (!json || !key || !out) return false;

  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char* p = strstr(json, pattern);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;

  char* endp = nullptr;
  float v = strtof(p, &endp);
  if (endp == p) return false;
  *out = v;
  return true;
}

static bool jsonGetInt(const char* json, const char* key, int* out) {
  if (!json || !key || !out) return false;

  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char* p = strstr(json, pattern);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;

  char* endp = nullptr;
  long v = strtol(p, &endp, 10);
  if (endp == p) return false;
  *out = (int)v;
  return true;
}

/* ===================== MQTT：处理下发命令（阈值/水泵） ===================== */
static void mqttOnMessage(const char* topic, const char* payload) {
  (void)topic;
  if (!payload) return;

  char cmd[24];
  if (!jsonGetString(payload, "cmd", cmd, sizeof(cmd))) return;

  if (strcmp(cmd, "set_th") == 0) {
    float v;
    if (jsonGetFloat(payload, "mq7_th", &v))   g_sensors[0].thresholdV = v;
    if (jsonGetFloat(payload, "flame_th", &v)) g_sensors[1].thresholdV = v;

  } else if (strcmp(cmd, "pump") == 0) {
    int st;
    if (jsonGetInt(payload, "state", &st)) {
      g_pumpManual = (st != 0);
    } else {
      int tog;
      if (jsonGetInt(payload, "toggle", &tog) && tog != 0) {
        g_pumpManual = !g_pumpManual;
      }
    }
  }
}

/* ===================== MQTT：初始化 & 循环 ===================== */
static void mqttInit() {
  g_mqtt.onMessage(mqttOnMessage);
  g_mqtt.begin(
    WIFI_SSID,
    WIFI_PASS,
    MQTT_HOST,
    MQTT_PORT,
    MQTT_CLIENT_ID,
    MQTT_PUB_TOPIC,
    MQTT_SUB_TOPIC
  );
}

static void mqttLoop() {
  g_mqtt.loop();
}

/* ===================== MQTT：仅上报两个数据 + 两个阈值 ===================== */
static void mqttPublishTelemetryIfNeeded() {
  if (!g_mqtt.connected()) return;

  uint32_t now = millis();
  if (now - g_lastPubMs < PUB_INTERVAL_MS) return;
  g_lastPubMs = now;

  // 固定字段：mq7_v, flame_v, mq7_th, flame_th
  char buf[160];
  snprintf(
    buf, sizeof(buf),
    "{\"mq7_v\":%.3f,\"flame_v\":%.3f,\"mq7_th\":%.3f,\"flame_th\":%.3f}",
    g_sensors[0].lastV,
    g_sensors[1].lastV,
    g_sensors[0].thresholdV,
    g_sensors[1].thresholdV
  );

  g_mqtt.publish(buf, false);
}

/* ===================== 系统封装：init / loop ===================== */
static void systemInit() {
  Serial.begin(115200);

  sensorsInit();
  ioInit();
  mqttInit();

  if (!oledInit()) {
    Serial.println("[ERR] OLED init failed. Check addr/wiring.");
  }
}

static void systemLoop() {
  // 1) MQTT 保持连接 + 处理下发
  mqttLoop();

  // 2) 本地按键（翻转手动水泵）
  keyScanTogglePump();

  // 3) 采样 + 本地控制
  sensorsReadAll();
  outputsUpdate();

  // 4) OLED 显示
  oledRender();

  // 5) 上报（只发两路数据 + 两路阈值）
  mqttPublishTelemetryIfNeeded();

  delay(120);
}

/* ===================== main 保持整洁 ===================== */
void setup() {
  systemInit();
}

void loop() {
  systemLoop();
}
