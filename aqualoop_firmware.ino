/*
 * ============================================================
 *  AquaLoop — ESP32 Firmware v3.0 (Real Sensors)
 * ============================================================
 */

#include <ArduinoJson.h>
#include <BH1750.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <WebSocketsClient.h>
#include <WiFi.h>
#include <Wire.h>

const char *WIFI_SSID = "negm";
const char *WIFI_PASSWORD = "abdonegm";
const char *SERVER_HOST =
    "browsers-naturals-subsidiary-areas.trycloudflare.com";
                                             // (without https://)
const int SERVER_PORT = 443;
const char *SERVER_PATH = "/ws/sensors";

#define PH_PIN 34
#define TDS_PIN 35
#define TEMP_PIN 4
#define PUMP_1_PIN 33
#define PUMP_2_PIN 26
#define DILUTED_PUMP_PIN 25
#define LED_PIN 27
#define RELAY_ON HIGH
#define RELAY_OFF LOW

#define ADC_DISCONNECTED_LOW 50
#define ADC_DISCONNECTED_HIGH 4040
#define ADC_SAMPLES 30
#define ADC_SAMPLE_DELAY_MS 5
#define SENSOR_INTERVAL_MS 5000
#define TEMP_REQUEST_DELAY_MS 750
#define WIFI_TIMEOUT_MS 30000

float PH_VOLTAGE_AT_4 = 3.00;
float PH_VOLTAGE_AT_7 = 2.50;
float TDS_CALIBRATION_FACTOR = 0.5;

BH1750 lightMeter;
OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensor(&oneWire);
WebSocketsClient webSocket;
bool wsConnected = false;

struct Actuator {
  int pin;
  bool state;
  bool manual;
};

Actuator pump1 = {PUMP_1_PIN, false, false};
Actuator pump2 = {PUMP_2_PIN, false, false};
Actuator dilutedPump = {DILUTED_PUMP_PIN, false, false};
Actuator led = {LED_PIN, false, false};

struct Profile {
  float ph_min, ph_max;
  float tds_min, tds_max;
  float lux_min, lux_max;
};

Profile profiles[3] = {
    {7.0, 7.5, 300, 700, 13500, 16500},
    {6.0, 7.0, 700, 1200, 16000, 20000},
    {6.5, 7.5, 600, 1000, 14000, 18000},
};
const char *profileNames[3] = {"main", "db1", "db2"};
int activeIndex = 0;
Profile *current = &profiles[0];

// ── WiFi ──────────────────────────────────────────────────────
void printWiFiStatus(wl_status_t s) {
  switch (s) {
  case WL_NO_SSID_AVAIL:
    Serial.println("[WiFi] ERROR: SSID not found");
    break;
  case WL_CONNECT_FAILED:
    Serial.println("[WiFi] ERROR: Wrong password");
    break;
  case WL_CONNECTION_LOST:
    Serial.println("[WiFi] ERROR: Connection lost");
    break;
  case WL_DISCONNECTED:
    Serial.println("[WiFi] ERROR: Disconnected");
    break;
  default:
    Serial.printf("[WiFi] ERROR: Status code %d\n", s);
    break;
  }
}

bool connectWiFi() {
  Serial.println("\n[WiFi] --- Connection Attempt ---");
  Serial.print("[WiFi] SSID: ");
  Serial.println(WIFI_SSID);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);
  WiFi.setHostname("AquaLoop-ESP32");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start >= WIFI_TIMEOUT_MS) {
      Serial.println("\n[WiFi] FAILED: Timeout reached (30s)");
      printWiFiStatus(WiFi.status());
      return false;
    }
    delay(500);
    Serial.print(".");
    if ((millis() - start) % 5000 == 0) {
      Serial.printf(" [%ds] ", (int)((millis() - start) / 1000));
    }
  }

  Serial.println("\n[WiFi] SUCCESS!");
  Serial.print("[WiFi] IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("[WiFi] RSSI: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  return true;
}

// ── Actuators ─────────────────────────────────────────────────
void applyActuator(Actuator &a) {
  digitalWrite(a.pin, a.state ? RELAY_ON : RELAY_OFF);
}

Actuator *getActuatorByName(const char *name) {
  if (strcmp(name, "pump1") == 0)
    return &pump1;
  if (strcmp(name, "pump2") == 0)
    return &pump2;
  if (strcmp(name, "diluted_pump") == 0)
    return &dilutedPump;
  if (strcmp(name, "led") == 0)
    return &led;
  return nullptr;
}

// ── Sensors ───────────────────────────────────────────────────
float readAnalogAveraged(int pin) {
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(pin);
    delay(ADC_SAMPLE_DELAY_MS);
  }
  float avg = sum / (float)ADC_SAMPLES;
  if (avg < ADC_DISCONNECTED_LOW || avg > ADC_DISCONNECTED_HIGH)
    return -1.0;
  return avg;
}

float readPH() {
  float raw = readAnalogAveraged(PH_PIN);
  if (raw < 0)
    return -1.0;
  float voltage = raw * (3.3 / 4095.0);
  float slope = (7.0 - 4.0) / (PH_VOLTAGE_AT_7 - PH_VOLTAGE_AT_4);
  return constrain(7.0 + slope * (voltage - PH_VOLTAGE_AT_7), 0.0, 14.0);
}

float readTDS() {
  float raw = readAnalogAveraged(TDS_PIN);
  if (raw < 0)
    return -1.0;
  float voltage = raw * (3.3 / 4095.0);
  float tds =
      (133.42 * pow(voltage, 3) - 255.86 * pow(voltage, 2) + 857.39 * voltage) *
      TDS_CALIBRATION_FACTOR;
  return constrain(tds, 0.0, 2000.0);
}

float readLight() {
  float lux = lightMeter.readLightLevel();
  if (lux < 0)
    return -1.0;
  return lux;
}

float readTemperature() {
  tempSensor.requestTemperatures();
  delay(TEMP_REQUEST_DELAY_MS);
  float t = tempSensor.getTempCByIndex(0);
  if (t <= -100.0)
    return -1.0;
  return t;
}

// ── WebSocket sends ───────────────────────────────────────────
void sendActuatorStatus() {
  StaticJsonDocument<512> doc;
  doc["type"] = "actuator_status";
  doc["actuators"]["pump1"]["state"] = pump1.state ? "on" : "off";
  doc["actuators"]["pump1"]["mode"] = pump1.manual ? "manual" : "auto";
  doc["actuators"]["pump2"]["state"] = pump2.state ? "on" : "off";
  doc["actuators"]["pump2"]["mode"] = pump2.manual ? "manual" : "auto";
  doc["actuators"]["diluted_pump"]["state"] = dilutedPump.state ? "on" : "off";
  doc["actuators"]["diluted_pump"]["mode"] =
      dilutedPump.manual ? "manual" : "auto";
  doc["actuators"]["led"]["state"] = led.state ? "on" : "off";
  doc["actuators"]["led"]["mode"] = led.manual ? "manual" : "auto";
  String output;
  serializeJson(doc, output);
  webSocket.sendTXT(output);
}

void sendSensorReading(float ph, float tds, float light, float temp) {
  DynamicJsonDocument doc(1024);
  doc["type"] = "sensor_reading";
  doc["system"] = activeIndex;
  doc["profile"] = profileNames[activeIndex];

  if (ph < 0)
    doc["ph"] = nullptr;
  else
    doc["ph"] = ph;
  if (tds < 0)
    doc["tds"] = nullptr;
  else
    doc["tds"] = tds;
  if (light < 0)
    doc["light"] = nullptr;
  else
    doc["light"] = light;
  if (temp < 0)
    doc["temp"] = nullptr;
  else
    doc["temp"] = temp;

  doc["status"]["ph_ok"] =
      (ph >= 0) && (ph >= current->ph_min) && (ph <= current->ph_max);
  doc["status"]["tds_ok"] =
      (tds >= 0) && (tds >= current->tds_min) && (tds <= current->tds_max);
  doc["status"]["light_ok"] = (light >= 0) && (light >= current->lux_min) &&
                              (light <= current->lux_max);
  doc["status"]["temp_ok"] = (temp >= 0);

  doc["sensors"]["ph_connected"] = (ph >= 0);
  doc["sensors"]["tds_connected"] = (tds >= 0);
  doc["sensors"]["light_connected"] = (light >= 0);
  doc["sensors"]["temp_connected"] = (temp >= 0);

  doc["actuators"]["pump1"]["state"] = pump1.state ? "on" : "off";
  doc["actuators"]["pump1"]["mode"] = pump1.manual ? "manual" : "auto";
  doc["actuators"]["pump2"]["state"] = pump2.state ? "on" : "off";
  doc["actuators"]["pump2"]["mode"] = pump2.manual ? "manual" : "auto";
  doc["actuators"]["diluted_pump"]["state"] = dilutedPump.state ? "on" : "off";
  doc["actuators"]["diluted_pump"]["mode"] =
      dilutedPump.manual ? "manual" : "auto";
  doc["actuators"]["led"]["state"] = led.state ? "on" : "off";
  doc["actuators"]["led"]["mode"] = led.manual ? "manual" : "auto";

  String output;
  serializeJson(doc, output);
  webSocket.sendTXT(output);
  Serial.println("[SEND] " + output);
}

// ── Automation ────────────────────────────────────────────────
void runAutomation(float ph, float tds, float light) {
  if (!pump1.manual) {
    pump1.state = true;
    applyActuator(pump1);
  }
  if (!pump2.manual) {
    pump2.state = true;
    applyActuator(pump2);
  }

  if (!dilutedPump.manual) {
    bool phOOR = (ph >= 0) && (ph < current->ph_min || ph > current->ph_max);
    bool tdsOOR =
        (tds >= 0) && (tds < current->tds_min || tds > current->tds_max);
    bool fire = ((ph >= 0) || (tds >= 0)) && (phOOR || tdsOOR);
    dilutedPump.state = fire;
    applyActuator(dilutedPump);
    if (fire) {
      Serial.print("[AUTO] Diluted pump ON — ");
      if (phOOR) {
        Serial.print("pH=");
        Serial.print(ph);
        Serial.print(" ");
      }
      if (tdsOOR) {
        Serial.print("TDS=");
        Serial.print(tds);
      }
      Serial.println();
    }
  }

  if (!led.manual && light >= 0) {
    if (light < current->lux_min) {
      led.state = true;
      Serial.print("[AUTO] LED ON  lux=");
      Serial.println(light);
    } else if (light > current->lux_max) {
      led.state = false;
      Serial.print("[AUTO] LED OFF lux=");
      Serial.println(light);
    }
    applyActuator(led);
  }
}

// ── WebSocket event handler ───────────────────────────────────
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
  case WStype_CONNECTED:
    wsConnected = true;
    Serial.println("[WS] Connected to backend");
    webSocket.sendTXT("{\"type\":\"identify\",\"device\":\"esp32-aqualoop\","
                      "\"firmware\":\"3.0\"}");
    sendActuatorStatus();
    break;

  case WStype_DISCONNECTED:
    wsConnected = false;
    Serial.println("[WS] Disconnected from backend");
    break;

  case WStype_TEXT: {
    StaticJsonDocument<512> cmd;
    if (deserializeJson(cmd, payload, length))
      break;
    const char *msgType = cmd["type"];
    if (!msgType)
      break;

    if (strcmp(msgType, "actuator_command") == 0) {
      const char *id = cmd["actuator"];
      const char *state = cmd["state"];
      const char *mode = cmd["mode"];
      if (!id || !state)
        break;
      Actuator *a = getActuatorByName(id);
      if (!a)
        break;
      a->state = (strcmp(state, "on") == 0);
      a->manual = (mode && strcmp(mode, "manual") == 0);
      applyActuator(*a);
      Serial.printf("[CMD] %s → %s %s\n", id, a->state ? "ON" : "OFF",
                    a->manual ? "MANUAL" : "AUTO");
      sendActuatorStatus();
      break;
    }

    if (strcmp(msgType, "set_profile") == 0) {
      const char *p = cmd["profile"];
      if (!p)
        break;
      for (int i = 0; i < 3; i++) {
        if (strcmp(p, profileNames[i]) == 0) {
          activeIndex = i;
          current = &profiles[i];
          Serial.print("[PROFILE] → ");
          Serial.println(profileNames[i]);
          StaticJsonDocument<128> ack;
          ack["type"] = "profile_switched";
          ack["profile"] = profileNames[i];
          ack["system"] = i;
          String s;
          serializeJson(ack, s);
          webSocket.sendTXT(s);
          break;
        }
      }
      break;
    }

    if (strcmp(msgType, "set_setpoints") == 0) {
      if (cmd.containsKey("ph_min"))
        current->ph_min = cmd["ph_min"];
      if (cmd.containsKey("ph_max"))
        current->ph_max = cmd["ph_max"];
      if (cmd.containsKey("tds_min"))
        current->tds_min = cmd["tds_min"];
      if (cmd.containsKey("tds_max"))
        current->tds_max = cmd["tds_max"];
      if (cmd.containsKey("lux_min"))
        current->lux_min = cmd["lux_min"];
      if (cmd.containsKey("lux_max"))
        current->lux_max = cmd["lux_max"];
      Serial.printf("[SETPOINTS] pH %.2f–%.2f TDS %.0f–%.0f Lux %.0f–%.0f\n",
                    current->ph_min, current->ph_max, current->tds_min,
                    current->tds_max, current->lux_min, current->lux_max);
      StaticJsonDocument<256> ack;
      ack["type"] = "setpoints_updated";
      ack["profile"] = profileNames[activeIndex];
      ack["ph_min"] = current->ph_min;
      ack["ph_max"] = current->ph_max;
      ack["tds_min"] = current->tds_min;
      ack["tds_max"] = current->tds_max;
      ack["lux_min"] = current->lux_min;
      ack["lux_max"] = current->lux_max;
      String s;
      serializeJson(ack, s);
      webSocket.sendTXT(s);
      break;
    }

    if (strcmp(msgType, "set_calibration") == 0) {
      if (cmd.containsKey("ph_v4"))
        PH_VOLTAGE_AT_4 = cmd["ph_v4"];
      if (cmd.containsKey("ph_v7"))
        PH_VOLTAGE_AT_7 = cmd["ph_v7"];
      if (cmd.containsKey("tds_factor"))
        TDS_CALIBRATION_FACTOR = cmd["tds_factor"];
      Serial.printf("[CAL] pH V@4=%.3f V@7=%.3f TDS=%.3f\n", PH_VOLTAGE_AT_4,
                    PH_VOLTAGE_AT_7, TDS_CALIBRATION_FACTOR);
      StaticJsonDocument<128> ack;
      ack["type"] = "calibration_updated";
      ack["ph_v4"] = PH_VOLTAGE_AT_4;
      ack["ph_v7"] = PH_VOLTAGE_AT_7;
      ack["tds_factor"] = TDS_CALIBRATION_FACTOR;
      String s;
      serializeJson(ack, s);
      webSocket.sendTXT(s);
      break;
    }

    if (strcmp(msgType, "ping") == 0) {
      webSocket.sendTXT("{\"type\":\"pong\"}");
      break;
    }
    break;
  }
  default:
    break;
  }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n==============================");
  Serial.println(" AquaLoop Firmware v3.0");
  Serial.println(" Real Sensor Mode");
  Serial.println("==============================");

  pinMode(PUMP_1_PIN, OUTPUT);
  digitalWrite(PUMP_1_PIN, RELAY_OFF);
  pinMode(PUMP_2_PIN, OUTPUT);
  digitalWrite(PUMP_2_PIN, RELAY_OFF);
  pinMode(DILUTED_PUMP_PIN, OUTPUT);
  digitalWrite(DILUTED_PUMP_PIN, RELAY_OFF);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, RELAY_OFF);
  Serial.println("[INIT] Relays OFF");

  Wire.begin(21, 22);
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
    Serial.println("[INIT] BH1750 OK");
  else
    Serial.println("[INIT] BH1750 FAILED — check SDA/SCL wiring");

  tempSensor.begin();
  if (tempSensor.getDeviceCount() > 0) {
    tempSensor.setResolution(9);
    Serial.println("[INIT] DS18B20 OK");
  } else {
    Serial.println("[INIT] DS18B20 NOT FOUND — check GPIO4 + 4.7k pullup");
  }

  while (!connectWiFi()) {
    Serial.println("[WiFi] Retrying in 5 seconds...");
    delay(5000);
  }

  webSocket.beginSSL(SERVER_HOST, SERVER_PORT,
                     SERVER_PATH); // Use beginSSL for Cloudflare internet URLs
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);
  webSocket.enableHeartbeat(25000, 3000, 2);
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  webSocket.loop();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Lost connection — reconnecting...");
    wsConnected = false;
    while (!connectWiFi()) {
      Serial.println("[WiFi] Retrying in 5 seconds...");
      delay(5000);
    }
    webSocket.beginSSL(
        SERVER_HOST, SERVER_PORT,
        SERVER_PATH); // Use beginSSL for Cloudflare internet URLs
  }

  static unsigned long lastSensor = 0;
  if (millis() - lastSensor >= SENSOR_INTERVAL_MS && wsConnected) {
    lastSensor = millis();
    float ph = readPH();
    float tds = readTDS();
    float light = readLight();
    float temp = readTemperature();

    Serial.println("────────────────────────");
    Serial.print("[pH]    ");
    ph < 0 ? Serial.println("DISCONNECTED")
           : (Serial.print(ph, 2), Serial.println(" pH"));
    Serial.print("[TDS]   ");
    tds < 0 ? Serial.println("DISCONNECTED")
            : (Serial.print(tds, 1), Serial.println(" ppm"));
    Serial.print("[Lux]   ");
    light < 0 ? Serial.println("DISCONNECTED")
              : (Serial.print(light, 1), Serial.println(" lux"));
    Serial.print("[Temp]  ");
    temp < 0 ? Serial.println("DISCONNECTED")
             : (Serial.print(temp, 2), Serial.println(" °C"));
    Serial.print("[Profile] ");
    Serial.println(profileNames[activeIndex]);

    runAutomation(ph, tds, light);
    sendSensorReading(ph, tds, light, temp);
  }
}
