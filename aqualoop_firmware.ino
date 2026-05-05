// ============================================================
//  AquaLoop ESP32 - SMART SIMULATION & HYBRID AUTOMATION (v6.0)
// ============================================================

#include <ArduinoJson.h>

// ─── PIN DEFINITIONS ────────────────────────────────────────
#define PIN_TDS_SENSOR   35   
#define PIN_LUX_SENSOR   32   
#define PIN_PUMP_6V      33   
#define PIN_PUMP_12V     26   
#define PIN_LED          27   

// --- RELAY SETTINGS (Active-Low Logic) ---
#define PUMP_ON    LOW
#define PUMP_OFF   HIGH
#define LED_ON     LOW
#define LED_OFF    HIGH

const int BAUD_RATE = 115200;
const unsigned long SEND_INTERVAL = 5000UL; 

// ─── PH SIMULATION PROFILES ────────────────────────────────
struct PhSimProfile {
  float startPH;
  float lowPH;
  float highPH;
  float rateDec; // units per sec
  float rateInc; // units per sec (net)
};

const PhSimProfile PH_SIM[3] = {
  // MAIN: 7.5 -> 6.5 (8 min), 6.5 -> 7.5 (5 min)
  { 7.5f, 6.5f, 7.5f, (7.5f-6.5f)/(8.0f*60.0f), (7.5f-6.5f)/(5.0f*60.0f) },
  // DB1: 6.9 -> 5.4 (8 min), 5.4 -> 6.8 (5 min)
  { 6.9f, 5.4f, 6.8f, (6.9f-5.4f)/(8.0f*60.0f), (6.8f-5.4f)/(5.0f*60.0f) },
  // DB2: 7.2 -> 5.9 (8 min), 5.9 -> 7.2 (5 min)
  { 7.2f, 5.9f, 7.2f, (7.2f-5.9f)/(8.0f*60.0f), (7.2f-5.9f)/(5.0f*60.0f) }
};

// ─── SYSTEM DEFINITIONS ─────────────────────────────────────
struct SystemSetpoints {
  const char *id;
  int tdsMin; int tdsMax;
  int luxMin; int luxMax;
};

const SystemSetpoints SYSTEMS[3] = {
  { "main", 20,  700, 13500, 16500 }, 
  { "db1",  700, 1200, 16000, 20000 },
  { "db2",  600, 1000, 14000, 18000 }
};

// ─── GLOBAL STATE ────────────────────────────────────────────
int  activeSystemIndex = 0;
bool identified        = false;
bool triggerCycle      = false;

struct ActuatorState {
  bool state;
  bool manual;
  bool silenced;
};

ActuatorState pump6v = {false, false, false};
ActuatorState pump12v = {false, false, false};
ActuatorState led = {false, false, false};

float  simPH[3];
enum SimPhase { DECREASING, INCREASING };
SimPhase currentPhase[3] = {DECREASING, DECREASING, DECREASING};

unsigned long lastPhUpdate;
unsigned long lastSendTime = 0;

// ─── SENSORS ────────────────────────────────────────────────
int readTDS() { return 450; } // Simulated for now
int readLux() { return 15000; } // Simulated for now

// ─── ACTUATORS ──────────────────────────────────────────────
void sendActuatorStatus(const char *act, bool on) {
  StaticJsonDocument<128> doc;
  doc["type"] = "actuator_status";
  doc["actuator"] = act;
  doc["state"] = on ? "on" : "off";
  serializeJson(doc, Serial); Serial.println();
}

void writeActuator(const char *name, ActuatorState &s, bool on, uint8_t pin) {
  if (s.state == on) return;
  s.state = on;
  digitalWrite(pin, on ? PUMP_ON : PUMP_OFF);
  sendActuatorStatus(name, on);
}

void runAutomation() {
  const SystemSetpoints &sys = SYSTEMS[activeSystemIndex];
  const PhSimProfile &phProf = PH_SIM[activeSystemIndex];
  float ph = simPH[activeSystemIndex];
  int tds = readTDS();
  int lux = readLux();

  // 1. Determine if Pumps are needed (pH too low OR TDS out of range)
  bool phTooLow = (ph < phProf.lowPH);
  bool tdsDev   = (tds < sys.tdsMin || tds > sys.tdsMax);
  bool needsPumps = phTooLow || tdsDev;

  // --- Pump 6V & 12V Automation ---
  auto applyPump = [&](const char* name, ActuatorState &s, uint8_t pin) {
    if (needsPumps) {
      // If we need pumps and they aren't manually silenced, turn them ON
      if (!s.silenced) writeActuator(name, s, true, pin);
    } else {
      // System is in equilibrium
      s.silenced = false; // Reset manual silence
      if (!s.manual) writeActuator(name, s, false, pin);
    }
  };

  applyPump("pump6v", pump6v, PIN_PUMP_6V);
  applyPump("pump12v", pump12v, PIN_PUMP_12V);

  // 2. Determine if LED is needed (Light too low)
  bool luxTooLow = (lux < sys.luxMin);
  
  if (luxTooLow) {
    if (!led.manual) writeActuator("led", led, true, PIN_LED);
  } else {
    if (!led.manual) writeActuator("led", led, false, PIN_LED);
  }
}

// ─── PH SIMULATION LOGIC ────────────────────────────────────
void updatePhSimulation() {
  unsigned long now = millis();
  float dtSec = (now - lastPhUpdate) / 1000.0f;
  lastPhUpdate = now;
  if (dtSec > 10.0f) dtSec = 0;

  for (int i = 0; i < 3; i++) {
    const PhSimProfile &p = PH_SIM[i];
    if (currentPhase[i] == DECREASING) {
      simPH[i] -= p.rateDec * dtSec;
      if (simPH[i] <= p.lowPH) {
        simPH[i] = p.lowPH;
        currentPhase[i] = INCREASING;
      }
    } else {
      // INCREASING
      bool pumpsOn = (i == activeSystemIndex) ? (pump6v.state || pump12v.state) : true;
      if (pumpsOn) simPH[i] += p.rateInc * dtSec;
      else simPH[i] -= p.rateDec * dtSec;

      if (simPH[i] >= p.highPH) {
        simPH[i] = p.highPH;
        currentPhase[i] = DECREASING;
      }
    }
  }
}

void handleIncoming(const String &input) {
  StaticJsonDocument<512> cmd;
  if (deserializeJson(cmd, input)) return;
  const char *type = cmd["type"] | "";

  if (strcmp(type, "identify_ack") == 0) {
    identified = true;
    sendActuatorStatus("pump6v", pump6v.state);
    sendActuatorStatus("pump12v", pump12v.state);
    sendActuatorStatus("led", led.state);
  } else if (strcmp(type, "set_profile") == 0) {
    const char* prof = cmd["profile"] | "main";
    for(int i=0; i<3; i++) {
      if(strcmp(SYSTEMS[i].id, prof) == 0) {
        activeSystemIndex = i;
        triggerCycle = true;
        break;
      }
    }
  } else if (strcmp(type, "actuator_command") == 0) {
    const char *act = cmd["actuator"] | "";
    const char *state = cmd["state"] | "off";
    const char *mode = cmd["mode"] | "auto";
    bool on = (strcmp(state, "on") == 0);
    bool isMan = (strcmp(mode, "manual") == 0);
    bool isDev = (currentPhase[activeSystemIndex] == INCREASING);

    if (strcmp(act, "pump6v") == 0) {
      pump6v.manual = isMan;
      if (isDev && !on) pump6v.silenced = true;
      else if (on) pump6v.silenced = false;
      writeActuator("pump6v", pump6v, on, PIN_PUMP_6V);
    } else if (strcmp(act, "pump12v") == 0) {
      pump12v.manual = isMan;
      if (isDev && !on) pump12v.silenced = true;
      else if (on) pump12v.silenced = false;
      writeActuator("pump12v", pump12v, on, PIN_PUMP_12V);
    } else if (strcmp(act, "led") == 0) {
      led.manual = isMan;
      writeActuator("led", led, on, PIN_LED);
    }
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.setRxBufferSize(2048); // Ensure we don't miss incoming commands
  pinMode(PIN_PUMP_6V, OUTPUT); pinMode(PIN_PUMP_12V, OUTPUT); pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_PUMP_6V, PUMP_OFF); digitalWrite(PIN_PUMP_12V, PUMP_OFF); digitalWrite(PIN_LED, LED_OFF);

  for(int i=0; i<3; i++) { simPH[i] = PH_SIM[i].startPH; }
  lastPhUpdate = millis();
  delay(2000); // Give server time to connect
  Serial.print("{\"type\":\"identify\",\"device\":\"AquaLoop_ESP32\"}\r\n");
}

void loop() {
  updatePhSimulation();
  while (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) handleIncoming(input);
  }
  if (millis() - lastSendTime >= SEND_INTERVAL || triggerCycle) {
    lastSendTime = millis(); triggerCycle = false;
    StaticJsonDocument<512> doc;
    doc["type"] = "sensor_reading";
    doc["profile"] = SYSTEMS[activeSystemIndex].id;
    doc["ph"] = serialized(String(simPH[activeSystemIndex], 2));
    doc["tds"] = readTDS();
    doc["light"] = readLux();
    doc["phase"] = (currentPhase[activeSystemIndex] == INCREASING) ? "correction" : "drift";
    serializeJson(doc, Serial);
    Serial.print("\r\n"); // Explicitly send the delimiter the server expects
    runAutomation();
  }
}
