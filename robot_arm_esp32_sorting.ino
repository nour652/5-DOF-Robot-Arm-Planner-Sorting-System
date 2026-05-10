// ================================================================
// ESP32 + PCA9685 + WiFi Web Server — 5-DOF Arm Planner
// WITH LJ12A3-4-Z/BX Inductive + HC-SR04 Ultrasonic
// SORTING LOGIC: Ultrasonic detects object → Inductive checks material
//   US + IND (metal)    → Metal path
//   US only (non-metal) → Non-metal path
//   IND only            → No action (waiting for object)
// Auto-generated. Open Serial Monitor @ 115200 for IP.
// ================================================================
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const char* ssid     = "ssid";
const char* password = "password";
WebServer server(80);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ── SENSOR CONFIG ──────────────────────────────────────────────
#define INDUCTIVE_PIN      34     // GPIO34 for inductive sensor (NPN NO)
#define ULTRASONIC_TRIG    18     // GPIO18 - Trigger
#define ULTRASONIC_ECHO    19     // GPIO19 - Echo
#define ULTRASONIC_MIN_CM  17      // Minimum detection distance (cm)
#define ULTRASONIC_MAX_CM  19    // Maximum detection distance (cm)

// ── SERVO CONFIG ───────────────────────────────────────────────
#define SERVOMIN  102
#define SERVOMAX  512
#define SERVO_FREQ 50
#define NUM_JOINTS 6
#define MAX_STEPS  100

bool loopPath = false;

const uint8_t CH[NUM_JOINTS] = {0, 1, 2, 3, 4, 5};
const bool REV[NUM_JOINTS] = {false, true, true, false, false, false};
const float ZERO[NUM_JOINTS] = {90, 90, 90, 90, 90, 90};
const float MIN_ANG[NUM_JOINTS] = {0, 30, 15, 20, 0, 0};
const float MAX_ANG[NUM_JOINTS] = {180, 160, 165, 160, 180, 180};
const float TRIM[NUM_JOINTS] = {0, 0, 0, 0, 0, 0};
const float START_DEG[NUM_JOINTS] = {0, 60, -120, 60, 0, 0};

// ── PATH DATA ──────────────────────────────────────────────────
int pathLength = 0;
float waypoints[MAX_STEPS][NUM_JOINTS];
uint32_t segDur[MAX_STEPS];
int curPulse[NUM_JOINTS];
volatile bool pathRun = false;
volatile bool pathStop = false;
String pathMsg = "Ready";

// ── SENSOR AUTOMATION ──────────────────────────────────────────
bool sensorAutoEnabled = false;
bool sensorAutoRunning = false;

int sensorPathMetalLength = 0;
float sensorPathMetal[MAX_STEPS][NUM_JOINTS];
uint32_t sensorPathMetalDur[MAX_STEPS];

int sensorPathNoMetalLength = 0;
float sensorPathNoMetal[MAX_STEPS][NUM_JOINTS];
uint32_t sensorPathNoMetalDur[MAX_STEPS];

// ── SENSOR STATE ───────────────────────────────────────────────
float lastDistanceCm = 999;
bool lastUltrasonic = false;
bool lastInductive = false;
bool lastBothTriggered = false;

uint16_t angleToPulse(int j, float k_deg) {
  float phys = k_deg;
  if (REV[j]) phys = -phys;
  phys += ZERO[j] + TRIM[j];
  phys = constrain(phys, MIN_ANG[j], MAX_ANG[j]);
  return (uint16_t)map((long)phys, 0, 180, SERVOMIN, SERVOMAX);
}

void moveJoint(int j, float deg, int ms) {
  int target = angleToPulse(j, deg);
  int start = curPulse[j];
  if (target == start) return;
  int steps = abs(target - start);
  int delayUs = max(1, (ms * 1000) / steps);
  if (start < target)
    for (int p = start; p <= target; p++) { pwm.setPWM(CH[j], 0, p); delayMicroseconds(delayUs); }
  else
    for (int p = start; p >= target; p--) { pwm.setPWM(CH[j], 0, p); delayMicroseconds(delayUs); }
  curPulse[j] = target;
}

void moveAll(float degs[NUM_JOINTS], uint32_t ms) {
  int targets[NUM_JOINTS], starts[NUM_JOINTS], maxSteps = 0;
  for (int j = 0; j < NUM_JOINTS; j++) {
    starts[j] = curPulse[j];
    targets[j] = angleToPulse(j, degs[j]);
    maxSteps = max(maxSteps, abs(targets[j] - starts[j]));
  }
  if (maxSteps == 0) return;
  int delayUs = max(1, (int)((ms * 1000UL) / maxSteps));
  for (int s = 0; s <= maxSteps; s++) {
    float t = (float)s / maxSteps;
    float ts = t * t * (3.0f - 2.0f * t);
    for (int j = 0; j < NUM_JOINTS; j++)
      pwm.setPWM(CH[j], 0, starts[j] + (int)((targets[j]-starts[j]) * ts));
    delayMicroseconds(delayUs);
  }
  for (int j = 0; j < NUM_JOINTS; j++) curPulse[j] = targets[j];
}

void initJoints() {
  for (int j = 0; j < NUM_JOINTS; j++) {
    int target = angleToPulse(j, START_DEG[j]);
    int steps = abs(target - SERVOMIN);
    int dUs = max(1, (900 * 1000) / max(steps, 1));
    for (int p = SERVOMIN; p <= target; p++) { pwm.setPWM(CH[j], 0, p); delayMicroseconds(dUs); }
    curPulse[j] = target;
    delay(250);
  }
}

float readUltrasonicCm() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
  if (duration == 0) return 999;
  return (duration * 0.0343) / 2.0;
}

bool readInductive() {
  return digitalRead(INDUCTIVE_PIN) == LOW;
}

// ── SORTING SENSOR HANDLER ───────────────────────────────────
// Ultrasonic is the primary detector (any object).
// Inductive decides material:
//   US + IND  → METAL path
//   US only   → NON-METAL path
//   IND only  → No action (no object confirmed)
void handleSensorAutomation() {
  // Always read sensors so the web UI gets live status
  float dist = readUltrasonicCm();
  lastDistanceCm = dist;
  bool ultrasonicNow = (dist >= ULTRASONIC_MIN_CM && dist <= ULTRASONIC_MAX_CM);
  bool inductiveNow = readInductive();

  lastInductive = inductiveNow;

  if (!sensorAutoEnabled || sensorAutoRunning) {
    lastUltrasonic = ultrasonicNow;
    lastBothTriggered = ultrasonicNow && inductiveNow;
    return;
  }

  // Detect RISING EDGE on ultrasonic (object just arrived)
  bool ultrasonicRising = ultrasonicNow && !lastUltrasonic;
  lastUltrasonic = ultrasonicNow;
  lastBothTriggered = ultrasonicNow && inductiveNow;

  if (!ultrasonicRising) return;

  // Object detected by ultrasonic – decide material
  if (inductiveNow) {
    // ── METAL OBJECT ──
    Serial.print("[SENSOR] Object detected! Distance: ");
    Serial.print(dist);
    Serial.println(" cm. Material: METAL");

    if (sensorPathMetalLength >= 2) {
      sensorAutoRunning = true;
      pathMsg = "SENSOR: Metal object → running metal path...";
      Serial.println("[SENSOR] >>> Running METAL path <<<");

      for (int i = 0; i < sensorPathMetalLength - 1; i++) {
        if (!sensorAutoEnabled) break;
        pathMsg = "Metal step " + String(i+1) + "/" + String(sensorPathMetalLength-1);
        moveAll(sensorPathMetal[i+1], sensorPathMetalDur[i]);
        server.handleClient();
      }

      sensorAutoRunning = false;
      if (sensorAutoEnabled) {
        pathMsg = "SENSOR: Metal path done. Waiting for next object...";
        Serial.println("[SENSOR] Metal path complete.");
      }
    } else {
      pathMsg = "SENSOR: Metal detected but metal path not configured!";
      Serial.println("[SENSOR] Metal detected but path has <2 waypoints!");
    }

  } else {
    // ── NON-METAL OBJECT ──
    Serial.print("[SENSOR] Object detected! Distance: ");
    Serial.print(dist);
    Serial.println(" cm. Material: NON-METAL");

    if (sensorPathNoMetalLength >= 2) {
      sensorAutoRunning = true;
      pathMsg = "SENSOR: Non-metal object → running non-metal path...";
      Serial.println("[SENSOR] >>> Running NON-METAL path <<<");

      for (int i = 0; i < sensorPathNoMetalLength - 1; i++) {
        if (!sensorAutoEnabled) break;
        pathMsg = "Non-metal step " + String(i+1) + "/" + String(sensorPathNoMetalLength-1);
        moveAll(sensorPathNoMetal[i+1], sensorPathNoMetalDur[i]);
        server.handleClient();
      }

      sensorAutoRunning = false;
      if (sensorAutoEnabled) {
        pathMsg = "SENSOR: Non-metal path done. Waiting for next object...";
        Serial.println("[SENSOR] Non-metal path complete.");
      }
    } else {
      pathMsg = "SENSOR: Non-metal detected but non-metal path not configured!";
      Serial.println("[SENSOR] Non-metal detected but path has <2 waypoints!");
    }
  }
}

// ── HTML PAGE ──────────────────────────────────────────────────
const char PAGE[] PROGMEM = "<!DOCTYPE html><html><head><meta name=viewport content=width=device-width,initial-scale=1><title>Robot Arm</title><style>body{font-family:Arial;background:#0d1117;color:#e6edf3;padding:12px;margin:0}h2{color:#58a6ff;text-align:center;margin-bottom:14px}.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:12px;margin-bottom:10px}.row{display:flex;align-items:center;gap:8px;margin-bottom:6px}.row label{font-size:.8em;color:#8b949e;min-width:100px}input[type=range]{flex:1;accent-color:#58a6ff}.v{font-weight:bold;color:#58a6ff;min-width:36px;font-size:.85em}.btn{width:100%;padding:9px;border:none;border-radius:6px;font-weight:bold;cursor:pointer;margin-top:5px}.gr{background:#238636;color:#fff}.rd{background:#da3633;color:#fff}.gy{background:#21262d;color:#8b949e;border:1px solid #30363d}.bl{background:#1f6feb;color:#fff}.yl{background:#d29922;color:#fff}.st{text-align:center;font-size:.8em;color:#8b949e;margin-top:6px}input[type=number]{background:#0d1117;border:1px solid #30363d;color:#e6edf3;border-radius:4px;padding:4px 6px;width:70px}.sensor-box{font-size:1.1em;text-align:center;padding:10px;border-radius:6px;margin-bottom:10px;font-weight:bold}.sensor-both{background:#23863633;border:1px solid #238636;color:#3fb950}.sensor-wait{background:#1f6feb33;border:1px solid #1f6feb;color:#58a6ff}.sensor-one{background:#d2992233;border:1px solid #d29922;color:#d29922}.sensor-off{background:#21262d33;border:1px solid #30363d;color:#8b949e}</style></head><body><h2>&#129470; Robot Arm</h2><div class=card><b style=font-size:.85em>Move time (ms):</b> <input type=number id=ms value=1000 min=100 step=100></div><div class=card><div class=row><label>J1 Base Yaw</label><input type=range min=-90 max=90 value=0.0 id=s0 oninput=\"mv(0,this.value)\"><span class=v id=v0>0.0</span></div><div class=row><label>J2 Shoulder</label><input type=range min=-70 max=60 value=60.0 id=s1 oninput=\"mv(1,this.value)\"><span class=v id=v1>60.0</span></div><div class=row><label>J3 Elbow</label><input type=range min=-75 max=75 value=-120.0 id=s2 oninput=\"mv(2,this.value)\"><span class=v id=v2>-120.0</span></div><div class=row><label>J4 Wrist Pi</label><input type=range min=-70 max=70 value=60.0 id=s3 oninput=\"mv(3,this.value)\"><span class=v id=v3>60.0</span></div><div class=row><label>J5 Wrist Ro</label><input type=range min=-90 max=90 value=0.0 id=s4 oninput=\"mv(4,this.value)\"><span class=v id=v4>0.0</span></div><div class=row><label>J6 Gripper</label><input type=range min=-90 max=90 value=0.0 id=s5 oninput=\"mv(5,this.value)\"><span class=v id=v5>0.0</span></div><button class=\"btn gy\" onclick=\"rst()\">Reset (Home)</button></div><div class=card><b style=font-size:.85em>Path Control</b><button class=\"btn gr\" onclick=\"run()\">&#9654; Run Path</button><button class=\"btn rd\" onclick=\"stp()\" style=margin-top:6px>&#9209; Stop</button><div class=st id=st>Ready</div></div><div class=card><b style=font-size:.85em>Sensor Automation (Sorting)</b><div id=sensorBox class=\"sensor-box sensor-off\">&#9203; Auto OFF</div><div style=font-size:.75em;color:#8b949e;text-align:center;margin-bottom:8px>US: <span id=us>--</span> cm | IND: <span id=ind>--</span> | BOTH: <span id=both>--</span></div><button class=\"btn bl\" onclick=\"sensorStart()\">&#9654; Start Sensor Auto</button><button class=\"btn rd\" onclick=\"sensorStop()\" style=margin-top:6px>&#9209; Stop Sensor Auto</button><div class=st id=sensorSt>Sensor Auto: OFF</div></div><script>function mv(n,v){document.getElementById('v'+n).innerText=v;fetch('/set?servo='+n+'&angle='+v+'&time='+document.getElementById('ms').value);}function rst(){fetch('/reset');const st=[0.0,60.0,-120.0,60.0,0.0,0.0];for(let i=0;i<6;i++){var s=document.getElementById('s'+i);if(s){s.value=st[i];document.getElementById('v'+i).innerText=st[i];}}}function run(){fetch('/run');document.getElementById('st').innerText='Running...';}function stp(){fetch('/stop');document.getElementById('st').innerText='Stopped.';}function sensorStart(){fetch('/sensor_start');document.getElementById('sensorSt').innerText='Sensor Auto: RUNNING';}function sensorStop(){fetch('/sensor_stop');document.getElementById('sensorSt').innerText='Sensor Auto: STOPPED';}setInterval(()=>fetch('/status').then(r=>r.text()).then(t=>{if(t)document.getElementById('st').innerText=t;}),800);setInterval(()=>fetch('/sensor_full_status').then(r=>r.json()).then(d=>{const el=document.getElementById('sensorBox');document.getElementById('us').innerText=d.dist.toFixed(1);document.getElementById('ind').innerText=d.ind==1?'METAL':'none';document.getElementById('both').innerText=d.both==1?'YES':'NO';if(d.auto==0){el.className='sensor-box sensor-off';el.innerHTML='&#9203; Auto OFF';}else if(d.us==1&&d.ind==1){el.className='sensor-box sensor-both';el.innerHTML='&#9989; METAL OBJECT! Running metal...';}else if(d.us==1&&d.ind==0){el.className='sensor-box sensor-one';el.innerHTML='&#9898; NON-METAL OBJECT! Running non-metal...';}else if(d.us==0&&d.ind==1){el.className='sensor-box sensor-one';el.innerHTML='&#9888; Inductive only (no object)';}else{el.className='sensor-box sensor-wait';el.innerHTML='&#9203; Waiting for object...';}}),400);<\/script></body></html>";

void sendCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void setup() {
  Serial.begin(115200);
  pinMode(INDUCTIVE_PIN, INPUT_PULLUP);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  Wire.begin(21, 22);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  initJoints();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\n========================================");
  Serial.println("WiFi Connected! IP: " + WiFi.localIP().toString());
  Serial.println("----------------------------------------");
  Serial.println("SENSOR SORTING SYSTEM:");
  Serial.println("  Ultrasonic detects object → Inductive checks material");
  Serial.println("  US + IND = METAL path | US only = NON-METAL path");
  Serial.println("  Ultrasonic: GPIO" + String(ULTRASONIC_TRIG) + "/" + String(ULTRASONIC_ECHO));
  Serial.println("  Inductive:  GPIO" + String(INDUCTIVE_PIN));
  Serial.println("  Range: " + String(ULTRASONIC_MIN_CM) + "-" + String(ULTRASONIC_MAX_CM) + " cm");
  Serial.println("========================================");

  server.on("/", []() { sendCORS(); server.send_P(200, "text/html", PAGE); });
  server.on("/set", []() {
    sendCORS();
    moveJoint(server.arg("servo").toInt(), server.arg("angle").toFloat(), server.arg("time").toInt());
    server.send(200, "text/plain", "OK");
  });
  server.on("/set_all", []() {
    sendCORS();
    float degs[NUM_JOINTS];
    for (int j = 0; j < NUM_JOINTS; j++) degs[j] = server.arg("j" + String(j)).toFloat();
    moveAll(degs, server.arg("time").toInt());
    server.send(200, "text/plain", "OK");
  });
  server.on("/reset", []() {
    sendCORS();
    for (int j = 0; j < NUM_JOINTS; j++) moveJoint(j, START_DEG[j], 1000);
    server.send(200, "text/plain", "OK");
  });
  server.on("/run", []() { sendCORS(); pathStop = false; pathRun = true; server.send(200, "text/plain", "OK"); });
  server.on("/stop", []() { sendCORS(); pathStop = true; pathRun = false; pathMsg = "Stopped."; server.send(200, "text/plain", "OK"); });
  server.on("/status", []() { sendCORS(); server.send(200, "text/plain", pathMsg); });
  server.on("/clear_path", []() { sendCORS(); pathLength = 0; server.send(200, "text/plain", "OK"); });
  server.on("/add_wp", []() {
    sendCORS();
    if (pathLength < MAX_STEPS) {
      for (int j = 0; j < NUM_JOINTS; j++) waypoints[pathLength][j] = server.arg("j" + String(j)).toFloat();
      segDur[pathLength] = server.arg("time").toInt();
      pathLength++;
      server.send(200, "text/plain", "OK");
    } else { server.send(400, "text/plain", "Path Full"); }
  });

  server.on("/sensor_status", []() {
    sendCORS();
    server.send(200, "text/plain", readInductive() ? "1" : "0");
  });
  server.on("/sensor_start", []() {
    sendCORS();
    sensorAutoEnabled = true;
    sensorAutoRunning = false;
    lastBothTriggered = false;
    pathMsg = "Sensor automation STARTED - AND logic";
    Serial.println("[SENSOR] Automation STARTED (AND logic)");
    server.send(200, "text/plain", "Sensor automation started");
  });
  server.on("/sensor_stop", []() {
    sendCORS();
    sensorAutoEnabled = false;
    sensorAutoRunning = false;
    pathMsg = "Sensor automation STOPPED";
    Serial.println("[SENSOR] Automation STOPPED");
    server.send(200, "text/plain", "Sensor automation stopped");
  });

  server.on("/sensor_full_status", []() {
    sendCORS();
    float dist = lastDistanceCm;
    bool us = (dist >= ULTRASONIC_MIN_CM && dist <= ULTRASONIC_MAX_CM);
    bool ind = lastInductive;
    bool both = us && ind;
    String json = "{";
    json += "\"auto\":" + String(sensorAutoEnabled ? 1 : 0) + ",";
    json += "\"us\":" + String(us ? 1 : 0) + ",";
    json += "\"ind\":" + String(ind ? 1 : 0) + ",";
    json += "\"both\":" + String(both ? 1 : 0) + ",";
    json += "\"dist\":" + String(dist);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/clear_sensor_metal", []() {
    sendCORS();
    sensorPathMetalLength = 0;
    Serial.println("[SENSOR] Metal path cleared");
    server.send(200, "text/plain", "Metal path cleared");
  });
  server.on("/add_sensor_metal_wp", []() {
    sendCORS();
    if (sensorPathMetalLength < MAX_STEPS) {
      for (int j = 0; j < NUM_JOINTS; j++) sensorPathMetal[sensorPathMetalLength][j] = server.arg("j" + String(j)).toFloat();
      sensorPathMetalDur[sensorPathMetalLength] = server.arg("time").toInt();
      sensorPathMetalLength++;
      server.send(200, "text/plain", "OK");
    } else { server.send(400, "text/plain", "Path Full"); }
  });
  server.on("/clear_sensor_nometal", []() {
    sendCORS();
    sensorPathNoMetalLength = 0;
    Serial.println("[SENSOR] No-metal path cleared");
    server.send(200, "text/plain", "No-metal path cleared");
  });
  server.on("/add_sensor_nometal_wp", []() {
    sendCORS();
    if (sensorPathNoMetalLength < MAX_STEPS) {
      for (int j = 0; j < NUM_JOINTS; j++) sensorPathNoMetal[sensorPathNoMetalLength][j] = server.arg("j" + String(j)).toFloat();
      sensorPathNoMetalDur[sensorPathNoMetalLength] = server.arg("time").toInt();
      sensorPathNoMetalLength++;
      server.send(200, "text/plain", "OK");
    } else { server.send(400, "text/plain", "Path Full"); }
  });

  server.onNotFound([]() {
    if (server.method() == HTTP_OPTIONS) { sendCORS(); server.send(204); }
    else { sendCORS(); server.send(404, "text/plain", "Not found"); }
  });
  server.begin();
  Serial.println("HTTP server started.");
}

void loop() {
  server.handleClient();
  handleSensorAutomation();

  if (pathRun && !pathStop && pathLength >= 2 && !sensorAutoRunning) {
    pathRun = false;
    for (int i = 0; i < pathLength - 1 && !pathStop; i++) {
      pathMsg = "Step " + String(i+1) + "/" + String(pathLength);
      moveAll(waypoints[i+1], segDur[i]);
      server.handleClient();
    }
    if (!pathStop) {
      if (loopPath) { pathRun = true; pathMsg = "Looping..."; }
      else { pathMsg = "Done."; }
    }
  }
}
