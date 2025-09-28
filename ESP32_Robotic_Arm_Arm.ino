#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// ------------ CONFIG ------------
#define THIS_ARM_ID 1  // <<< Set 1 for Arm A's receiver, 2 for Arm B's receiver
// -------------------------------

// Servo pins
#define S1_PIN 18   // Main 270°
#define S2_PIN 19   // Main 270°
#define S3_PIN 21   // Main 270°
#define S4_PIN 22   // Main 270°
#define S5_PIN 23   // Gripper 180°

// Packet with arm_id
typedef struct {
  uint16_t j1x, j1y, j2x, j2y, pot; // 0..4095
  uint8_t  flags;                   // bit0 = HOME
  uint8_t  arm_id;                  // 1 or 2
  uint32_t seq;
} ControlPacket;

const uint8_t FLAG_HOME = 0x01;

// Motion & tuning
const int   MAIN_MIN_DEG = 0,   MAIN_MAX_DEG = 270;  // S1..S4
const int   GRIP_MIN_DEG = 0,   GRIP_MAX_DEG = 180;  // S5
const int   DEADZONE = 80;           // around 2048
const float MAX_SPEED_DPS = 120.0f;
const float MIN_STEP_DEG = 0.8f;
const uint32_t WRITE_PERIOD = 15;
const int   HOME_ANGLE_MAIN = 135;
const int   HOME_ANGLE_GRIP = 90;

const bool  INV_S1=false, INV_S2=false, INV_S3=false, INV_S4=false;
const int   OFFSET_S1=0, OFFSET_S2=0, OFFSET_S3=0, OFFSET_S4=0, OFFSET_S5=0;

// Pulse windows (tune to your servos)
const int MG995_MIN_US = 500, MG995_MAX_US = 2400; // mains
const int MS18_MIN_US  = 500, MS18_MAX_US  = 2400; // gripper

Servo s1,s2,s3,s4,s5;

// Held angles
float a1=HOME_ANGLE_MAIN, a2=HOME_ANGLE_MAIN, a3=HOME_ANGLE_MAIN, a4=HOME_ANGLE_MAIN, a5=HOME_ANGLE_GRIP;
// Last written trackers
float w1_us=-1, w2_us=-1, w3_us=-1, w4_us=-1;  // mains store last µs
float w5_deg=-999;                              // gripper stores last deg

volatile ControlPacket g_pkt;
volatile bool g_hasPkt=false;
volatile uint32_t g_lastRxMs=0;

uint32_t lastUpdateMs=0, lastWriteMs=0;

// Helpers
inline int angleToUs(float deg, int minDeg, int maxDeg, int minUs, int maxUs){
  if (deg < minDeg) deg = minDeg;
  if (deg > maxDeg) deg = maxDeg;
  return (int)((deg - minDeg) * (maxUs - minUs) / (float)(maxDeg - minDeg) + minUs);
}
int centeredWithDZ_12bit(uint16_t raw){
  int c = (int)raw - 2048;
  if (abs(c) <= DEADZONE) return 0;
  if (c > 0) c -= DEADZONE; else c += DEADZONE;
  int maxAfter = 2048 - DEADZONE;
  return map(c, -maxAfter, maxAfter, -2048, 2048);
}
float stickToVelDps(int c, bool inv){ if(inv) c = -c; return (c / 2048.0f) * MAX_SPEED_DPS; }
float clampMain(float d){ if (d < MAIN_MIN_DEG) d = MAIN_MIN_DEG; if (d > MAIN_MAX_DEG) d = MAIN_MAX_DEG; return d; }
float clampGrip(float d){ if (d < GRIP_MIN_DEG) d = GRIP_MIN_DEG; if (d > GRIP_MAX_DEG) d = GRIP_MAX_DEG; return d; }

void writeMain270(Servo &sv, float degWithOffset, float &last_us){
  int us = angleToUs(degWithOffset, MAIN_MIN_DEG, MAIN_MAX_DEG, MG995_MIN_US, MG995_MAX_US);
  if (fabs((float)us - last_us) >= 4.0f) { sv.writeMicroseconds(us); last_us = us; }
}
void writeGrip180(Servo &sv, float degWithOffset, float &last_deg){
  float d = clampGrip(degWithOffset);
  if (fabs(d - last_deg) >= MIN_STEP_DEG) { sv.write((int)(d + 0.5f)); last_deg = d; }
}

void centerAll(bool smooth=true){
  a1=a2=a3=a4=HOME_ANGLE_MAIN; a5=HOME_ANGLE_GRIP;
  if (!smooth){
    s1.writeMicroseconds(angleToUs(a1+OFFSET_S1, MAIN_MIN_DEG, MAIN_MAX_DEG, MG995_MIN_US, MG995_MAX_US));
    s2.writeMicroseconds(angleToUs(a2+OFFSET_S2, MAIN_MIN_DEG, MAIN_MAX_DEG, MG995_MIN_US, MG995_MAX_US));
    s3.writeMicroseconds(angleToUs(a3+OFFSET_S3, MAIN_MIN_DEG, MAIN_MAX_DEG, MG995_MIN_US, MG995_MAX_US));
    s4.writeMicroseconds(angleToUs(a4+OFFSET_S4, MAIN_MIN_DEG, MAIN_MAX_DEG, MG995_MIN_US, MG995_MAX_US));
    s5.write((int)(a5+OFFSET_S5));
    w1_us=w2_us=w3_us=w4_us=-1; w5_deg=-999;
  }
}

// ESP-NOW receive callback — drops packets for other arms
void onRecv(const uint8_t*, const uint8_t* data, int len){
  if (len == (int)sizeof(ControlPacket)) {
    ControlPacket tmp;
    memcpy(&tmp, data, sizeof(tmp));
    if (tmp.arm_id != THIS_ARM_ID) return;  // ignore packets for other arms
    g_pkt = tmp;
    g_hasPkt = true;
    g_lastRxMs = millis();
  }
}

void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("Receiver MAC: "); Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK){ Serial.println("ESP-NOW init failed"); while (1) delay(10); }
  esp_now_register_recv_cb(onRecv);

  s1.setPeriodHertz(50); s2.setPeriodHertz(50); s3.setPeriodHertz(50); s4.setPeriodHertz(50); s5.setPeriodHertz(50);
  s1.attach(S1_PIN, MG995_MIN_US, MG995_MAX_US);
  s2.attach(S2_PIN, MG995_MIN_US, MG995_MAX_US);
  s3.attach(S3_PIN, MG995_MIN_US, MG995_MAX_US);
  s4.attach(S4_PIN, MG995_MIN_US, MG995_MAX_US);
  s5.attach(S5_PIN, MS18_MIN_US, MS18_MAX_US);

  centerAll(false);
  lastUpdateMs = lastWriteMs = millis();
}

void loop(){
  static ControlPacket pktLocal; static uint32_t lastRxSeen = 0;

  if (g_hasPkt){ noInterrupts(); pktLocal = g_pkt; g_hasPkt = false; interrupts(); lastRxSeen = g_lastRxMs; }

  uint32_t now = millis();
  float dt = (now - lastUpdateMs) / 1000.0f; if (dt < 0.001f) dt = 0.001f; lastUpdateMs = now;
  bool linkAlive = (now - lastRxSeen) < 200;

  if (linkAlive && (pktLocal.flags & FLAG_HOME)) centerAll(true);

  int c1=0,c2=0,c3=0,c4=0; int potRaw = 2048;
  if (linkAlive){
    c1 = centeredWithDZ_12bit(pktLocal.j1x);
    c2 = centeredWithDZ_12bit(pktLocal.j1y);
    c3 = centeredWithDZ_12bit(pktLocal.j2x);
    c4 = centeredWithDZ_12bit(pktLocal.j2y);
    potRaw = pktLocal.pot;
  }

  a1 = clampMain(a1 + stickToVelDps(c1, INV_S1) * dt);
  a2 = clampMain(a2 + stickToVelDps(c2, INV_S2) * dt);
  a3 = clampMain(a3 + stickToVelDps(c3, INV_S3) * dt);
  a4 = clampMain(a4 + stickToVelDps(c4, INV_S4) * dt);
  a5 = clampGrip(map(potRaw, 0, 4095, GRIP_MIN_DEG, GRIP_MAX_DEG));

  if (now - lastWriteMs >= WRITE_PERIOD){
    writeMain270(s1, a1 + OFFSET_S1, w1_us);
    writeMain270(s2, a2 + OFFSET_S2, w2_us);
    writeMain270(s3, a3 + OFFSET_S3, w3_us);
    writeMain270(s4, a4 + OFFSET_S4, w4_us);
    writeGrip180 (s5, a5 + OFFSET_S5, w5_deg);
    lastWriteMs = now;
  }
}
