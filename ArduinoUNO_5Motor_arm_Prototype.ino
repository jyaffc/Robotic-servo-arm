#include <Servo.h>

/* --- Pins (UNO) --- */
const uint8_t SERVO_PIN_1 = 3;   // Main joint (270°)
const uint8_t SERVO_PIN_2 = 5;   // Main joint (270°)
const uint8_t SERVO_PIN_3 = 6;   // Main joint (270°)
const uint8_t SERVO_PIN_4 = 9;   // Main joint (270°)
const uint8_t SERVO_PIN_5 = 10;  // Gripper (180°)

const uint8_t J1_X = A0, J1_Y = A1, J2_X = A2, J2_Y = A3, POT = A4;
const uint8_t J1_SW = 2, J2_SW = 4; // active-LOW, INPUT_PULLUP

/* --- Motion / Filters --- */
const int   MAIN_MIN_DEG = 0,   MAIN_MAX_DEG = 270;  // S1..S4
const int   GRIP_MIN_DEG = 0,   GRIP_MAX_DEG = 180;  // S5 only

const int   SERVO_SAFE_MIN = 5;   // safety clamp applied after offsets
const int   SERVO_SAFE_MAX = 175;

const int   DEADZONE = 40;         // around 512
const float SMOOTH_ALPHA = 0.25f;  // EMA for analogs
const float MAX_SPEED_DPS = 120.0f;
const float MIN_STEP_DEG = 0.8f;   // reduce MG995 hunting
const unsigned WRITE_PERIOD_MS = 15;

/* --- Home angles --- */
const int HOME_ANGLE_MAIN = 135; // S1..S4
const int HOME_ANGLE_GRIP = 90;  // S5

/* --- Axis options --- */
const bool INV_S1=false, INV_S2=false, INV_S3=false, INV_S4=false;
const int  OFFSET_S1=0, OFFSET_S2=0, OFFSET_S3=0, OFFSET_S4=0, OFFSET_S5=0;

/* --- Pulse ranges (tune if needed) --- */
const int MG995_MIN_US = 500, MG995_MAX_US = 2400;   // S1..S4
const int MS18_MIN_US  = 500, MS18_MAX_US  = 2400;   // S5

Servo s1,s2,s3,s4,s5;

/* filtered analogs */
float fJ1X=512,fJ1Y=512,fJ2X=512,fJ2Y=512,fP=512;

/* held angles: S1..S4 in 0..270, S5 in 0..180 */
float a1=HOME_ANGLE_MAIN, a2=HOME_ANGLE_MAIN, a3=HOME_ANGLE_MAIN, a4=HOME_ANGLE_MAIN, a5=HOME_ANGLE_GRIP;

/* last written trackers:
   - For mains, track last microseconds
   - For gripper, track last degrees (or you can switch to µs too) */
float w1_us=-1, w2_us=-1, w3_us=-1, w4_us=-1;  // mains
float w5_deg=-999;                               // gripper

/* HOME combo debounce */
const uint16_t BOTH_PRESS_MS=30, HOME_COOLDOWN_MS=600;
bool wasBoth=false; unsigned long bothStartMs=0,lastHomeTriggerMs=0;

unsigned long lastUpdateMs=0,lastWriteMs=0;

/* --- Helpers --- */
inline int angleToUs(float deg, int minDeg, int maxDeg, int minUs, int maxUs){
  if (deg < minDeg) deg = minDeg;
  if (deg > maxDeg) deg = maxDeg;
  return (int)((deg - minDeg) * (maxUs - minUs) / (float)(maxDeg - minDeg) + minUs);
}
int centeredWithDZ(int raw){
  raw=constrain(raw,0,1023);
  int c=raw-512;
  if (abs(c)<=DEADZONE) return 0;
  if (c>0) c-=DEADZONE; else c+=DEADZONE;
  int maxAfter=512-DEADZONE;
  return map(c,-maxAfter,maxAfter,-512,512);
}
float stickToVelDps(int c,bool inv){ if(inv) c=-c; return (c/512.0f)*MAX_SPEED_DPS; }
float clampMain(float d){ if (d<MAIN_MIN_DEG) d=MAIN_MIN_DEG; if (d>MAIN_MAX_DEG) d=MAIN_MAX_DEG; return d; }
float clampGrip(float d){ if (d<GRIP_MIN_DEG) d=GRIP_MIN_DEG; if (d>GRIP_MAX_DEG) d=GRIP_MAX_DEG; return d; }
int   clampSafeDeg(int d){ return constrain(d, SERVO_SAFE_MIN, SERVO_SAFE_MAX); }

/* writers */
void writeMain270(Servo &sv, float degWithOffset, float &last_us){
  // (optional) safety for physical build: clamp to ~5..175 AFTER offsets for standard horns
  // but we still scale over 0..270 -> µs
  int us = angleToUs(degWithOffset, MAIN_MIN_DEG, MAIN_MAX_DEG, MG995_MIN_US, MG995_MAX_US);
  if (fabs((float)us - last_us) >= 4.0f) { // ~4 µs guard
    sv.writeMicroseconds(us);
    last_us = us;
  }
}
void writeGrip180(Servo &sv, float degWithOffset, float &last_deg){
  float d = clampGrip(degWithOffset);
  d = clampSafeDeg((int)d);  // optional safety
  if (fabs(d - last_deg) >= MIN_STEP_DEG){
    sv.write((int)(d + 0.5f));
    last_deg = d;
  }
}

void centerAll(bool smooth=true){
  a1=a2=a3=a4=HOME_ANGLE_MAIN;
  a5=HOME_ANGLE_GRIP;
  if (!smooth){
    s1.writeMicroseconds(angleToUs(a1+OFFSET_S1, MAIN_MIN_DEG, MAIN_MAX_DEG, MG995_MIN_US, MG995_MAX_US));
    s2.writeMicroseconds(angleToUs(a2+OFFSET_S2, MAIN_MIN_DEG, MAIN_MAX_DEG, MG995_MIN_US, MG995_MAX_US));
    s3.writeMicroseconds(angleToUs(a3+OFFSET_S3, MAIN_MIN_DEG, MAIN_MAX_DEG, MG995_MIN_US, MG995_MAX_US));
    s4.writeMicroseconds(angleToUs(a4+OFFSET_S4, MAIN_MIN_DEG, MAIN_MAX_DEG, MG995_MIN_US, MG995_MAX_US));
    s5.write((int)(a5+OFFSET_S5));
    w1_us=w2_us=w3_us=w4_us=-1; w5_deg=-999; // force next write
  }
}

void setup(){
  pinMode(J1_SW, INPUT_PULLUP);
  pinMode(J2_SW, INPUT_PULLUP);

  // Attach with per-servo ranges
  s1.attach(SERVO_PIN_1, MG995_MIN_US, MG995_MAX_US);
  s2.attach(SERVO_PIN_2, MG995_MIN_US, MG995_MAX_US);
  s3.attach(SERVO_PIN_3, MG995_MIN_US, MG995_MAX_US);
  s4.attach(SERVO_PIN_4, MG995_MIN_US, MG995_MAX_US);
  s5.attach(SERVO_PIN_5, MS18_MIN_US,  MS18_MAX_US);

  // Home
  centerAll(false);

  Serial.begin(115200);
  lastUpdateMs=lastWriteMs=millis();
}

void loop(){
  /* HOME combo (both joystick buttons) */
  bool j1 = (digitalRead(J1_SW)==LOW);
  bool j2 = (digitalRead(J2_SW)==LOW);
  bool both = j1 && j2;
  unsigned long nowMs = millis();
  if (both){
    if (!wasBoth) bothStartMs = nowMs;
    else if ((nowMs-bothStartMs)>=BOTH_PRESS_MS && (nowMs-lastHomeTriggerMs)>=HOME_COOLDOWN_MS){
      centerAll(true);
      lastHomeTriggerMs = nowMs;
    }
  }
  wasBoth = both;

  /* Read & smooth */
  int rJ1X=analogRead(J1_X), rJ1Y=analogRead(J1_Y), rJ2X=analogRead(J2_X), rJ2Y=analogRead(J2_Y), rP=analogRead(POT);
  fJ1X=SMOOTH_ALPHA*rJ1X+(1-SMOOTH_ALPHA)*fJ1X;
  fJ1Y=SMOOTH_ALPHA*rJ1Y+(1-SMOOTH_ALPHA)*fJ1Y;
  fJ2X=SMOOTH_ALPHA*rJ2X+(1-SMOOTH_ALPHA)*fJ2X;
  fJ2Y=SMOOTH_ALPHA*rJ2Y+(1-SMOOTH_ALPHA)*fJ2Y;
  fP  =SMOOTH_ALPHA*rP  +(1-SMOOTH_ALPHA)*fP;

  /* dt */
  unsigned long now=millis();
  float dt=(now-lastUpdateMs)/1000.0f; if (dt<0.001f) dt=0.001f;
  lastUpdateMs=now;

  /* Velocity/hold integration — S1..S4 track 0..270°, S5 tracks 0..180° */
  a1 = clampMain(a1 + stickToVelDps(centeredWithDZ((int)fJ1X), INV_S1) * dt);
  a2 = clampMain(a2 + stickToVelDps(centeredWithDZ((int)fJ1Y), INV_S2) * dt);
  a3 = clampMain(a3 + stickToVelDps(centeredWithDZ((int)fJ2X), INV_S3) * dt);
  a4 = clampMain(a4 + stickToVelDps(centeredWithDZ((int)fJ2Y), INV_S4) * dt);

  a5 = clampGrip(map((int)fP, 0, 1023, GRIP_MIN_DEG, GRIP_MAX_DEG)); // absolute

  /* Write at fixed cadence */
  if (now - lastWriteMs >= WRITE_PERIOD_MS){
    writeMain270(s1, a1 + OFFSET_S1, w1_us);
    writeMain270(s2, a2 + OFFSET_S2, w2_us);
    writeMain270(s3, a3 + OFFSET_S3, w3_us);
    writeMain270(s4, a4 + OFFSET_S4, w4_us);
    writeGrip180 (s5, a5 + OFFSET_S5, w5_deg);
    lastWriteMs = now;
  }
}
