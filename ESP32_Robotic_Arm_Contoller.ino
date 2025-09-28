#include <WiFi.h>
#include <esp_now.h>

// ------------ CONFIG ------------
#define ARM_ID 1   // <<< Set 1 for Arm A's controller, 2 for Arm B's controller

uint8_t RECEIVER_MAC[] = { 0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC }; // <<< UPDATE to your receiver's MAC
// -------------------------------

// Pins (ADC1 + buttons)
#define J1X   34
#define J1Y   35
#define J2X   32
#define J2Y   33
#define GP    36
#define J1_SW 25
#define J2_SW 26

// Packet with arm_id
typedef struct {
  uint16_t j1x, j1y, j2x, j2y, pot; // 0..4095
  uint8_t  flags;                   // bit0 = HOME
  uint8_t  arm_id;                  // 1 or 2
  uint32_t seq;
} ControlPacket;

const uint8_t FLAG_HOME = 0x01;

esp_now_peer_info_t peerInfo{};
volatile uint32_t seqCounter = 0;

// smoothing
static float fJ1X = 2048, fJ1Y = 2048, fJ2X = 2048, fJ2Y = 2048, fP = 2048;
const float ALPHA = 0.25f;

// home debounce
const uint16_t BOTH_PRESS_MS = 30;
const uint16_t HOME_COOLDOWN_MS = 600;
bool wasBoth = false;
uint32_t bothStartMs = 0, lastHomeSentMs = 0;

void setup() {
  Serial.begin(115200);
  pinMode(J1_SW, INPUT_PULLUP);
  pinMode(J2_SW, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  Serial.print("Sender MAC: "); Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); while (1) delay(10); }

  memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false; // keep false unless you add LMKs on both ends
  if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("Peer add failed"); while (1) delay(10); }

  analogReadResolution(12); // 0..4095
}

void loop() {
  // analog reads
  int rJ1X = analogRead(J1X);
  int rJ1Y = analogRead(J1Y);
  int rJ2X = analogRead(J2X);
  int rJ2Y = analogRead(J2Y);
  int rP   = analogRead(GP);

  // smooth
  fJ1X = ALPHA * rJ1X + (1 - ALPHA) * fJ1X;
  fJ1Y = ALPHA * rJ1Y + (1 - ALPHA) * fJ1Y;
  fJ2X = ALPHA * rJ2X + (1 - ALPHA) * fJ2X;
  fJ2Y = ALPHA * rJ2Y + (1 - ALPHA) * fJ2Y;
  fP   = ALPHA * rP   + (1 - ALPHA) * fP;

  // buttons -> HOME flag
  bool b1 = (digitalRead(J1_SW) == LOW);
  bool b2 = (digitalRead(J2_SW) == LOW);
  bool both = b1 && b2;
  uint32_t now = millis();

  uint8_t flags = 0;
  if (both) {
    if (!wasBoth) bothStartMs = now;
    else if ((now - bothStartMs) >= BOTH_PRESS_MS && (now - lastHomeSentMs) >= HOME_COOLDOWN_MS) {
      flags |= FLAG_HOME;
      lastHomeSentMs = now;
    }
  }
  wasBoth = both;

  // send packet
  ControlPacket pkt;
  pkt.j1x = (uint16_t)constrain((int)fJ1X, 0, 4095);
  pkt.j1y = (uint16_t)constrain((int)fJ1Y, 0, 4095);
  pkt.j2x = (uint16_t)constrain((int)fJ2X, 0, 4095);
  pkt.j2y = (uint16_t)constrain((int)fJ2Y, 0, 4095);
  pkt.pot = (uint16_t)constrain((int)fP,   0, 4095);
  pkt.flags = flags;
  pkt.arm_id = ARM_ID;              // <<< include our ID
  pkt.seq = seqCounter++;

  esp_now_send(RECEIVER_MAC, (uint8_t*)&pkt, sizeof(pkt));
  delay(20); // ~50 Hz
}
