/*
 * CarePath - 환자 착용 낙상 감지 태그
 *
 * MPU6500 계열 6축 센서로 낙상을 판정하고,
 * 부저 경보와 함께 UDP로 로봇과 모니터링 PC에 신호를 전송한다.
 *
 * 배선
 *   MPU6500  VCC-3V3  GND-GND  SCL-D22  SDA-D21
 *   부저     S-D25    VCC-VIN  GND-GND
 */
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// 네트워크 설정 - 실행 환경에 맞게 수정한다
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";
const int   UDP_PORT  = 5005;

IPAddress PI_IP(192, 168, 0, 100);       // 로봇 제어부
IPAddress LAPTOP_IP(192, 168, 0, 101);   // 모니터링 PC, 미사용 시 마지막 자리를 0으로

#define BUZZER_PIN 25
#define CANCEL_BTN  0     // 보드 내장 BOOT 버튼을 알람 해제에 사용
#define MPU_ADDR   0x68

// 낙상 판정 기준
const float FREEFALL_TH = 0.55;   // 자유낙하로 간주할 가속도 (g)
const float IMPACT_TH   = 2.60;   // 자유낙하 직후 충격으로 간주할 가속도 (g)
const float BIG_HIT_TH  = 3.50;   // 자유낙하 없이도 낙상으로 볼 강한 충격 (g)

const unsigned long FALL_WINDOW = 900;     // 자유낙하 후 충격을 기다리는 시간 (ms)
const unsigned long ALARM_DUR   = 15000;   // 알람 최대 지속 시간 (ms)
const unsigned long COOLDOWN    = 5000;    // 알람 해제 후 재감지 대기 시간 (ms)

WiFiUDP udp;
IPAddress bcastIP;

enum TagState { ST_IDLE, ST_FALLING, ST_ALARM, ST_COOLING };
TagState state = ST_IDLE;

unsigned long tFreefall = 0, tAlarmStart = 0, tCoolStart = 0;
unsigned long tLastSend = 0, tLastBeep = 0, tLog = 0;
bool beepOn = false;


void mpuWrite(byte reg, byte val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuInit() {
  Wire.begin(21, 22);
  Wire.setClock(100000);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  if (!Wire.available()) return false;
  Serial.printf("WHO_AM_I = 0x%02X\n", Wire.read());

  mpuWrite(0x6B, 0x00);   // 절전 해제
  delay(100);
  mpuWrite(0x1C, 0x10);   // 가속도 측정 범위 +-8g
  mpuWrite(0x1B, 0x08);   // 자이로 측정 범위 +-500 deg/s
  mpuWrite(0x1A, 0x03);   // 저역 통과 필터 44Hz
  return true;
}

// 가속도와 각속도의 크기를 각각 g와 deg/s 단위로 읽는다
void mpuRead(float &aMag, float &gMag) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                        // 온도 레지스터
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  float x = ax / 4096.0, y = ay / 4096.0, z = az / 4096.0;
  aMag = sqrt(x * x + y * y + z * z);

  float p = gx / 65.5, q = gy / 65.5, r = gz / 65.5;
  gMag = sqrt(p * p + q * q + r * r);
}


void connectWiFi() {
  Serial.printf("WiFi 연결 중: %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ 연결됨\n   내 IP       : %s\n",
                  WiFi.localIP().toString().c_str());
    bcastIP = (uint32_t)WiFi.localIP() | ~((uint32_t)WiFi.subnetMask());
    Serial.printf("   브로드캐스트 : %s\n", bcastIP.toString().c_str());
    Serial.printf("   로봇 제어부  : %s\n", PI_IP.toString().c_str());
    udp.begin(UDP_PORT);
  } else {
    Serial.println("\n❌ WiFi 실패. 부저 단독 모드.");
  }
}

void sendTo(IPAddress ip) {
  udp.beginPacket(ip, UDP_PORT);
  udp.print("FALL");
  udp.endPacket();
}

// 브로드캐스트가 차단된 환경을 대비해 개별 주소로도 함께 전송한다
void sendAlert() {
  if (WiFi.status() != WL_CONNECTED) return;
  sendTo(bcastIP);
  sendTo(PI_IP);
  if (LAPTOP_IP[3] != 0) sendTo(LAPTOP_IP);
  Serial.println("   → UDP 'FALL' 송신");
}

// delay 없이 부저를 단속적으로 울린다
void beepUpdate(bool active) {
  if (!active) {
    digitalWrite(BUZZER_PIN, LOW);
    beepOn = false;
    return;
  }
  if (millis() - tLastBeep >= 250) {
    beepOn = !beepOn;
    digitalWrite(BUZZER_PIN, beepOn ? HIGH : LOW);
    tLastBeep = millis();
  }
}


void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CANCEL_BTN, INPUT_PULLUP);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("\n===== 환자 태그 부팅 =====");

  if (!mpuInit()) {
    Serial.println("❌ 센서 없음. 배선 확인.");
    while (1) {
      digitalWrite(BUZZER_PIN, HIGH); delay(80);
      digitalWrite(BUZZER_PIN, LOW);  delay(920);
    }
  }
  Serial.println("✅ 센서 준비 완료");

  connectWiFi();

  // 배터리 구동 시 시리얼을 볼 수 없으므로 부팅 상태를 소리로 알린다
  if (WiFi.status() == WL_CONNECTED) {
    for (int i = 0; i < 2; i++) {
      digitalWrite(BUZZER_PIN, HIGH); delay(80);
      digitalWrite(BUZZER_PIN, LOW);  delay(80);
    }
  } else {
    digitalWrite(BUZZER_PIN, HIGH); delay(800);
    digitalWrite(BUZZER_PIN, LOW);
  }

  Serial.println("===== 감시 시작 =====\n");
}


void loop() {
  float aMag, gMag;
  mpuRead(aMag, gMag);
  unsigned long now = millis();

  switch (state) {

    // 자유낙하 또는 강한 충격을 감시한다
    case ST_IDLE:
      beepUpdate(false);
      if (aMag < FREEFALL_TH) {
        state = ST_FALLING;
        tFreefall = now;
        Serial.printf("[감지] 자유낙하 의심 (%.2f g)\n", aMag);
      } else if (aMag > BIG_HIT_TH) {
        Serial.printf("🚨 강한 충격! (%.2f g)\n", aMag);
        state = ST_ALARM;
        tAlarmStart = now;
        tLastSend = 0;
      }
      break;

    // 자유낙하 직후 충격이 이어지는지 확인한다
    case ST_FALLING:
      if (now - tFreefall > FALL_WINDOW) {
        state = ST_IDLE;
        Serial.println("[해제] 충격 없음");
      } else if (aMag > IMPACT_TH) {
        Serial.printf("🚨 낙상 확정! %.2f g / %.0f deg/s\n", aMag, gMag);
        state = ST_ALARM;
        tAlarmStart = now;
        tLastSend = 0;
      }
      break;

    // 부저를 울리며 주기적으로 신호를 재전송한다
    case ST_ALARM:
      beepUpdate(true);
      if (now - tLastSend >= 300) {
        sendAlert();
        tLastSend = now;
      }
      if (digitalRead(CANCEL_BTN) == LOW) {
        Serial.println("[해제] 수동 확인 버튼");
        state = ST_COOLING;
        tCoolStart = now;
      } else if (now - tAlarmStart > ALARM_DUR) {
        Serial.println("[해제] 알람 시간 종료");
        state = ST_COOLING;
        tCoolStart = now;
      }
      break;

    // 해제 직후 같은 충격으로 재발동하지 않도록 대기한다
    case ST_COOLING:
      beepUpdate(false);
      if (now - tCoolStart > COOLDOWN) {
        state = ST_IDLE;
        Serial.println("===== 감시 재개 =====\n");
      }
      break;
  }

  if (now - tLog >= 200) {
    Serial.printf("a=%.2fg  w=%.0f deg/s  state=%d\n", aMag, gMag, state);
    tLog = now;
  }

  delay(20);   // 약 50Hz 샘플링
}