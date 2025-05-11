#include <Wire.h>
#include <Arduino_LSM9DS1.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoBLE.h>

const int buzzerPin            = 3;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// 추가된 부분: 움직임 임계치 (g)
const float accelThreshold    = 0.3f;

// 발견 후 유지 시간(ms)
const unsigned long holdTime  = 10000;
// 부저 최대 지속 시간(ms)
const unsigned long buzzerDur = 30000;
// 루프 딜레이(ms)
const int loopDelay           = 200;
// BLE 스캔 타임아웃(ms)
const unsigned long scanTimeout = 200;
// BLE 스캔 주기(ms)
const unsigned long scanPeriod = 100;

float prevX, prevY, prevZ;
unsigned long lastOwnerSeen   = 0;
bool          firstIgnored    = false;
bool          ownerInitialized = false;
unsigned long lastScanTime    = 0;  // 마지막 스캔 시간

unsigned long buzzerStart     = 0;
bool          buzzerActive    = false;

void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0); lcd.print("Init: BLE & IMU");
  lcd.setCursor(0,1); lcd.print("Please wait...");

  pinMode(buzzerPin, OUTPUT);
  noTone(buzzerPin);

  if (!IMU.begin()) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("IMU init FAIL");
    while (true);
  }
  IMU.readAcceleration(prevX, prevY, prevZ);

  if (!BLE.begin()) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("BLE init FAIL");
    while (true);
  }
  BLE.scan(true);

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("System Ready");
  delay(500);
  lcd.clear();
}

void loop() {
  unsigned long now = millis();

  // 스캔 주기 제어 (100ms마다 스캔)
  if (now - lastScanTime >= scanPeriod) {
    lastScanTime = now;
    unsigned long scanStart = now;

    // Owner 광고 감지 (타임아웃 단축)
    BLEDevice dev = BLE.available();
    while (dev && (now - scanStart < scanTimeout)) {
      if (dev.hasLocalName()) {
        String name = dev.localName();
        if (name == "owner") {
          lastOwnerSeen = now;
          ownerInitialized = true;
          Serial.println("[Owner] Detected");
          break;  // 소유자를 찾으면 즉시 스캔 중단
        }
      }
      dev = BLE.available();
      now = millis();
    }
  }

  // 소유자 초기화가 완료된 경우에만 상태 확인
  bool ownerPresent = ownerInitialized && (now - lastOwnerSeen) < holdTime;

  // 1행: Owner 상태
  lcd.setCursor(0,0);
  lcd.print(ownerPresent
            ? "Owner Present   "
            : "Owner Not Found ");

  Serial.print("[Owner] ");
  Serial.print(ownerPresent ? "Present, " : "NotFound, ");
  Serial.print(now - lastOwnerSeen);
  Serial.println(" ms ago");

  // IMU 값 항상 읽기 (LCD 미표시)
  if (IMU.accelerationAvailable()) {
    float x,y,z;
    IMU.readAcceleration(x,y,z);
    float delta = fabs(x - prevX)
                + fabs(y - prevY)
                + fabs(z - prevZ);
    prevX = x; prevY = y; prevZ = z;

    Serial.print("[IMU] Δacc=");
    Serial.println(delta,4);

    // 소유자 없을 때만 알람 트리거
    if (!ownerPresent && delta > accelThreshold) {
      if (!firstIgnored) {
        firstIgnored = true;
        Serial.println("[Motion] first ignored");
      } else if (!buzzerActive) {
        tone(buzzerPin, 1000);
        buzzerStart  = now;
        buzzerActive = true;
        Serial.println("[Motion] Alarm ON");
      }
    }
    // 소유자 복귀 시 알람 중지
    else if (ownerPresent && buzzerActive) {
      noTone(buzzerPin);
      buzzerActive = false;
      Serial.println("[Buzzer] Stopped (owner returned)");
    }
  }

  // 2행: 알람 남은 시간 또는 No Motion
  lcd.setCursor(0,1);
  if (buzzerActive) {
    unsigned long elapsed = now - buzzerStart;
    if (elapsed < buzzerDur) {
      int sec = (buzzerDur - elapsed + 999) / 1000;
      lcd.print("Remain:");
      lcd.print(sec);
      lcd.print("s   ");
      Serial.print("[Buzzer] Remain ");
      Serial.print(sec);
      Serial.println("s");
    } else {
      noTone(buzzerPin);
      buzzerActive = false;
      lcd.print("No Motion       ");
      Serial.println("[Buzzer] Timeout Off");
    }
  } else {
    lcd.print("No Motion       ");
  }

  delay(loopDelay);
}
