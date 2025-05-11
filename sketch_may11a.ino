#include <Wire.h>
#include <Arduino_LSM9DS1.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoBLE.h>
const int buzzerPin = 3;// 부저 핀
LiquidCrystal_I2C lcd(0x27, 16, 2);// I²C LCD(주소 0x27), 16×2
// 움직임 감지 임계치 (g 단위)
const float accelThreshold = 0.15;
// owner 마지막 발견 시각 기록(ms)
unsigned long lastOwnerSeen = 0;
// owner를 "발견된 상태"로 유지할 시간(ms)
const unsigned long ownerTimeout = 5000;
float prevX, prevY, prevZ;
void setup() {
  Serial.begin(115200);
  while (!Serial);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Init: BLE & IMU ");
  lcd.setCursor(0, 1); lcd.print("Please wait...  ");
// BLE 스캔 시작 (중복 광고도 계속 수신)
  if (!BLE.begin()) {
    Serial.println("[Error] BLE init failed!");
    lcd.setCursor(0, 0); lcd.print("BLE ERROR!      ");
    while (true);
  }
  BLE.scan(true); 
  Serial.println("[Setup] BLE scanning (dup enabled)");
// IMU(가속도) 초기화
  if (!IMU.begin()) {
    Serial.println("[Error] IMU init failed!");
    lcd.setCursor(0, 0); lcd.print("IMU ERROR!      ");
    while (true);
  }
  Serial.println("[Setup] IMU ready");
// 초기 가속도 값 1회 읽기
  IMU.readAcceleration(prevX, prevY, prevZ);
  lcd.setCursor(0, 0); lcd.print("System Ready    ");
  lcd.setCursor(0, 1); lcd.print("Scanning...     ");
}
void loop() {
  unsigned long now = millis();
// BLE.available()로 새로 들어온 광고 확인
BLEDevice dev = BLE.available();
  while (dev) {
String name = dev.localName();
    Serial.print("[BLE] Adv from: "); Serial.println(name);
    if (name == "owner") {
lastOwnerSeen = now;
      Serial.println("[BLE] owner detected, timestamp updated");
    }
dev = BLE.available();
  }
// owner 상태 결정
  bool ownerPresent = (now - lastOwnerSeen) < ownerTimeout;
  lcd.setCursor(0, 0);
  if (ownerPresent) {
    lcd.print("Owner Present   ");
  } else {
    lcd.print("Owner Not Found ");
  }
// 모션 감지
  if (IMU.accelerationAvailable()) {
    float x, y, z;
    IMU.readAcceleration(x, y, z);
    float delta = fabs(x - prevX) + fabs(y - prevY) + fabs(z - prevZ);
prevX = x; prevY = y; prevZ = z;
    Serial.print("[IMU] Δaccel = "); Serial.println(delta, 4);
    lcd.setCursor(0, 1);
    if (delta > accelThreshold) {
      tone(buzzerPin, 1000);
      lcd.print("Motion Detected!");
      Serial.println("[IMU] Motion detected!");
    } else {
      noTone(buzzerPin);
      lcd.print("No Motion       ");
      Serial.println("[IMU] No motion.");
    }
  }
  delay(200);
}
