// =============================================================================
// board_id.ino — 지금 꽂힌 드라이버 보드가 어느 것인지 한 번에 판별
// -----------------------------------------------------------------------------
// 배경(2026-08-22): 보드가 여러 개인데 개체마다 조향센서 핀·엔코더 카운터 유무·
//   공급전압이 다르다. 교체할 때마다 수동으로 재느라 혼선이 컸다.
//
// 이 스케치 하나로 세 가지를 동시에 본다:
//   ① VCC (밴드갭 역산)              — 이 차의 정상대는 3.9~4.1V (완주 실적 4018mV)
//   ② SPI 엔코더 카운터 CS22/23 응답  — MDR0 write/read-back
//   ③ 조향센서가 A8 인지 A15 인지     — 앞바퀴를 돌리는 동안 변동폭이 큰 쪽
//
// 사용: 플래시 후 시리얼 57600. **앞바퀴(조향)를 좌우 끝까지 계속 돌린다.**
//       2초마다 판정을 출력한다.
// ※ 판별이 끝나면 henes_firmware 를 그 보드에 맞게 설정해 재플래시할 것.
//    amap 계열 : Steering_Sensor A15, ENC1_ADD 22(SPI), NO_ENCODER 0
//    ms2405 계열: Steering_Sensor A8 , 엔코더 인터럽트 직결(D18/19), NO_ENCODER 1
// =============================================================================

#include <SPI.h>

int readVccMv() {
  ADCSRB &= ~_BV(MUX5);
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRA |= _BV(ADSC); while (bit_is_set(ADCSRA, ADSC));
  ADCSRA |= _BV(ADSC); while (bit_is_set(ADCSRA, ADSC));
  uint8_t low = ADCL, high = ADCH;
  long raw = ((long)high << 8) | low;
  if (raw == 0) return 0;
  return (int)(1125300L / raw);
}

// LS7366R MDR0 에 val 을 쓰고 되읽는다
uint8_t probeCS(uint8_t cs, uint8_t val) {
  digitalWrite(cs, LOW);
  SPI.transfer(0x88); SPI.transfer(val);
  digitalWrite(cs, HIGH);
  delayMicroseconds(50);
  digitalWrite(cs, LOW);
  SPI.transfer(0x48);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  delayMicroseconds(50);
  return v;
}

int a8_min = 9999, a8_max = 0, a15_min = 9999, a15_max = 0;

void setup() {
  Serial.begin(57600);
  pinMode(22, OUTPUT); pinMode(23, OUTPUT);
  digitalWrite(22, HIGH); digitalWrite(23, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  delay(800);
  Serial.println();
  Serial.println("=== 보드 판별기 — 앞바퀴(조향)를 좌우로 계속 돌리세요 ===");
  Serial.println();
}

void loop() {
  // 조향센서 후보 관측 (2초 누적)
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    int a8 = analogRead(A8), a15 = analogRead(A15);
    if (a8 < a8_min) a8_min = a8;   if (a8 > a8_max) a8_max = a8;
    if (a15 < a15_min) a15_min = a15; if (a15 > a15_max) a15_max = a15;
    delay(5);
  }

  int vcc = readVccMv();
  bool enc22 = (probeCS(22, 0x03) == 0x03) && (probeCS(22, 0x01) == 0x01);
  bool enc23 = (probeCS(23, 0x03) == 0x03) && (probeCS(23, 0x01) == 0x01);
  int w8 = a8_max - a8_min, w15 = a15_max - a15_min;

  Serial.print("VCC="); Serial.print(vcc); Serial.print("mV ");
  Serial.print(vcc >= 3700 ? "(정상)" : "(★낮음)");   // 이 차의 정상대는 3.9~4.1V (BOD 2.7V)
  Serial.print("   엔코더카운터: ");
  Serial.print((enc22 || enc23) ? "✅있음" : "❌없음");
  Serial.print("   조향후보 A8폭="); Serial.print(w8);
  Serial.print(" A15폭="); Serial.print(w15);
  Serial.print("  → ");
  if (w8 < 100 && w15 < 100)      Serial.print("아직 판정불가(바퀴를 더 돌리세요)");
  else if (w15 > w8)              Serial.print("조향=A15");
  else                            Serial.print("조향=A8");

  // 종합 판정
  Serial.print("   |  ");
  if (vcc < 3700)                       Serial.println("★전압 낮음 — 확인 필요");
  else if ((enc22 || enc23) && w15 > w8) Serial.println("★amap 계열 (A15+SPI카운터) — 권장");
  else if (!(enc22 || enc23))           Serial.println("ms2405 계열 (엔코더 없음)");
  else                                  Serial.println("판정보류");
}
