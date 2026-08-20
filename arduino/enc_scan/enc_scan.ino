// =============================================================================
// enc_scan.ino — 엔코더 SPI 카운터(LS7366R) 진단 / CS핀 스캔
// -----------------------------------------------------------------------------
// 증상: henes_firmware 에서 ENC1 이 손으로 바퀴를 굴려도 정확히 0 고정.
//       (조향 A8 아날로그는 정상 → 아두이노·보드 전원은 살아있음)
// 목적: SPI 통신이 되는지, 된다면 카운터 칩의 CS 가 어느 핀인지 실측으로 찾는다.
//       조향센서를 adc_scan 으로 A8 로 찾아낸 것과 같은 접근.
//
// 원리: LS7366R 의 MDR0 레지스터에 알려진 값을 쓰고(0x88) 되읽어(0x48) 비교한다.
//       칩이 그 CS 에 붙어있으면 쓴 값이 그대로 돌아온다.
//       - 되읽기가 0x00 만 나옴  → MISO 가 LOW 고정 (미연결/칩 전원없음)
//       - 되읽기가 0xFF 만 나옴  → MISO 풀업 상태로 뜸 (미연결)
//       - 쓴 값이 그대로 옴      → ★그 핀이 CS
//
// 사용:
//   arduino --upload --board arduino:avr:mega:cpu=atmega2560 --port /dev/arduino \
//           arduino/enc_scan/enc_scan.ino
//   그다음 시리얼 57600 으로 결과 확인.
// ※ 스캔이 끝나면 henes_firmware 를 다시 플래시할 것.
// =============================================================================

#include <SPI.h>

// 스캔 후보: 디지털 22~49 + 53(SS).
// 제외 — 2~10: 구동/조향 모터(건드리면 위험), 11~13: 소나, 50/51/52: MISO/MOSI/SCK.
const uint8_t CAND[] = {
  22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,
  38,39,40,41,42,43,44,45,46,47,48,49,53
};
const uint8_t NCAND = sizeof(CAND) / sizeof(CAND[0]);

// MDR0 에 val 을 쓰고 되읽어서 반환.
uint8_t probe(uint8_t cs, uint8_t val) {
  digitalWrite(cs, LOW);
  SPI.transfer(0x88);          // WR_REG | MDR0
  SPI.transfer(val);
  digitalWrite(cs, HIGH);
  delayMicroseconds(50);
  digitalWrite(cs, LOW);
  SPI.transfer(0x48);          // RD_REG | MDR0
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  delayMicroseconds(50);
  return v;
}

void setup() {
  Serial.begin(57600);
  delay(800);
  Serial.println();
  Serial.println("=== 엔코더 SPI 스캔 (LS7366R MDR0 write/read-back) ===");

  // 모든 후보를 OUTPUT/HIGH(비활성)로 두어 서로 간섭 없게 한다.
  for (uint8_t i = 0; i < NCAND; i++) {
    pinMode(CAND[i], OUTPUT);
    digitalWrite(CAND[i], HIGH);
  }
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  uint8_t found = 0;
  for (uint8_t i = 0; i < NCAND; i++) {
    uint8_t cs = CAND[i];
    // 서로 다른 두 값으로 검증 → 우연한 일치(0x00/0xFF 고정) 배제
    uint8_t r1 = probe(cs, 0x03);
    uint8_t r2 = probe(cs, 0x01);
    bool hit = (r1 == 0x03 && r2 == 0x01);

    Serial.print("  D");
    if (cs < 10) Serial.print(' ');
    Serial.print(cs);
    Serial.print(" : wrote 0x03->read 0x"); if (r1 < 16) Serial.print('0'); Serial.print(r1, HEX);
    Serial.print(" , wrote 0x01->read 0x"); if (r2 < 16) Serial.print('0'); Serial.print(r2, HEX);
    if (hit) { Serial.print("   <<== ★ 카운터 칩 발견"); found++; }
    Serial.println();
  }

  SPI.endTransaction();
  Serial.println("--------------------------------------------------");
  if (found == 0) {
    Serial.println("결과: 응답하는 CS 핀 없음.");
    Serial.println("  → 되읽기가 전부 0x00 이면 MISO(D50) 가 LOW 고정 = 미연결/칩 전원 없음.");
    Serial.println("  → 전부 0xFF 여도 미연결. 엔코더 케이블/카운터보드 전원을 물리 점검할 것.");
  } else {
    Serial.print("결과: "); Serial.print(found); Serial.println(" 개 핀에서 카운터 칩 응답.");
    Serial.println("  → henes_firmware 의 ENC1_ADD/ENC2_ADD 를 이 핀으로 맞출 것.");
  }
  Serial.println("=== 스캔 종료 (henes_firmware 재플래시 필요) ===");
}

void loop() {}
