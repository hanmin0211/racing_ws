// =============================================================================
// enc_pulse_scan.ino — 엔코더 A/B 직교펄스 핀 스캔
// -----------------------------------------------------------------------------
// 배경(2026-08-20):
//   ms2405 보드로 교체 후 ENC1 이 0 고정. enc_scan.ino 로 SPI CS 후보 29핀을
//   전수 스캔했더니 **전부 0x00 무응답** → SPI 카운터(LS7366R)가 없다.
//   그런데 엔코더 케이블은 ms2405 에 꽂혀 있다.
//   ⇒ ms2405 는 엔코더를 SPI 가 아니라 **A/B 직교 펄스로 직접** 아두이노에
//     보내는 것으로 보인다(조향 포텐셔미터를 A8 로 보내는 것과 같은 이치).
//
// 목적: 바퀴를 손으로 굴리는 동안 **어느 핀이 토글되는지** 세어 엔코더 채널을 찾는다.
//       (adc_scan 으로 조향센서 A8 을 찾아낸 것과 같은 접근)
//
// 사용:
//   arduino --upload --board arduino:avr:mega:cpu=atmega2560 --port /dev/arduino \
//           arduino/enc_pulse_scan/enc_pulse_scan.ino
//   시리얼 57600 열고 **바퀴를 계속 굴리면서** 출력을 본다.
//   2초마다 전이(edge) 횟수 상위 핀을 보고한다. 굴릴 때만 크게 뛰는 핀이 엔코더다.
//
// ⚠ 안전: 모터 핀(2~10)과 소나(11~13)는 스캔에서 제외한다. 그 핀에 INPUT_PULLUP 을
//   걸면 드라이버 입력이 떠서 모터가 돌 수 있기 때문. SPI(50~52)도 제외.
// ※ 스캔이 끝나면 henes_firmware 를 다시 플래시할 것.
// =============================================================================

// 스캔 후보: 디지털 14~49, 53 + 아날로그핀(A0~A15 = 54~69)을 디지털로 읽기.
// 메가의 외부인터럽트 핀은 2,3,18,19,20,21 인데 2·3 은 모터라 제외 →
// **18/19/20/21 이 가장 유력한 후보**다(엔코더는 보통 인터럽트 핀에 배선).
// ★ 2026-08-20 1차 스캔 결과: D17 만 뛰었으나 **바퀴 정지 상태에서도 동일**(44~86)
//   → 회전과 무관한 노이즈(D17=Serial2 RX). 엔코더 아님. 그래서 D17 은 제외했다.
//   1차에서 안전상 뺐던 소나(11~13)·SPI(50~52)를 2차 스캔에 추가한다.
//   (모터 핀 2~10 은 모터가 정상 동작하므로 엔코더일 수 없다 → 계속 제외)
const uint8_t CAND[] = {
  11,12,13,
  14,15,16,18,19,20,21,          // 17 제외(노이즈원)
  22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,
  40,41,42,43,44,45,46,47,48,49,50,51,52,53,
  54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69   // A0~A15 (62=A8=조향센서)
};
const uint8_t N = sizeof(CAND) / sizeof(CAND[0]);

uint8_t  prev[N];
uint16_t edges[N];
uint16_t snap[N];

void setup() {
  Serial.begin(57600);
  delay(800);
  for (uint8_t i = 0; i < N; i++) {
    pinMode(CAND[i], INPUT_PULLUP);   // 오픈컬렉터 엔코더도 읽히도록 풀업
  }
  delay(50);
  for (uint8_t i = 0; i < N; i++) {
    prev[i] = digitalRead(CAND[i]);
    edges[i] = 0;
  }
  Serial.println();
  Serial.println("=== 엔코더 A/B 펄스 스캔 ===");
  Serial.println("바퀴를 계속 굴리세요. 2초마다 전이(edge) 상위 핀을 보고합니다.");
  Serial.println("굴릴 때만 수치가 크게 오르는 핀이 엔코더 채널입니다.");
  Serial.println();
}

void loop() {
  static unsigned long last = 0;

  // 최대한 빠르게 폴링해 전이를 놓치지 않는다
  for (uint8_t i = 0; i < N; i++) {
    uint8_t v = digitalRead(CAND[i]);
    if (v != prev[i]) { edges[i]++; prev[i] = v; }
  }

  unsigned long now = millis();
  if (now - last >= 2000) {
    last = now;
    // 이번 구간 스냅샷을 뜨고 즉시 카운터를 비운다(출력 중 유실 최소화)
    for (uint8_t i = 0; i < N; i++) { snap[i] = edges[i]; edges[i] = 0; }

    Serial.print("[2s] ");
    bool any = false;
    for (uint8_t rank = 0; rank < 6; rank++) {
      int best = -1; uint16_t bv = 0;
      for (uint8_t i = 0; i < N; i++) {
        if (snap[i] > bv) { bv = snap[i]; best = i; }
      }
      if (best < 0 || bv == 0) break;
      any = true;
      uint8_t p = CAND[best];
      Serial.print(p >= 54 ? "A" : "D");
      Serial.print(p >= 54 ? (p - 54) : p);
      Serial.print("="); Serial.print(bv); Serial.print("  ");
      snap[best] = 0;   // 출력했으니 비워 다음 순위가 뽑히게
    }
    if (!any) Serial.print("(전이 없음 — 바퀴를 굴려주세요)");
    Serial.println();
  }
}
