// =============================================================================
// d23_probe.ino — D2/D3 가 엔코더 A/B 인지 검증 (INT0/INT1)
// -----------------------------------------------------------------------------
// 가설(2026-08-22, 사용자 제보): ms2405 는 **D2/D3 를 엔코더**로, **D4/D5 를 종방향
// (구동)** 으로 쓴다.
//
// 이게 맞다면 지금까지 안 잡힌 이유가 전부 설명된다:
//   ① henes_firmware 는 D2=MOTOR2_PWM, D3=MOTOR2_ENA 로 **출력 구동** 중이었다
//      → 아두이노가 미는 신호에 엔코더 입력이 묻힌다.
//   ② 펄스 스캔 때 모터핀(2~10)은 '풀업 걸면 모터가 돈다'는 이유로 **INPUT(풀업 없음)**
//      으로만 읽었다 → 엔코더가 오픈컬렉터 출력이면 읽히지 않는다.
//
// 검증: D2/D3 를 **INPUT_PULLUP** 으로 두고(아두이노는 아무것도 출력하지 않음)
//       바퀴를 굴리며 전이를 센다. 정지/회전 대조로 판정한다.
//       비교를 위해 D4/D5(구동 후보)와 기준핀 D30 도 함께 읽는다.
//
// ⚠ 바퀴 공중 필수. 이 스케치는 모터를 구동하지 않지만, 드라이버 입력이 뜰 수 있다.
// ※ 끝나면 henes_firmware 재플래시.
// =============================================================================

struct P { uint8_t n; const char *nm; };
P PINS[] = {
  {2,  "D2 "}, {3,  "D3 "},      // ★ 엔코더 후보 (INT0/INT1)
  {4,  "D4 "}, {5,  "D5 "},      // 구동 후보(대조)
  {18, "D18"}, {19, "D19"},      // 현재 펌웨어가 엔코더로 보는 곳(대조)
  {30, "D30"},                   // 아무것도 안 물린 기준핀
};
const uint8_t N = sizeof(PINS) / sizeof(PINS[0]);
uint8_t prev[N];
uint16_t edg[N];

void reset_counts() {
  for (uint8_t i = 0; i < N; i++) { prev[i] = digitalRead(PINS[i].n); edg[i] = 0; }
}
void poll(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    for (uint8_t i = 0; i < N; i++) {
      uint8_t v = digitalRead(PINS[i].n);
      if (v != prev[i]) { edg[i]++; prev[i] = v; }
    }
  }
}
void report(const char *tag) {
  Serial.print(tag);
  for (uint8_t i = 0; i < N; i++) {
    Serial.print("  "); Serial.print(PINS[i].nm);
    Serial.print("="); Serial.print(edg[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(57600);
  // ★ 전부 INPUT_PULLUP — 아두이노는 어떤 핀도 출력하지 않는다.
  for (uint8_t i = 0; i < N; i++) pinMode(PINS[i].n, INPUT_PULLUP);
  delay(800);
  Serial.println();
  Serial.println("=== D2/D3 엔코더 검증 (전 핀 INPUT_PULLUP, 아두이노 출력 없음) ===");
  Serial.println("정지 구간과 회전 구간을 번갈아 보고합니다. 회전에서만 크게 오르는 핀이 엔코더입니다.");
  Serial.println();
}

void loop() {
  reset_counts(); poll(4000); report("[정지]");
  reset_counts(); poll(6000); report("[회전]");
}
