// =============================================================================
// spi_line_probe.ino — SPI 라인에 카운터 칩이 '물려는 있는지' 전기적으로 판별
// -----------------------------------------------------------------------------
// 배경(2026-08-20~22): ms2405 교체 후 엔코더 무신호.
//   SPI CS 29핀 스캔은 전부 0x00 이었다. 그런데 0x00 은 두 경우 모두에서 나온다:
//     (a) 아무것도 안 물려 있어 MISO 가 뜬 채 LOW 로 읽힘
//     (b) 카운터 칩은 물려 있는데 **전원이 없어** MISO 를 LOW 로 끌고 있음
//   (a)와 (b)는 대처가 완전히 다르다 — (b)라면 전원만 주면 살아난다.
//
// 판별법: 아두이노 **내부 풀업(20~50k)** 을 걸고 라인 상태를 읽는다.
//   · 풀업 걸었더니 HIGH  → 라인이 비어 있다(외부에 아무것도 안 물림) ⇒ (a)
//   · 풀업 걸었는데 LOW 유지 → 외부의 무언가가 라인을 GND 로 당기고 있다 ⇒ (b)
//                              (죽은 칩의 입력, 또는 GND 단락)
//
// 대상: D50(MISO) D51(MOSI) D52(SCK) D53(SS) + CS 후보 D22/D23
//       비교용으로 확실히 비어있는 핀(D30~D33)도 같이 읽어 기준선을 만든다.
//
// ※ 끝나면 henes_firmware 재플래시.
// =============================================================================

struct Pin { uint8_t n; const char *name; };
Pin PINS[] = {
  {50, "D50 MISO "}, {51, "D51 MOSI "}, {52, "D52 SCK  "}, {53, "D53 SS   "},
  {22, "D22 CS1  "}, {23, "D23 CS2  "},
  {30, "D30 (기준)"}, {31, "D31 (기준)"}, {32, "D32 (기준)"}, {33, "D33 (기준)"},
};
const uint8_t N = sizeof(PINS) / sizeof(PINS[0]);

// 풀업을 걸고 안정화 후 여러 번 읽어 다수결
uint8_t readWithPullup(uint8_t p) {
  pinMode(p, INPUT_PULLUP);
  delay(5);
  uint8_t hi = 0;
  for (uint8_t k = 0; k < 20; k++) { if (digitalRead(p)) hi++; delayMicroseconds(200); }
  return hi;           // 0~20 (20이면 확실한 HIGH)
}
uint8_t readFloating(uint8_t p) {
  pinMode(p, INPUT);
  delay(5);
  uint8_t hi = 0;
  for (uint8_t k = 0; k < 20; k++) { if (digitalRead(p)) hi++; delayMicroseconds(200); }
  return hi;
}

void setup() {
  Serial.begin(57600);
  delay(1000);
  Serial.println();
  Serial.println("=== SPI 라인 전기적 진단 ===");
  Serial.println("풀업 20/20 = 라인 비어있음(칩 없음)");
  Serial.println("풀업  0/20 = 외부가 LOW 로 당김(칩이 물려있으나 죽음 / GND 단락)");
  Serial.println();
  Serial.println("핀          플로팅   풀업     판정");
  Serial.println("--------------------------------------------------");
  for (uint8_t i = 0; i < N; i++) {
    uint8_t f = readFloating(PINS[i].n);
    uint8_t u = readWithPullup(PINS[i].n);
    Serial.print("  "); Serial.print(PINS[i].name);
    Serial.print("  "); Serial.print(f); Serial.print("/20");
    Serial.print("   "); Serial.print(u); Serial.print("/20");
    Serial.print("   ");
    if (u >= 18)      Serial.print("라인 비어있음");
    else if (u <= 2)  Serial.print("★ 외부가 LOW 로 당김");
    else              Serial.print("불안정(노이즈)");
    Serial.println();
  }
  Serial.println("--------------------------------------------------");
  Serial.println("D30~D33(기준)과 D50~D53 을 비교할 것.");
  Serial.println("기준핀과 SPI핀이 같은 값이면 SPI 라인에도 아무것도 안 물려 있는 것이다.");
  Serial.println("=== 종료 ===");
}

void loop() {}
