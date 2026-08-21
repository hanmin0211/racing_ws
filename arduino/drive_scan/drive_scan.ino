// =============================================================================
// drive_scan.ino — 구동모터를 실제로 돌리면서 전 핀 전이를 스캔
// -----------------------------------------------------------------------------
// 배경(2026-08-20): 지금까지의 펄스 스캔은 **손으로 굴린** 회전만 봤다.
//   손 회전이 느려서 엔코더 펄스를 놓쳤을 가능성을 배제하기 위해, 모터로 실제
//   주행 속도만큼 돌리면서 같은 스캔을 한다.
//   (사용자 관찰: 개루프 PWM:60 테스트에서 바퀴가 실제로 돌았다 →
//    아두이노→ms2405→모터 경로는 살아있다. 그때 엔코더도 함께 돌았을 것이다.)
//
// 동작: [정지 4초] → [구동 6초] → [정지 4초] → [구동 6초] ... 반복하며
//       구동 구간에서만 전이가 늘어나는 핀을 찾는다.
//
// ⚠ 안전: **바퀴 공중** 필수. 구동 PWM 은 90 으로 제한했고 구간마다 정지한다.
//   모터핀(2~10)은 출력으로 쓰므로 스캔 대상에서 제외한다.
// ※ 끝나면 henes_firmware 재플래시.
// =============================================================================

#define MOTOR1_PWM 5
#define MOTOR1_ENA 6
#define MOTOR1_ENB 7
#define MOTOR2_PWM 2
#define MOTOR2_ENA 3
#define MOTOR2_ENB 4
#define DRIVE_PWM  90

// 스캔 후보: 모터핀(2~10)·USB시리얼(0,1) 제외한 전 핀.
const uint8_t CAND[] = {
  11,12,13,14,15,16,17,18,19,20,21,
  22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,
  40,41,42,43,44,45,46,47,48,49,50,51,52,53,
  54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69   // A0~A15
};
const uint8_t N = sizeof(CAND) / sizeof(CAND[0]);
uint8_t prev[N];
uint16_t edges[N];
uint16_t snap[N];

void drive(bool on) {
  if (on) {
    digitalWrite(MOTOR1_ENA, HIGH); digitalWrite(MOTOR1_ENB, LOW);
    analogWrite(MOTOR1_PWM, DRIVE_PWM);
    digitalWrite(MOTOR2_ENA, HIGH); digitalWrite(MOTOR2_ENB, LOW);
    analogWrite(MOTOR2_PWM, DRIVE_PWM);
  } else {
    digitalWrite(MOTOR1_ENA, LOW); digitalWrite(MOTOR1_ENB, LOW);
    analogWrite(MOTOR1_PWM, 0);
    digitalWrite(MOTOR2_ENA, LOW); digitalWrite(MOTOR2_ENB, LOW);
    analogWrite(MOTOR2_PWM, 0);
  }
}

// sec 초 동안 폴링하며 전이를 센 뒤 상위 핀을 출력
void sample(const char *tag, float sec) {
  for (uint8_t i = 0; i < N; i++) { prev[i] = digitalRead(CAND[i]); edges[i] = 0; }
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)(sec * 1000)) {
    for (uint8_t i = 0; i < N; i++) {
      uint8_t v = digitalRead(CAND[i]);
      if (v != prev[i]) { edges[i]++; prev[i] = v; }
    }
  }
  for (uint8_t i = 0; i < N; i++) snap[i] = edges[i];
  Serial.print(tag); Serial.print(" ");
  bool any = false;
  for (uint8_t rank = 0; rank < 6; rank++) {
    int best = -1; uint16_t bv = 0;
    for (uint8_t i = 0; i < N; i++) if (snap[i] > bv) { bv = snap[i]; best = i; }
    if (best < 0 || bv == 0) break;
    any = true;
    uint8_t p = CAND[best];
    Serial.print(p >= 54 ? "A" : "D"); Serial.print(p >= 54 ? (p - 54) : p);
    Serial.print("="); Serial.print(bv); Serial.print("  ");
    snap[best] = 0;
  }
  if (!any) Serial.print("(전이 없음)");
  Serial.println();
}

void setup() {
  Serial.begin(57600);
  int mp[] = {MOTOR1_PWM,MOTOR1_ENA,MOTOR1_ENB,MOTOR2_PWM,MOTOR2_ENA,MOTOR2_ENB};
  for (int i = 0; i < 6; i++) pinMode(mp[i], OUTPUT);
  drive(false);
  for (uint8_t i = 0; i < N; i++) pinMode(CAND[i], INPUT_PULLUP);
  delay(800);
  Serial.println();
  Serial.println("=== 구동 스캔: [정지]/[구동] 번갈아 비교 (바퀴 공중!) ===");
}

void loop() {
  drive(false); delay(500);
  sample("[정지]", 4.0);

  drive(true);  delay(500);        // 기동 시간
  sample("[구동]", 6.0);
  drive(false);
  delay(1500);
}
