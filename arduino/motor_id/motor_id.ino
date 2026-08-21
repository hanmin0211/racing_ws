// =============================================================================
// motor_id.ino — 전륜(D5/6/7) / 후륜(D2/3/4) 을 따로 돌려 실제 배선을 판별
// -----------------------------------------------------------------------------
// 배경(2026-08-22): "ms2405 는 D2/D3 가 엔코더, D4/D5 가 종방향"이라는 제보가 있다.
//   현재 펌웨어는 D2=MOTOR2_PWM, D3=MOTOR2_ENA, D4=MOTOR2_ENB 로 **후륜 구동**에 쓴다.
//   지금까지는 전륜·후륜을 늘 함께 돌려서 어느 쪽이 실제로 도는지 구분되지 않았다.
//
// 판별: 한 번에 하나씩만 구동한다.
//   · [전륜] 구간에 바퀴가 돌고 [후륜] 구간에 안 돈다
//       → D2/D3/D4 는 모터 제어가 아니다 ⇒ 제보대로 엔코더일 가능성이 크다
//   · 둘 다 돈다 → 현재 핀맵이 맞다(제보가 틀림)
//
// ⚠ 바퀴 공중 필수. PWM 100 으로 3초씩만 구동한다.
// ※ 끝나면 henes_firmware 재플래시.
// =============================================================================

#define M1_PWM 5
#define M1_ENA 6
#define M1_ENB 7
#define M2_PWM 2
#define M2_ENA 3
#define M2_ENB 4
#define PWMVAL 100

void allOff() {
  digitalWrite(M1_ENA, LOW); digitalWrite(M1_ENB, LOW); analogWrite(M1_PWM, 0);
  digitalWrite(M2_ENA, LOW); digitalWrite(M2_ENB, LOW); analogWrite(M2_PWM, 0);
}

void setup() {
  Serial.begin(57600);
  int p[] = {M1_PWM, M1_ENA, M1_ENB, M2_PWM, M2_ENA, M2_ENB};
  for (int i = 0; i < 6; i++) pinMode(p[i], OUTPUT);
  allOff();
  delay(1000);
  Serial.println();
  Serial.println("=== 전륜/후륜 개별 구동 판별 (바퀴 공중!) ===");
  Serial.println("각 구간에 '어느 바퀴가 도는지' 눈으로 확인하세요.");
}

void loop() {
  Serial.println(">>> [전륜] D5/D6/D7 구동 3초 — 지금 도는 바퀴는?");
  digitalWrite(M1_ENA, HIGH); digitalWrite(M1_ENB, LOW); analogWrite(M1_PWM, PWMVAL);
  delay(3000);
  allOff();
  Serial.println("    (정지 2초)");
  delay(2000);

  Serial.println(">>> [후륜] D2/D3/D4 구동 3초 — 지금 도는 바퀴는? (안 돌면 D2/D3 는 모터가 아님)");
  digitalWrite(M2_ENA, HIGH); digitalWrite(M2_ENB, LOW); analogWrite(M2_PWM, PWMVAL);
  delay(3000);
  allOff();
  Serial.println("    (정지 3초)");
  delay(3000);
}
