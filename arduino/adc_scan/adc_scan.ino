// ADC 스캔 — 16개 아날로그 핀(A0~A15) 전부 읽어 출력.
// ms2405 보드에서 조향 센서가 실제로 어느 핀에 오는지 찾는 진단용.
// 바퀴를 손으로 돌리며, 값이 크게 변하는 핀 번호가 진짜 센서 핀이다.
void setup() { Serial.begin(57600); }
void loop() {
  Serial.print("ADC ");
  for (int i = 0; i < 16; i++) {
    Serial.print("A"); Serial.print(i); Serial.print("=");
    Serial.print(analogRead(A0 + i));
    Serial.print(i < 15 ? "  " : "\n");
  }
  delay(250);
}
