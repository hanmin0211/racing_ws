// steering_calib.ino — 조향 센서(A15) ADC 실시간 출력 (읽기 전용, 모터 미구동, 안전)
#define Steering_Sensor A15
void setup() {
  Serial.begin(57600);
}
void loop() {
  Serial.print("STEER_ADC=");
  Serial.println(analogRead(Steering_Sensor));
  delay(100);
}
