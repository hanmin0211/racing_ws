// =============================================================================
// i2c_probe.ino — I2C 0x3C 장치가 무엇인지 / 엔코더 값을 주는지 확인
// -----------------------------------------------------------------------------
// 배경(2026-08-20): bus_scan 에서 I2C 주소 **0x3C** 가 응답했다.
//   (0x3C 는 SSD1306 OLED 로 흔한 주소지만, ms2405 가 엔코더 카운트를 I2C 로
//    내보내는 것일 수도 있어 확인이 필요하다.)
//   D17 의 115200 UART 는 "Frame Lost or FailSafe" 만 반복 → RC 수신기 채널이고
//   엔코더가 아님이 확인됐다. I2C 가 마지막 남은 후보다.
//
// 방법: 0x3C 에서 주기적으로 여러 바이트를 읽어 hex 로 출력한다.
//       바퀴를 굴렸을 때 **변하는 바이트가 있으면** 그게 엔코더/속도다.
//       (OLED 같은 write-only 장치면 값이 고정이거나 0xFF 만 나온다)
//
// 사용: 플래시 후 시리얼 57600. 정지 → 회전 순으로 관찰.
// ※ 끝나면 henes_firmware 재플래시.
// =============================================================================

#include <Wire.h>

#define ADDR 0x3C
#define NBYTE 8

void setup() {
  Serial.begin(57600);
  Wire.begin();
  delay(500);
  Serial.println();
  Serial.println("=== I2C 0x3C 읽기 (바퀴 굴리며 변하는 바이트를 찾는다) ===");
}

void loop() {
  uint8_t n = Wire.requestFrom((uint8_t)ADDR, (uint8_t)NBYTE);
  Serial.print("rx="); Serial.print(n); Serial.print("  ");
  for (uint8_t i = 0; i < n; i++) {
    uint8_t b = Wire.read();
    if (b < 16) Serial.print('0');
    Serial.print(b, HEX); Serial.print(' ');
  }
  Serial.println();
  delay(200);
}
