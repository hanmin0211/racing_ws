// =============================================================================
// bus_scan.ino — ms2405 가 엔코더/속도를 UART 나 I2C 로 보내는지 확인
// -----------------------------------------------------------------------------
// 배경(2026-08-20):
//   엔코더 무신호. SPI CS 29핀·전 디지털핀 펄스·A0~A15 아날로그 모두 무응답.
//   그런데 ★D17 에서만 전이가 잡혔었다(2초당 44~92). 바퀴 정지 상태에서도 뛰길래
//   노이즈로 판정했는데, **D17 은 Serial2 의 RX** 다.
//   ms2405 가 UART 로 상태를 주기 전송한다면 정지 중에도 신호가 뛰는 게 정상이므로
//   그 판정이 틀렸을 수 있다. → 실제로 UART 데이터인지 확인한다.
//
//   ms2405 는 조향 포텐셔미터도 amap(A15)과 다른 A8 로 보냈다. 즉 이 보드는 자기
//   방식대로 신호를 배치한다. 엔코더도 SPI 가 아닌 다른 버스로 줄 수 있다.
//
// 검사:
//   1) UART: Serial1(RX=19) / Serial2(RX=17) / Serial3(RX=15) 를
//      9600~115200 로 각각 열어 수신 바이트가 있는지 + 샘플을 hex/ascii 로 출력.
//   2) I2C: 주소 스캔(1~126). 응답하는 슬레이브가 있으면 그 주소 출력.
//
// 사용:
//   arduino --upload --board arduino:avr:mega:cpu=atmega2560 --port /dev/arduino \
//           arduino/bus_scan/bus_scan.ino
//   시리얼 57600 으로 결과 확인. (바퀴는 굴리지 않아도 된다 — 먼저 '데이터 존재'만 본다)
// ※ 끝나면 henes_firmware 재플래시.
// =============================================================================

#include <Wire.h>

const long BAUDS[] = {9600, 19200, 38400, 57600, 115200};
const uint8_t NB = sizeof(BAUDS) / sizeof(BAUDS[0]);

void probeUart(HardwareSerial &port, const char *name, uint8_t rxpin) {
  for (uint8_t i = 0; i < NB; i++) {
    port.begin(BAUDS[i]);
    delay(30);
    while (port.available()) port.read();      // 버퍼 비우기
    delay(1200);                               // 이 보드레이트로 1.2초 수신
    int n = port.available();
    Serial.print("  "); Serial.print(name);
    Serial.print("(RX=D"); Serial.print(rxpin); Serial.print(")  ");
    Serial.print(BAUDS[i]); Serial.print(" bps : ");
    Serial.print(n); Serial.print(" bytes");
    if (n > 0) {
      Serial.print("   [");
      int lim = n > 16 ? 16 : n;
      for (int k = 0; k < lim; k++) {
        uint8_t b = port.read();
        if (b < 16) Serial.print('0');
        Serial.print(b, HEX); Serial.print(' ');
      }
      Serial.print("] ascii:");
      // 위에서 읽은 만큼은 소비됐으니 남은 것으로 ascii 표시
      int m = port.available(); int lim2 = m > 16 ? 16 : m;
      for (int k = 0; k < lim2; k++) {
        char c = (char)port.read();
        Serial.print((c >= 32 && c < 127) ? c : '.');
      }
      Serial.print("   <<== ★ 데이터 있음");
    }
    Serial.println();
    while (port.available()) port.read();
    port.end();
  }
}

void setup() {
  Serial.begin(57600);
  delay(1000);
  Serial.println();
  Serial.println("=== ms2405 버스 스캔 (UART / I2C) ===");
  Serial.println();

  Serial.println("[1] UART 수신 확인");
  probeUart(Serial2, "Serial2", 17);   // D17 에서 전이가 잡혔던 곳 — 최우선
  probeUart(Serial1, "Serial1", 19);
  probeUart(Serial3, "Serial3", 15);

  Serial.println();
  Serial.println("[2] I2C 주소 스캔 (SDA=D20, SCL=D21)");
  Wire.begin();
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  ★ 0x"); if (a < 16) Serial.print('0');
      Serial.print(a, HEX); Serial.println(" 응답");
      found++;
    }
  }
  if (!found) Serial.println("  응답하는 I2C 슬레이브 없음");

  Serial.println();
  Serial.println("=== 스캔 종료 (henes_firmware 재플래시 필요) ===");
}

void loop() {}
