# NGII VRS NTRIP → ZED-F9P RTK 설정

## 접속 정보 (NGII GNSS 서비스포털에서 확인, 상태=원활)
- 캐스터: `RTS2.ngii.go.kr`
- 포트: `2101`
- 마운트포인트: `VRS-RTCM32`  (rover에는 FKP보다 VRS 권장)
- 아이디: `<NGII_ID>`
- 비밀번호: **파일/채팅에 저장 금지.** 실행 시 환경변수 `NGII_PW`로만 전달.

> VRS는 rover가 NMEA GGA(대략 위치)를 캐스터로 보내야 보정이 온다.
> str2str의 `-p lat lon hgt -n 5` 옵션이 그 역할.

## 1단계: NTRIP 스트림 검증 (실내 가능)
RTK Fixed는 야외에서만 되지만, RTCM이 캐스터에서 흘러오는지는 실내에서 확인 가능.
```bash
sudo apt-get install -y rtklib          # str2str 설치 (최초 1회)
NGII_PW='본인_비밀번호' /home/han/racing_ws/ntrip/ntrip_test.sh
```
RTCM 바이트가 수신되면 계정·네트워크·GGA 경로 정상.

## 2단계: RTCM을 F9P에 주입 (야외 실차)
현재 F9P는 **USB 허브 1개**로만 연결(`/dev/ttyACM0`). 기존 `ros2-ublox-zedf9p`
드라이버는 그 포트를 점유하지만 **RTCM 주입 기능이 없음** → 같은 USB에 str2str를
동시에 붙일 수 없다(포트 충돌).

### 권장: ublox_dgnss 드라이버로 전환
- libusb로 F9P를 다뤄 tty 점유 문제가 없고, 내장 `ntrip_client`가
  VRS용 GGA 전송 + RTCM 주입을 USB 하나로 처리.
- NGII VRS + F9P + ROS2 조합에 사실상 표준.
- 다운스트림은 `/fix`(NavSatFix)만 쓰므로 드라이버 교체 영향 적음(리맵으로 흡수).

### 대안: UART2 하드웨어 주입
- F9P의 UART2에 별도 USB-UART로 RTCM 주입, 기존 드라이버 USB 유지.
- UART2 핀 물리 접근이 필요(현재 USB 허브만 있으면 불가).

## 멀티 GNSS
`ros2-ublox-zedf9p-master/ublox_gps/config/zed_f9p.yaml`에서 GPS 단독 →
GPS+GLONASS+Galileo+BeiDou+QZSS로 변경 완료(RTK Fixed 속도/유지 개선).
