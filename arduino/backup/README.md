# 펌웨어 백업 · 되돌리기

## 무엇이 들어 있나
| 파일 | 내용 |
|---|---|
| `henes_before_20260913_1951.hex.gz` | 굽기 **전**에 보드에서 읽어낸 것 (롤백용) |
| `henes_flashed_20260913_1951.hex` | 2026-09-13 19:51 에 올린 것 |

## 되돌리는 법
차가 이상하게 움직이면 **먼저 bringup 을 끄고**(그래야 리셋 후 명령이 안 나간다)
포트가 비어 있는지 확인한 뒤:

```bash
gunzip -k arduino/backup/henes_before_20260913_1951.hex.gz
avrdude -patmega2560 -cwiring -P/dev/ttyACM0 -b115200 -D \
        -Uflash:w:arduino/backup/henes_before_20260913_1951.hex:i
```

⚠ 포트는 **VID:PID 2341:0042** 로 찾을 것. ttyACM 번호는 GPS 와 뒤바뀐다.
   `udevadm info -q property -n /dev/ttyACM0 | grep ID_MODEL_ID`

## 다시 굽는 법 (arduino-cli 없이)
이 PC 에는 `arduino-builder`(데비안 arduino 패키지)가 있다.

```bash
arduino-builder -compile \
  -hardware /usr/share/arduino/hardware \
  -tools /usr/share/arduino/hardware/tools -tools /usr/bin \
  -libraries /home/han/Arduino/libraries \
  -fqbn arduino:avr:mega:cpu=atmega2560 \
  -build-path /tmp/fwbuild \
  arduino/henes_firmware/henes_firmware.ino

avrdude -patmega2560 -cwiring -P/dev/ttyACM0 -b115200 -D \
        -Uflash:w:/tmp/fwbuild/henes_firmware.ino.hex:i
```

## 굽기 전 확인
- ROS 노드가 떠 있으면 안 된다 (`ros2 node list` 가 비어야 한다).
  리셋 직후 명령이 들어오면 차가 움직인다.
- 굽고 나면 보드가 **워치독 물린 상태**로 부팅해 명령이 올 때까지 모터가 꺼져 있다.
  다만 bringup 을 켜는 순간 조향이 중앙으로 홱 돌아가니 손을 치울 것.
