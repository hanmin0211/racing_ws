#!/usr/bin/env bash
# =============================================================================
# ntrip_test.sh  —  NGII VRS NTRIP 연결 테스트 (실내에서 가능)
# -----------------------------------------------------------------------------
# RTK Fixed 자체는 밖에서만 되지만, "NGII 캐스터에서 RTCM 보정신호가 실제로
# 흘러오는지"는 네트워크 테스트라 실내에서도 확인할 수 있다. 이 스크립트는
# 8초간 RTCM을 받아 파일에 쌓아보고, 바이트가 들어오면 성공이다.
#
# 사용법 (비밀번호는 채팅/파일에 남기지 말고 환경변수로만):
#   NGII_PW='여기에_NGII_비밀번호' ./ntrip_test.sh
#
# 사전: sudo apt-get install -y rtklib   (str2str 설치)
# =============================================================================
set -u
: "${NGII_PW:?환경변수 NGII_PW에 NGII 비밀번호를 넣어 실행하세요. 예: NGII_PW='****' ./ntrip_test.sh}"

ID=<NGII_ID>
CASTER=RTS2.ngii.go.kr
PORT=2101
MOUNT=VRS-RTCM32          # 접속경로 창에서 확인한 마운트포인트(원활)

# 비밀번호에 @, :, / 같은 URL 특수문자가 있으면 NTRIP 주소 파싱이 깨져서
# 401이 난다. URL 인코딩(@ -> %40 등)해서 넣는다. (str2str가 %XX를 디코드하면
# 성공. 만약 그래도 401이면 NGII 포털에서 비밀번호를 영문+숫자로 바꿔야 함.)
ENC_PW=$(python3 -c "import urllib.parse,os;print(urllib.parse.quote(os.environ['NGII_PW'],safe=''))")

# VRS는 rover의 대략 위치(NMEA GGA)를 캐스터로 보내야 보정이 온다.
# 실내 테스트라 충주 상시관측소(CGJU) 근방 좌표를 사용한다.
LAT=36.9706; LON=127.8748; HGT=100

OUT=/tmp/ngii_rtcm_test.bin
LOG=/tmp/ngii_str2str.log
rm -f "$OUT" "$LOG"

echo "[NTRIP TEST] ${CASTER}:${PORT}/${MOUNT} (id=${ID}) 8초 수신 시도..."
# -n 은 밀리초 단위(nmea request cycle). VRS는 GGA를 주기적으로 보내야 하므로
# 1000ms(1초)마다 -p 위치의 GGA를 캐스터로 전송한다.
timeout 8 str2str \
  -in "ntrip://${ID}:${ENC_PW}@${CASTER}:${PORT}/${MOUNT}" \
  -p ${LAT} ${LON} ${HGT} -n 1000 \
  -out "file://${OUT}" 2> "$LOG" || true

SZ=$(stat -c%s "$OUT" 2>/dev/null || echo 0)
echo "[결과] 수신된 RTCM 바이트: ${SZ}"
if [ "${SZ}" -gt 0 ]; then
  echo "✅ 성공 — NGII VRS에서 RTCM 보정신호가 정상 수신됩니다."
  echo "   (밖에서 이 RTCM을 F9P에 넣으면 RTK Fixed로 갑니다.)"
else
  echo "❌ 실패 — RTCM이 안 옵니다. str2str 로그:"
  echo "-----------------------------------------------------------"
  cat "$LOG"
  echo "-----------------------------------------------------------"
  echo "점검: 1) 비밀번호  2) 마운트포인트  3) 동시접속 제한  4) 방화벽"
fi
