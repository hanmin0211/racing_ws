#!/usr/bin/env bash
# =============================================================================
# ntrip_auth_test.sh  —  NGII NTRIP 자격증명(아이디/비번) 검증 (curl 기반)
# -----------------------------------------------------------------------------
# str2str는 결과가 "401"로만 나와 원인 구분이 애매하다. curl로 마운트포인트에
# Basic 인증 요청을 보내 HTTP 상태코드를 직접 확인한다.
#   - 200 / ICY 200 OK  → 자격증명 정상 (인증 통과)
#   - 401 Unauthorized  → 아이디/비번 틀림 (NTRIP 접속 비번 확인 필요)
#   - 403 / 기타         → 권한/승인 문제
#
# 사용법 (비번은 환경변수로만):
#   NGII_PW='접속비밀번호' ./ntrip_auth_test.sh
#   NGII_ID='발급받은_아이디' NGII_PW='...' ./ntrip_auth_test.sh   # 아이디도 지정 가능
# =============================================================================
set -u
: "${NGII_PW:?환경변수 NGII_PW에 NTRIP 접속 비밀번호를 넣어 실행하세요}"
ID="${NGII_ID:?환경변수 NGII_ID에 NTRIP 아이디를 넣어 실행하세요}"
CASTER=RTS2.ngii.go.kr
PORT=2101
MOUNT="${NGII_MOUNT:-VRS-RTCM32}"

echo "[AUTH TEST] ${CASTER}:${PORT}/${MOUNT}  (id=${ID})"
echo "-----------------------------------------------------------"
# 헤더만 확인 (-I 대신 GET + head, NTRIP은 HEAD 미지원일 수 있음)
curl -s -i -m 8 \
  -H "Ntrip-Version: Ntrip/2.0" \
  -H "User-Agent: NTRIP racing/1.0" \
  -u "${ID}:${NGII_PW}" \
  "http://${CASTER}:${PORT}/${MOUNT}" 2>&1 | head -8
echo "-----------------------------------------------------------"
echo "판독: 200/ICY 200 OK = 인증 성공 |  401 = 아이디·비번 틀림 |  403 = 권한/승인"
