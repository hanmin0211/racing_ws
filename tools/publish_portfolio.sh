#!/usr/bin/env bash
# publish_portfolio.sh — 포트폴리오 정리분을 커밋하고 main 에 병합해 GitHub 에 올린다.
# 사용: cd ~/racing_ws && bash tools/publish_portfolio.sh
set -euo pipefail
cd "$(dirname "$0")/.."

WORK_BRANCH="yongin-2026-09-05"

echo "== 0. 현재 상태 =="
git branch --show-current
git status --short | head -40
read -rp "위 변경을 확인했나요? 계속하려면 y: " ok; [[ "$ok" == "y" ]] || exit 1

echo "== 1. 실행 권한 복구 =="
chmod +x ntrip/*.sh tools/*.sh 2>/dev/null || true

echo "== 2. 계정 문자열이 남아 있는지 점검 =="
if git grep -nI "<NGII_ID>" -- . ':!tools/publish_portfolio.sh'; then
  echo "❌ 계정 아이디가 아직 남아 있습니다. 위 파일을 고친 뒤 다시 실행하세요."; exit 1
fi
echo "✅ 작업 트리에 계정 없음"

echo "== 3. 작업 브랜치(${WORK_BRANCH})에 커밋 =="
git checkout "${WORK_BRANCH}"
git add README.md docs legacy tools/publish_portfolio.sh ntrip \
        src/ngii_ntrip src/gps_localization/launch/bringup.launch.py
git status --short
read -rp "나머지 미커밋 변경(git add -A)도 함께 올릴까요? y/n: " all
[[ "$all" == "y" ]] && git add -A
git commit -m "포트폴리오 정리 — README, legacy 도구 통합, NTRIP 계정 환경변수화" || echo "(커밋할 변경 없음)"

echo "== 4. main 에 병합 =="
git checkout main
git pull --ff-only origin main || true
git merge --no-edit "${WORK_BRANCH}"

echo "== 5. push =="
git push origin main
git push origin "${WORK_BRANCH}"
git checkout "${WORK_BRANCH}"

echo
echo "✅ 완료: https://github.com/hanmin0211/racing_ws"
echo "⚠ 저장소를 Public 으로 바꾸기 전에 과거 커밋 속 계정 아이디 처리 여부를 결정하세요 (안내 참고)."
