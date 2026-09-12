#!/usr/bin/env bash
# use_site.sh — 로컬 좌표계 원점을 장소별 보관본에서 골라 세운다.
#
# ★ 왜 필요한가
#   원점(config/site_origin.yaml)은 유일한 정본이라 장소를 옮길 때마다 바뀐다.
#   그런데 **되돌리는 걸 잊으면 다음 주행이 통째로 죽는다** — 경로가 수백 km
#   밖으로 판정돼 global_path_publisher 가 발행을 거부하고 차가 안 움직인다.
#   .bak 하나에 의존하면 두 번 바꿀 때 원본이 날아간다. 장소별로 이름을 붙여
#   보관하고 그중에서 고른다.
#
# 사용:
#   bash tools/use_site.sh            # 현재 원점과 고를 수 있는 장소 목록
#   bash tools/use_site.sh chungju    # 학교 시험용
#   bash tools/use_site.sh yongin     # 대회장 (대회 전 반드시 이걸로)
set -u
CFG=/home/han/racing_ws/config
CUR=$CFG/site_origin.yaml

show() {
  echo "현재 원점 ($CUR):"
  grep -E '^(site|utm_epsg|origin_x|origin_y):' "$CUR" | sed 's/^/  /'
  echo
  echo "고를 수 있는 장소:"
  for f in "$CFG"/site_origin.*.yaml; do
    [ -e "$f" ] || continue
    n=$(basename "$f"); n=${n#site_origin.}; n=${n%.yaml}
    [ "$n" = "yaml" ] && continue
    [ "$n" = "prev" ] && continue        # 직전 값 보관본 — 장소가 아니다
    s=$(grep -E '^site:' "$f" | sed 's/^site: *//')
    printf '  %-10s %s\n' "$n" "$s"
  done
}

if [ $# -eq 0 ]; then
  show
  echo
  echo "바꾸려면:  bash tools/use_site.sh <장소>"
  exit 0
fi

SRC=$CFG/site_origin.$1.yaml
if [ ! -e "$SRC" ]; then
  echo "그런 장소가 없다: $1" >&2; echo >&2; show >&2; exit 1
fi

cp "$CUR" "$CFG/site_origin.prev.yaml"      # 직전 값은 항상 남긴다
cp "$SRC" "$CUR"
echo "원점을 '$1' 로 바꿨다. (직전 값: config/site_origin.prev.yaml)"
grep -E '^(site|origin_x|origin_y):' "$CUR" | sed 's/^/  /'
echo
if [ "$1" != "yongin" ]; then
  echo "⚠ 대회(용인) 전에 되돌릴 것:  bash tools/use_site.sh yongin"
fi
echo "확인:  python3 tools/preflight.py --waypoints <그 장소의 경로 파일>"
