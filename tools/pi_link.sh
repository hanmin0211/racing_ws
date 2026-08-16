#!/usr/bin/env bash
# pi_link.sh — 노트북 ↔ 라즈베리파이5 랜선 직결 세팅 (매 부팅 후 1회)
#
# 파이(Hailo-8 신호등 인식)는 랜선으로 노트북에 직결된다. 노트북이 DHCP 서버
# 겸 인터넷 공유 게이트웨이 역할을 한다.
#
#   노트북 enp3s0 = 192.168.99.1   ←랜선→   파이 eth0 = 192.168.99.8
#
# ★ IP 포워딩과 iptables NAT 규칙은 재부팅하면 날아간다. ufw 규칙과 고정 IP는
#   파일로 남으므로 유지된다. 그래서 이 스크립트를 매 부팅 후 한 번 돌린다.
#
# 사용법:
#   bash tools/pi_link.sh          # 설정 + dnsmasq 를 백그라운드로
#   bash tools/pi_link.sh --check  # 상태만 확인 (변경 없음)
#
# sudo 암호를 물어본다.
set -uo pipefail

ETH=enp3s0
WIFI=wlp4s0            # 매뉴얼엔 wlo1 로 되어 있으나 이 노트북은 wlp4s0
PI_IP=192.168.99.8
HOST_IP=192.168.99.1
CONN="Wired connection 1"

check() {
  echo "=== 링크 ==="
  ip -br addr show "$ETH" 2>/dev/null || echo "  $ETH 없음"
  [ "$(cat /sys/class/net/$ETH/carrier 2>/dev/null)" = "1" ] \
      && echo "  케이블 연결됨" || echo "  ⚠ 케이블 없음"
  echo "=== dnsmasq ==="
  pgrep -a dnsmasq >/dev/null && echo "  실행 중" || echo "  ⚠ 실행 안 됨"
  echo "=== IP 포워딩 ==="
  echo "  $(cat /proc/sys/net/ipv4/ip_forward)  (1이어야 함)"
  echo "=== 파이 ==="
  if ping -c1 -W2 "$PI_IP" &>/dev/null; then
    echo "  $PI_IP 응답"
    ssh -o BatchMode=yes -o ConnectTimeout=5 "pi@$PI_IP" \
        'echo "  SSH OK — $(hostname), $(uname -r)"' 2>/dev/null \
        || echo "  ⚠ SSH 실패 (키 등록됐나?)"
  else
    echo "  ⚠ $PI_IP 무응답"
  fi
}

if [ "${1:-}" = "--check" ]; then
  check
  exit 0
fi

echo "[1/4] 이더넷 고정 IP ($HOST_IP)"
nmcli connection modify "$CONN" ipv4.method manual \
    ipv4.addresses "$HOST_IP/24" ipv4.gateway "" ipv4.dns "" \
  && nmcli connection down "$CONN" >/dev/null \
  && nmcli connection up "$CONN" >/dev/null \
  && echo "  OK" || echo "  ⚠ 실패 — nmcli connection show 로 이름 확인"

echo "[2/4] 방화벽에서 DHCP(67/udp) 허용"
# 이게 막혀 있으면 파이가 IP를 영영 못 받는다. tcpdump 로는 패킷이 도착하는
# 것처럼 보여서 원인 찾기가 아주 어렵다 (매뉴얼 부록 A 참조).
sudo ufw allow in on "$ETH" to any port 67 proto udp

echo "[3/4] 인터넷 공유 (파이에서 apt/pip 쓰려면 필요)"
sudo sysctl -w net.ipv4.ip_forward=1 >/dev/null
# 중복 추가 방지: 이미 있으면 건너뛴다
if ! sudo iptables -t nat -C POSTROUTING -o "$WIFI" -j MASQUERADE 2>/dev/null; then
  sudo iptables -t nat -A POSTROUTING -o "$WIFI" -j MASQUERADE
  echo "  NAT 규칙 추가"
else
  echo "  NAT 규칙 이미 있음"
fi

echo "[4/4] DHCP 서버(dnsmasq)"
if pgrep -x dnsmasq >/dev/null; then
  echo "  이미 실행 중"
else
  sudo dnsmasq --interface="$ETH" --bind-interfaces --except-interface=lo \
      --dhcp-range=192.168.99.2,192.168.99.10,255.255.255.0,24h \
      --dhcp-option=3,"$HOST_IP" --dhcp-option=6,"$HOST_IP" --log-dhcp
  echo "  기동 (백그라운드)"
fi

echo
echo "파이가 IP 받을 때까지 20초 대기..."
for i in $(seq 1 20); do
  ping -c1 -W1 "$PI_IP" &>/dev/null && break
  sleep 1
done
echo
check
