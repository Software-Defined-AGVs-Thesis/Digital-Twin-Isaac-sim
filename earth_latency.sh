#!/bin/bash
# Usage: ./earth_latency.sh moon | none | custom <ms>
# Run on Earth PC to delay outbound commands to EC2

EC2_IP=10.10.8.159
UDP_PORT=5005
IFACE=$(ip route get $EC2_IP | awk '{for(i=1;i<=NF;i++) if ($i=="dev") print $(i+1)}')

clear_latency() {
  sudo tc qdisc del dev $IFACE root 2>/dev/null || true
  echo "[$IFACE] Earth-side latency cleared."
}

apply_latency() {
  local delay=$1
  local jitter=$2
  local label=$3

  clear_latency
  sudo tc qdisc add dev $IFACE root handle 1: prio
  sudo tc qdisc add dev $IFACE parent 1:3 handle 30: \
    netem delay ${delay}ms ${jitter}ms distribution normal
  sudo tc filter add dev $IFACE protocol ip parent 1:0 prio 3 u32 \
    match ip dst $EC2_IP/32 \
    match ip dport $UDP_PORT 0xffff \
    flowid 1:3

  echo "[$label] ${delay}ms ± ${jitter}ms on UDP $UDP_PORT → $EC2_IP ($IFACE)"
}

case "$1" in
  moon)   apply_latency 1300 50 "Moon" ;;
  none)   clear_latency ;;
  custom) 
    if [ -z "$2" ]; then
      echo "Usage: $0 custom <delay_ms>"
      exit 1
    fi
    apply_latency $2 50 "Custom" ;;
  *)      
    echo "Usage: $0 moon | none | custom <ms>"
    exit 1 ;;
esac