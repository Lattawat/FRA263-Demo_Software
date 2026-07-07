#!/usr/bin/env bash
# make_pair_env.sh — generate a per-pair environment file for multi-pair LAN isolation.
#
# A "pair" = one robot + one verifier (1 or 2 machines). Every terminal on each
# machine the pair spans must `source` the generated file so all three network
# layers (ROS 2 domain, LSL stream suffix, web ports) stay consistent.
#
# Usage:
#   scripts/make_pair_env.sh <pair_id 0-101>
#   source pairs/pair<N>.env       # on each machine the pair spans
#
# Derives, from a single pair id N:
#   ROS_DOMAIN_ID = N    isolates the verifier's ROS graph from other verifiers (range 0-101)
#   CV_PAIR_ID    = N    LSL stream suffix "_N" — the cross-machine isolator
# The launch file derives ws_port/http_port from CV_PAIR_ID (9000+N / 8000+N,
# or the legacy 9090/8000 when N=0).

set -euo pipefail

if [ $# -ne 1 ]; then
    echo "usage: $0 <pair_id 0-101>" >&2
    exit 1
fi

N="$1"
if ! [[ "$N" =~ ^[0-9]+$ ]] || [ "$N" -gt 101 ]; then
    echo "error: pair_id must be an integer in 0-101 (got '$N')" >&2
    exit 1
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="$ROOT/pairs"
mkdir -p "$OUT_DIR"
OUT="$OUT_DIR/pair${N}.env"

cat > "$OUT" <<EOF
# Pair $N environment — source on EACH machine this pair spans.
#   Verifier machine: ROS_DOMAIN_ID (ROS graph isolation) + CV_PAIR_ID (LSL suffix) + derived ports.
#   Robot machine:    CV_PAIR_ID (LSL suffix) only.
export ROS_DOMAIN_ID=$N
export CV_PAIR_ID=$N
EOF

echo "wrote $OUT"
