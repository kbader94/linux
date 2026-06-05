#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
#
# drx_rtt_compare - run drx_rtt twice (off, on) and print a side-by-side
#                   comparison of the latency distributions.
#
# Usage:
#   drx_rtt_compare.sh [options] /dev/ttyXXX
#
# The script intentionally does NOT manipulate the system environment
# (CPU isolation, cpuidle states, stress-ng load). See README.md for
# the recommended setups; run them once around an invocation of this
# script.

set -u

PROG_DIR=$(dirname "$(readlink -f "$0")")
DRX_RTT=${DRX_RTT:-$PROG_DIR/drx_rtt}

BAUD=19200
ITERS=1000
WARMUP=10
LDISC=29

usage() {
	cat >&2 <<EOF
Usage: $0 [options] /dev/ttyXXX

Options:
  --baud=N         Baud rate (default: $BAUD)
  --iterations=N   Per-mode iteration count (default: $ITERS)
  --warmup=N       Warmup iterations per mode (default: $WARMUP)
  --ldisc=N        Ldisc number (default: $LDISC)
  --help           This help

The script runs '\$DRX_RTT --mode=off' followed by '\$DRX_RTT --mode=on'
and prints a difference table. Other environment variables:

  DRX_RTT          path to the drx_rtt binary (default: \$dirname/drx_rtt)
EOF
}

while [ $# -gt 0 ]; do
	case "$1" in
		--baud=*)       BAUD="${1#*=}"   ;;
		--iterations=*) ITERS="${1#*=}"  ;;
		--warmup=*)     WARMUP="${1#*=}" ;;
		--ldisc=*)      LDISC="${1#*=}"  ;;
		--help|-h)      usage; exit 0    ;;
		--)             shift; break     ;;
		-*)             usage; exit 1    ;;
		*)              break            ;;
	esac
	shift
done

if [ $# -lt 1 ]; then
	usage
	exit 1
fi
TTY="$1"

if [ ! -x "$DRX_RTT" ]; then
	echo "drx_rtt binary not found or not executable: $DRX_RTT" >&2
	echo "Build it with 'make' in this directory first." >&2
	exit 1
fi

if [ ! -c "$TTY" ]; then
	echo "Not a character device: $TTY" >&2
	exit 1
fi

if [ ! -c /dev/n_drx_test ]; then
	echo "/dev/n_drx_test not present. Load the module first:" >&2
	echo "  sudo insmod ./n_drx_test/n_drx_test.ko" >&2
	exit 1
fi

run_one() {
	local mode=$1
	"$DRX_RTT" --mode="$mode" --baud="$BAUD" --iterations="$ITERS" \
	           --warmup="$WARMUP" --ldisc="$LDISC" --output=csv "$TTY" \
	           | tail -1
}

echo "Running mode=off ..." >&2
OFF_CSV=$(run_one off)
echo "Running mode=on  ..." >&2
ON_CSV=$(run_one on)

if [ -z "$OFF_CSV" ] || [ -z "$ON_CSV" ]; then
	echo "Failed to collect both runs" >&2
	exit 1
fi

# csv schema:
#   mode,iterations,baud,min_us,p10_us,p50_us,p90_us,p99_us,max_us,mean_us
IFS=','

# shellcheck disable=SC2206
OFF=($OFF_CSV)
# shellcheck disable=SC2206
ON=($ON_CSV)

unset IFS

# Indices into the csv split:
#   0 mode  1 iterations  2 baud
#   3 min   4 p10  5 p50  6 p90  7 p99  8 max  9 mean
fmt() { printf "%10.2f" "$1"; }

awk_diff() {
	# $1 off, $2 on -> delta (signed), pct improvement
	awk -v a="$1" -v b="$2" 'BEGIN {
		d = a - b;
		p = (a > 0) ? (100.0 * d / a) : 0;
		printf "%10.2f  %6.1f%%", d, p;
	}'
}

cat <<EOF

Direct-RX RTT comparison ($TTY @ $BAUD baud, $ITERS iterations/mode):

Latency (us)    workqueue    direct-RX        delta   improvement
------------  -----------  -----------  -----------  ------------
EOF

printf "min          "; fmt "${OFF[3]}"; printf "  "; fmt "${ON[3]}"; printf "  "; awk_diff "${OFF[3]}" "${ON[3]}"; printf "\n"
printf "p10          "; fmt "${OFF[4]}"; printf "  "; fmt "${ON[4]}"; printf "  "; awk_diff "${OFF[4]}" "${ON[4]}"; printf "\n"
printf "p50          "; fmt "${OFF[5]}"; printf "  "; fmt "${ON[5]}"; printf "  "; awk_diff "${OFF[5]}" "${ON[5]}"; printf "\n"
printf "p90          "; fmt "${OFF[6]}"; printf "  "; fmt "${ON[6]}"; printf "  "; awk_diff "${OFF[6]}" "${ON[6]}"; printf "\n"
printf "p99          "; fmt "${OFF[7]}"; printf "  "; fmt "${ON[7]}"; printf "  "; awk_diff "${OFF[7]}" "${ON[7]}"; printf "\n"
printf "max          "; fmt "${OFF[8]}"; printf "  "; fmt "${ON[8]}"; printf "  "; awk_diff "${OFF[8]}" "${ON[8]}"; printf "\n"
printf "mean         "; fmt "${OFF[9]}"; printf "  "; fmt "${ON[9]}"; printf "  "; awk_diff "${OFF[9]}" "${ON[9]}"; printf "\n"
echo
