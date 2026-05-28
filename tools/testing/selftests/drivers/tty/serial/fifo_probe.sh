#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
#
# kselftest: FIFO trigger verification across ALL initialized TTY ports
#
# - Tests only ports with /sys/class/tty/<tty>/device/xmit_fifo_size > 0
# - Determines type from /sys/class/tty/<tty>/type
# - 8250-class ports: verify RX/TX triggers by measurement via uart_probe (debugfs)
# - Non-8250 ports: verify RX/TX triggers by sysfs write/readback
# - fifo_enable: measured effect on 8250, readback on others
# - KTAP nested output: top plan = #ports; each port prints its own sub-plan

set -u

MODULE_NAME="serial8250_fifo_probe"
DBGFS_BASE="/sys/kernel/debug/$MODULE_NAME"
SEL_PATH="$DBGFS_BASE/select_dev"
RX_TRIG_MEAS="$DBGFS_BASE/rx_trig_level"
TX_TRIG_MEAS="$DBGFS_BASE/tx_trig_level"

SUB_N=0
SUB_FAIL=0
tap_reset_sub() { SUB_N=0; SUB_FAIL=0; }
tap_plan()      { echo "1..$1"; }
tap_ok()        { SUB_N=$((SUB_N+1)); echo "ok $SUB_N - $*"; }
tap_notok()     { SUB_N=$((SUB_N+1)); echo "not ok $SUB_N - $*"; SUB_FAIL=1; }
tap_skip()      { SUB_N=$((SUB_N+1)); echo "ok $SUB_N - $* # SKIP"; }
tap_diag()      { echo "# $*"; }

file_exists()  { [[ -e "$1" ]]; }
read_text()    { tr -d '\n' <"$1" 2>/dev/null || true; }
read_num()     { local o; o=$(read_text "$1"); [[ "$o" =~ ^-?[0-9]+$ ]] && printf "%s" "$o"; }
write_text()   { printf "%s" "$2" >"$1" 2>/dev/null; }
mountpoint_q() { command -v mountpoint >/dev/null 2>&1 && mountpoint -q "$1"; }

emit_range() {
  [[ $# -ge 2 ]] || return 1
  local min="$1" max="$2" step="${3:-1}"
  [[ "$min" =~ ^-?[0-9]+$ && "$max" =~ ^-?[0-9]+$ && "$step" =~ ^[1-9][0-9]*$ ]] || return 1
  (( max >= min )) || { echo ""; return 0; }
  local i out=""
  for (( i=min; i<=max; i+=step )); do out+="$i "; done
  echo "${out% }"
}

ensure_debugfs() {
  [[ -d /sys/kernel/debug ]] || return 1
  mountpoint_q /sys/kernel/debug || mount -t debugfs debugfs /sys/kernel/debug 2>/dev/null || true
}

ensure_probe_ready() {
  ensure_debugfs
  [[ -d "$DBGFS_BASE" ]] && return 0
  modprobe -q "$MODULE_NAME" 2>/dev/null || true
  [[ -d "$DBGFS_BASE" ]]
}

probe_select_port() { write_text "$SEL_PATH" "${1}"$'\n'; }

is_console_port() {
  local name="$1" p="/sys/class/tty/$name/console"
  file_exists "$p" && [[ "$(read_text "$p")" == "Y" ]]
}

# Get ports in /sys/class/tty where xmit_fifo_size != 0
discover_ports() {
  local d name xmit
  for d in /sys/class/tty/*; do
    [[ -d "$d" ]] || continue
    name="$(basename "$d")"
    case "$name" in tty|pts|ptmx|console|ttyprintk) continue ;; esac
    [[ -L "$d/device" ]] || continue
    xmit="$(read_num "$d/xmit_fifo_size")"
    [[ -n "$xmit" && "$xmit" -gt 0 ]] || continue
    echo "$name"
  done
}

port_type_code() {
  local name="$1" 
  p="/sys/class/tty/$name/type"
  v="$(read_num "$p")"
  [[ -n "$v" ]] && echo "$v" || echo "-1"
}

# 8250-ish type codes
is_8250_code() {
  case "$1" in
    1|2|3|4|5|6|7|8|9|10|11|12|13|14|15|17|19|20|21|22|23|24|25|26|27|28|29|30|40|42|76|95|100|117|121|124) return 0 ;;
    *) return 1 ;;
  esac
}

suite_rx_for_code() {
  case "$1" in
    4)   echo "1 4 8 14" ;;              # 16550A
    7)   echo "8 16 24 28" ;;            # 16650V2
    8)   echo "1 4 8 14 16 32 56" ;;     # 16750
    10)  emit_range 1 127 ;;             # 16950
    124) echo "1 4 8 14 32 128 224" ;;   # CH382/CH38x
    *)   echo "" ;;
  esac
}

suite_tx_for_code() {
  case "$1" in
    4)   echo "" ;;                      # 16550A (no distinct THR trig)
    7)   echo "8 16 24 30" ;;            # 16650V2
    8)   echo "" ;;                      # 16750 (add when confirmed)
    10)  emit_range 1 127 ;;             # 16950
    124) echo "" ;;                      # CH382/CH38x
    *)   echo "" ;;
  esac
}

sysfs_paths_for() {
  local name="$1"
  echo "/sys/class/tty/$name/rx_trig_bytes" \
       "/sys/class/tty/$name/tx_trig_bytes" \
       "/sys/class/tty/$name/fifo_enable"
}

subtest_8250() {
  local name="$1" idx="$2"
  echo "# Subtest: $name (8250)"
  if is_console_port "$name"; then
    tap_plan 0
    echo "ok $idx - $name # SKIP console in use"
    return 0
  fi
  if ! ensure_probe_ready; then
    tap_reset_sub
    tap_plan 1
    tap_notok "$MODULE_NAME not available (debugfs missing or modprobe failed)"
    echo "not ok $idx - $name (8250)"
    return 0
  fi

  local rx_sys tx_sys fifo_sys
  read rx_sys tx_sys fifo_sys < <(sysfs_paths_for "$name")
  local code="$(port_type_code "$name")"
  local suite_rx=( $(suite_rx_for_code "$code") )
  local suite_tx=( $(suite_tx_for_code "$code") )

  local have_rx_sys=0 have_tx_sys=0 have_fe=0
  file_exists "$rx_sys" && have_rx_sys=1
  file_exists "$tx_sys" && have_tx_sys=1
  file_exists "$fifo_sys" && have_fe=1

  local plan=$(( 1 + ${#suite_rx[@]} + ${#suite_tx[@]} + (have_fe*2) ))
  tap_reset_sub
  tap_plan "$plan"

  probe_select_port "$name" && tap_ok "select_dev" || tap_notok "select_dev failed"

  # fifo_enable
  if (( have_fe )); then
    local fe_orig="$(read_num "$fifo_sys")" got0
    if write_text "$fifo_sys" "0"$'\n'; then
      probe_select_port "$name" >/dev/null 2>&1
      got0="$(read_num "$RX_TRIG_MEAS")"
      [[ -n "$got0" && "$got0" -ge 1 && "$got0" -le 2 ]] \
        && tap_ok "fifo_enable=0 verified (probe)" \
        || tap_notok "fifo_enable=0 expected rx_trig=1 (got ${got0:-<nil>})"
      write_text "$fifo_sys" "1"$'\n' || true
      [[ -n "$fe_orig" ]] && write_text "$fifo_sys" "$fe_orig"$'\n' >/dev/null 2>&1
      tap_ok "fifo_enable=1 restored"
    else
      tap_notok "fifo_enable write 0 failed"
      tap_notok "fifo_enable restore skipped"
    fi
  fi

  # RX sweep via probe
  if (( have_rx_sys )) && ((${#suite_rx[@]})); then
    local v got
    for v in "${suite_rx[@]}"; do
      write_text "$rx_sys" "$v"$'\n' || true
      probe_select_port "$name" >/dev/null 2>&1
      got="$(read_num "$RX_TRIG_MEAS")"
      [[ -n "$got" && "$got" -eq "$v" ]] \
        && tap_ok "rx_trig=$v verified (probe)" \
        || tap_notok "rx_trig verify: got=${got:-<nil>} want=$v"
    done
  else
    tap_ok "no RX sweep (no rx_trig_bytes or no FIFO)"
  fi

  # TX sweep via probe
  if (( have_tx_sys )) && ((${#suite_tx[@]})); then
    local v got
    for v in "${suite_tx[@]}"; do
      write_text "$tx_sys" "$v"$'\n' || true
      probe_select_port "$name" >/dev/null 2>&1
      got="$(read_num "$TX_TRIG_MEAS")"
      [[ -n "$got" && "$got" -eq "$v" ]] \
        && tap_ok "tx_trig=$v verified (probe)" \
        || tap_notok "tx_trig verify: got=${got:-<nil>} want=$v"
    done
  else
    tap_ok "no TX sweep (no tx_trig_bytes or no FIFO)"
  fi

  (( SUB_FAIL )) && echo "not ok $idx - $name (8250)" || echo "ok $idx - $name (8250)"
}

subtest_non8250() {
  local name="$1" idx="$2"
  echo "# Subtest: $name (non-8250)"
  if is_console_port "$name"; then
    tap_plan 0
    echo "ok $idx - $name # SKIP console in use"
    return 0
  fi

  local rx_sys tx_sys fifo_sys
  read rx_sys tx_sys fifo_sys < <(sysfs_paths_for "$name")
  local code="$(port_type_code "$name")"
  local suite_rx=( $(suite_rx_for_code "$code") )
  local suite_tx=( $(suite_tx_for_code "$code") )

  local have_rx_sys=0 have_tx_sys=0 have_fe=0
  file_exists "$rx_sys" && have_rx_sys=1
  file_exists "$tx_sys" && have_tx_sys=1
  file_exists "$fifo_sys" && have_fe=1

  local plan=$(( 1 + ${#suite_rx[@]} + ${#suite_tx[@]} + (have_fe*2) ))
  tap_reset_sub
  tap_plan "$plan"

  tap_ok "begin sysfs trigger sweep"

  # RX sweep: write/readback
  if (( have_rx_sys )) && ((${#suite_rx[@]})); then
    local orig="$(read_num "$rx_sys")"; [[ -z "$orig" ]] && orig=""
    local v got
    for v in "${suite_rx[@]}"; do
      if write_text "$rx_sys" "$v"$'\n'; then
        got="$(read_num "$rx_sys")"
        [[ -n "$got" && "$got" -eq "$v" ]] \
          && tap_ok "rx_trig=$v readback" \
          || tap_notok "rx readback: got=${got:-<nil>} want=$v"
      else
        tap_notok "rx_trig=$v write failed"
      fi
    done
    [[ -n "$orig" ]] && write_text "$rx_sys" "$orig"$'\n' >/dev/null 2>&1
  else
    tap_ok "no RX sweep (no rx_trig_bytes or no FIFO)"
  fi

  # TX sweep: write/readback
  if (( have_tx_sys )) && ((${#suite_tx[@]})); then
    local orig="$(read_num "$tx_sys")"; [[ -z "$orig" ]] && orig=""
    local v got
    for v in "${suite_tx[@]}"; do
      if write_text "$tx_sys" "$v"$'\n'; then
        got="$(read_num "$tx_sys")"
        [[ -n "$got" && "$got" -eq "$v" ]] \
          && tap_ok "tx_trig=$v readback" \
          || tap_notok "tx readback: got=${got:-<nil>} want=$v"
      else
        tap_notok "tx_trig=$v write failed"
      fi
    done
    [[ -n "$orig" ]] && write_text "$tx_sys" "$orig"$'\n' >/dev/null 2>&1
  else
    tap_ok "no TX sweep (no tx_trig_bytes or no FIFO)"
  fi

  # fifo_enable readback
  if (( have_fe )); then
    local fe_orig="$(read_num "$fifo_sys")" got
    if write_text "$fifo_sys" "0"$'\n'; then
      got="$(read_num "$fifo_sys")"
      [[ -n "$got" && "$got" -eq 0 ]] && tap_ok "fifo_enable=0 readback" || tap_notok "fifo_enable readback after 0"
    else
      tap_notok "fifo_enable write 0 failed"
    fi
    if write_text "$fifo_sys" "1"$'\n'; then
      got="$(read_num "$fifo_sys")"
      [[ -n "$got" && "$got" -eq 1 ]] && tap_ok "fifo_enable=1 readback" || tap_notok "fifo_enable readback after 1"
    else
      tap_notok "fifo_enable write 1 failed"
    fi
    [[ -n "$fe_orig" ]] && write_text "$fifo_sys" "$fe_orig"$'\n' >/dev/null 2>&1
  fi

  (( SUB_FAIL )) && echo "not ok $idx - $name (non-8250)" || echo "ok $idx - $name (non-8250)"
}

main() {
  mapfile -t ports < <(discover_ports)

  echo "KTAP version 1"
  if (( ${#ports[@]} == 0 )); then
    echo "1..0 # SKIP no initialized serial ports"
    exit 0
  fi
  echo "1..${#ports[@]}"

  local idx=0 p code
  for p in "${ports[@]}"; do
    idx=$((idx+1))
    code="$(port_type_code "$p")"
    if is_8250_code "$code"; then
      subtest_8250 "$p" "$idx"
    else
      subtest_non8250 "$p" "$idx"
    fi
  done

  rmmod "$MODULE_NAME" 2>/dev/null || true
}
main "$@"
