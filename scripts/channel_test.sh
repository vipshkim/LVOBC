#!/usr/bin/env bash
set -euo pipefail

DEST_IP="${DEST_IP:-127.0.0.1}"
DEST_PORT="${DEST_PORT:-14531}"
RC_COUNT="${RC_COUNT:-8}"
STEP="${STEP:-0.1}"
SEND_HZ="${SEND_HZ:-20}"

if [[ "$RC_COUNT" -lt 0 ]]; then RC_COUNT=0; fi
if [[ "$RC_COUNT" -gt 18 ]]; then RC_COUNT=18; fi

arm=0
# rc values: uint16 raw, default 65535(ignore)
rc=()
for ((i=0; i<RC_COUNT; i++)); do rc[i]=65535; done
# actuator values: default 0
act=(0 0 0 0 0 0 0 0)
act_count=8

sel=0  # 0=arm, 1..RC_COUNT for RC, RC_COUNT+1..RC_COUNT+8 for actuator
last_key="-"

TTY_DEV="/dev/tty"
cleanup() {
  if [ -t 0 ]; then
    stty sane
  elif [ -r "$TTY_DEV" ]; then
    stty sane <"$TTY_DEV"
  fi
  printf "\n"
}
trap cleanup EXIT

if [ -t 0 ]; then
  stty raw -echo
elif [ -r "$TTY_DEV" ]; then
  stty raw -echo <"$TTY_DEV"
fi

last_send=0
send_period=$(awk -v hz="$SEND_HZ" 'BEGIN{if(hz<=0)hz=10; print 1.0/hz}')

print_status() {
  printf "\r\033[K"
  printf "arm=%d | sel=%d | key=%s | RC[%d]=%s | ACT[%d]=%s" \
    "$arm" "$sel" "$last_key" "$RC_COUNT" "${rc[*]}" "$act_count" "${act[*]}"
}

send_packet() {
  local fmt="C C C"
  local rc_fmt=""
  for ((i=0; i<RC_COUNT; i++)); do rc_fmt+=" v"; done
  fmt+="$rc_fmt C f<8 C C"

  perl -e 'my ($fmt,@args)=@ARGV; binmode STDOUT; print pack($fmt, @args);' \
    "$fmt" 36 "$arm" "$RC_COUNT" \
    ${RC_COUNT:+"${rc[@]}"} \
    "$act_count" \
    "${act[0]}" "${act[1]}" "${act[2]}" "${act[3]}" "${act[4]}" "${act[5]}" "${act[6]}" "${act[7]}" \
    0 \
    10 \
  | nc -u -w0 "$DEST_IP" "$DEST_PORT" >/dev/null 2>&1 || true
}

handle_key() {
  local key="$1"
  last_key="$key"
  case "$key" in
    w|W) key="[A" ;;
    s|S) key="[B" ;;
    d|D) key="[C" ;;
    a|A) key="[D" ;;
    $'\x1b')
      # Arrow keys send ESC [ A/B/C/D
      local rest=""
      if read -rsn1 -t 0.02 rest; then
        if [[ "$rest" == "[" ]]; then
          local code=""
          read -rsn1 -t 0.02 code || return 0
          key="[$code"
          last_key="ESC[$code"
        else
          return 0
        fi
      else
        return 0
      fi
      case "$key" in
        "[A") # up
          if (( sel == 0 )); then
            arm=1
          elif (( sel >= 1 && sel <= RC_COUNT )); then
            local idx=$((sel-1))
            local step_pwm
            step_pwm=$(awk -v s="$STEP" 'BEGIN{v=int(s*1000+0.5); if(v<1)v=1; print v}')
            if (( rc[$idx] == 65535 )); then rc[$idx]=1500; fi
            rc[$idx]=$((rc[$idx] + step_pwm))
            if (( rc[$idx] > 2000 )); then rc[$idx]=2000; fi
          else
            local idx=$((sel-RC_COUNT-1))
            act[$idx]=$(awk -v v="${act[$idx]}" -v s="$STEP" 'BEGIN{v+=s; if(v>1)v=1; printf("%.3f",v)}')
          fi
          ;;
        "[B") # down
          if (( sel == 0 )); then
            arm=0
          elif (( sel >= 1 && sel <= RC_COUNT )); then
            local idx=$((sel-1))
            local step_pwm
            step_pwm=$(awk -v s="$STEP" 'BEGIN{v=int(s*1000+0.5); if(v<1)v=1; print v}')
            if (( rc[$idx] == 65535 )); then rc[$idx]=1500; fi
            rc[$idx]=$((rc[$idx] - step_pwm))
            if (( rc[$idx] < 1000 )); then rc[$idx]=1000; fi
          else
            local idx=$((sel-RC_COUNT-1))
            act[$idx]=$(awk -v v="${act[$idx]}" -v s="$STEP" 'BEGIN{v-=s; if(v<-1)v=-1; printf("%.3f",v)}')
          fi
          ;;
        "[C") # right
          if (( sel < RC_COUNT + 8 )); then sel=$((sel+1)); fi
          ;;
        "[D") # left
          if (( sel > 0 )); then sel=$((sel-1)); fi
          ;;
      esac
      ;;
    q|Q)
      exit 0
      ;;
  esac
  return 0
}

printf "channel_test: DEST=%s:%s RC_COUNT=%s STEP=%s SEND_HZ=%s\n" "$DEST_IP" "$DEST_PORT" "$RC_COUNT" "$STEP" "$SEND_HZ"
printf "\nKeys: Left/Right=select, Up/Down=adjust, WASD also supported, q=quit\n"
print_status

while true; do
  # read key
  if read -rsn1 -t 0.05 key </dev/tty; then
    handle_key "$key"
    print_status
  fi

  # send packet at rate
  now=$(date +%s.%N)
  if awk -v n="$now" -v last="$last_send" -v p="$send_period" 'BEGIN{exit !(n-last>=p)}'; then
    send_packet
    last_send="$now"
    print_status
  fi

  sleep 0.02
done
