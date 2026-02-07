# mav_batt

MAVLink UDP(127.0.0.1:14550)에서 배터리 상태를 읽어 tmux 상태바에 표시하기 위한
간단한 데몬과 상태 문자열 제공 도구입니다.

## 구성
- `mav_battd.c`: MAVLink 배터리 상태를 계속 수신해 `/tmp/mav_batt_last`에 저장하는 데몬
- `mav_batt.c`: (옵션) 한 번만 읽어 상태 문자열을 출력하는 유틸리티

## 빌드
```bash
cd ~/mavlink_projects/mav_batt

gcc mav_battd.c -I ../mavlink_lib/common/ -o ~/Tools/mav_battd
# 옵션 유틸리티
# gcc mav_batt.c  -I ../mavlink_lib/common/ -o ~/Tools/mav_batt
```

## 실행
### 데몬 실행
```bash
nohup ~/Tools/mav_battd >/dev/null 2>&1 &
```

### systemd user 서비스 (자동 실행)
```bash
systemctl --user daemon-reload
systemctl --user enable --now mav_battd.service
```

서비스 파일 위치:
`~/.config/systemd/user/mav_battd.service`

## tmux 연동
`~/.config/tmux/tmux.conf.local`에 다음 함수가 있어야 합니다.
```bash
# # usage: #{mav_batt}
# mav_batt() {
#   cat /tmp/mav_batt_last 2>/dev/null || printf 'BATT:--'
# }
```
그리고 상태바에 `#{mav_batt}`를 추가합니다.

## 출력 예시
- `BATT:85%`
- `BATT:11.8V`
- 데이터 없음: `BATT:--`

## 참고
- `/tmp`는 대부분 tmpfs(RAM)로 동작합니다.
- 배터리 퍼센트는 `SYS_STATUS.battery_remaining` 또는 `BATTERY_STATUS.battery_remaining`를 사용합니다.
