# monitoring

MAVLink UDP(127.0.0.1:14550)에서 배터리/위치 상태를 읽어 tmux 상태바에 표시하기 위한
간단한 데몬과 상태 문자열 제공 도구입니다.

## 구성
- `src/apps/monitoringd/monitoringd.c`: 배터리 + GPS 위성 수를 계속 수신해 `/tmp/monitoring_last`에 저장하는 데몬
- `src/apps/monitoring/monitoring.c`: 한 번만 읽어 상태 문자열을 출력하는 유틸리티

## 빌드
```bash
cd ~/mavlink_projects
make
make install
```

## 실행
### 데몬 실행
```bash
nohup ~/Tools/monitoringd >/dev/null 2>&1 &
```

### systemd user 서비스 (자동 실행)
```bash
systemctl --user daemon-reload
systemctl --user enable --now monitoringd.service
```

서비스 파일 위치:
`~/.config/systemd/user/monitoringd.service`

## tmux 연동
`~/.config/tmux/tmux.conf.local`에 다음 함수가 있어야 합니다.
```bash
# # usage: #{monitoring}
# monitoring() {
#   cat /tmp/monitoring_last 2>/dev/null || printf 'BATT:-- GPS:--'
# }
```
그리고 상태바에 `#{monitoring}`를 추가합니다.

## 출력 예시
- `BATT=85% GPS_SATS=14`
- `BATT=11.8V GPS_SATS=10`
- 데이터 없음: `BATT=NA GPS_SATS=NA`

## 참고
- `/tmp`는 대부분 tmpfs(RAM)로 동작합니다.
- 배터리 퍼센트는 `SYS_STATUS.battery_remaining` 또는 `BATTERY_STATUS.battery_remaining`를 사용합니다.
- `monitoringd`는 `SYSTEM_TIME`을 구독하며 로컬 시간과 60초 이상 차이나면 시스템 시간 동기화를 시도합니다.
- 시간 동기화에는 권한이 필요합니다(`root` 또는 `CAP_SYS_TIME`).
