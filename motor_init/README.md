# motor_init

MAVLink 액추에이터 제어(`MAV_CMD_DO_SET_ACTUATOR`) 기반 모터 초기화/테스트 도구입니다.
모터 1, 2, ... 순서로 -1..1 신호를 출력하며, Enter로 다음 모터로 넘어갑니다.

## 입력 파라미터
`config.params`와 airframe 파일 문법을 지원합니다.

`config.params` 자동 탐색 순서:
- `/tmp/config.params`
- `~/mavlink_projects/scan/config.params`

MAVLink 통신 기본값은 공통 설정에서 읽습니다.
- `~/.config/rocket-mav/ports.env`
- `/etc/rocket-mav/ports.env`
- 우선순위: 실행 환경변수 > 사용자 설정 > 시스템 설정 > 코드 기본값

## 빌드
```bash
cd ~/mavlink_projects
make build
make install
```

## 실행
```bash
~/Tools/motor_init
```

옵션 예:
```bash
~/Tools/motor_init \
  -c /tmp/config.params \
  -a /home/rocket/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13200_generic_vtol_tailsitter \
  -n 4
```

## 옵션
- `-n <count>`: 모터 개수 강제 지정
- `-u <ip>`: 대상 IP (기본: `ROCKET_MAV_TOOLS_TARGET_IP` 또는 `127.0.0.1`)
- `-p <port>`: 대상 포트 (기본: `ROCKET_MAV_TOOLS_TARGET_PORT` 또는 `14550`)
- `-s <sysid>`: 송신 sysid (기본: `ROCKET_MAV_TOOLS_SYSID` 또는 `255`)
- `-c2 <compid>`: 송신 compid (기본: `ROCKET_MAV_TOOLS_COMPID` 또는 `190`)
- `-t <tsys>`: 대상 sysid (기본: `ROCKET_MAV_TOOLS_TARGET_SYS` 또는 `1`)
- `-k <tcomp>`: 대상 compid (기본: `ROCKET_MAV_TOOLS_TARGET_COMP` 또는 `1`)

## 동작
- 시작 시 강제 ARM
- 모터별 출력 테스트
- 종료 시 모든 출력 `0`으로 리셋 후 강제 DISARM
