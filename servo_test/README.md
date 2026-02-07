# servo_test

MAVLink 액추에이터 제어 명령(`MAV_CMD_DO_SET_ACTUATOR`)을 사용해 Pixhawk의 서보 출력(Servo 1, 2, …)을
순서대로 테스트하는 도구입니다. Enter를 누를 때마다 다음 서보로 넘어가며,
각 서보는 1초 주기의 sine 파형으로 -1..1 범위를 왕복합니다.

## 입력 파라미터
`config.params`와 airframe 파일의 문법을 지원합니다. 둘 다 없으면 기본값으로 동작합니다.
`config.params`는 아래 순서로 자동 탐색합니다.
- `/tmp/config.params`
- `~/mavlink_projects/scan/config.params`
- 예: `param set-default NAME VALUE`
- 예: `NAME=VALUE`
- 예: `NAME VALUE`

MAVLink 통신 기본값은 공통 설정에서 읽습니다.
- `~/.config/rocket-mav/ports.env`
- `/etc/rocket-mav/ports.env`
- 우선순위: 실행 환경변수 > 사용자 설정 > 시스템 설정 > 코드 기본값

서보 개수는 다음 순서로 감지합니다.
1. `CA_SV_COUNT`
2. `CA_SV_CS_COUNT`
3. `PWM_MAIN_MINx/MAXx`, `PWM_AUX_MINx/MAXx`
4. 감지 실패 시 기본값 `8`

## 빌드
```bash
cd ~/mavlink_projects
make build
make install
```

## 실행
```bash
~/Tools/servo_test \
  -c /path/to/config.params \
  -a /home/rocket/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13200_generic_vtol_tailsitter
```

파라미터 없이 실행(기본값 사용):
```bash
~/Tools/servo_test
```

## 옵션
- `-n <count>`: 서보 개수 강제 지정
- `-u <ip>`: 대상 IP (기본: `ROCKET_MAV_TOOLS_TARGET_IP` 또는 `127.0.0.1`)
- `-p <port>`: 대상 포트 (기본: `ROCKET_MAV_TOOLS_TARGET_PORT` 또는 `14550`)
- `-s <sysid>`: 송신 sysid (기본: `ROCKET_MAV_TOOLS_SYSID` 또는 `255`)
- `-c2 <compid>`: 송신 compid (기본: `ROCKET_MAV_TOOLS_COMPID` 또는 `190`)
- `-t <tsys>`: 대상 sysid (기본: `ROCKET_MAV_TOOLS_TARGET_SYS` 또는 `1`)
- `-k <tcomp>`: 대상 compid (기본: `ROCKET_MAV_TOOLS_TARGET_COMP` 또는 `1`)

## 매핑 참고
- `MAV_CMD_DO_SET_ACTUATOR`는 6채널 단위로 동작합니다.
- Servo N은 `set_index = (N-1)/6`에 해당하며, 나머지 채널은 `NaN`으로 무시됩니다.
- PX4에서 해당 actuator가 UAVCAN 출력으로 매핑되어 있어야 실제 장치가 동작합니다.

## 사용 방법
- 실행 시 강제로 ARM → 테스트 진행 → 종료 직전에 강제 DISARM 순서로 동작합니다.
- 실행 후 Enter를 누르면 다음 서보로 넘어갑니다.
- `q` + Enter를 누르면 종료합니다.

## 출력 예
```
Servo init tool: target 127.0.0.1:14550 (tsys=1 tcomp=1)
Detected servo count: 2
Press Enter to advance servo, 'q' then Enter to quit.
Servo 1: publishing actuator value (-1..1)
Servo 1 value: +0.123
```
