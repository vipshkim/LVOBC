# rocket MAVLink Workspace

이 문서는 `/home/rocket/mavlink_projects`의 전체 구조, 데이터 흐름, 실행 방법을 한 번에 정리한 운영 문서입니다.

## 1) 디렉터리 구조

- `src/`
  - 모든 앱 소스 (`apps/*`)
  - 공통 헤더 (`common/rocket_mav_common.h`)
- `build/`
  - 빌드 산출물(바이너리)
- `monitoring/`
  - 모니터링 문서
- `scan/`
  - 파라미터 스냅샷 보관 경로 (`scan/config.params`)
- `servo_test/`
  - 서보 테스트 문서
- `motor_init/`
  - 모터/엔진 초기화 문서
- `rocket_mav_common/`
  - 공통 설정 문서
- `archive/duplicate_outside_src/`
  - `src` 밖에 있던 과거 중복 소스/바이너리 백업
- `config/`
  - 시스템 설정 템플릿(`ports.env.sample`, `rate.env.sample`, `mavlink-router.main.conf.sample`)
- `mavlink_lib/`
  - C MAVLink 헤더 라이브러리
- `mavlink/`, `mavlink-router/`
  - upstream 저장소(외부 프로젝트)

## 2) 실행 파일 배치 정책

- 소스 프로젝트: `~/mavlink_projects/<project_name>`
- 빌드 결과물: `~/Tools/<program_name>`

현재 표준 빌드 예시(권장):

```bash
cd ~/mavlink_projects
make
make install
```

개별 바이너리만 만들고 싶다면:

```bash
cd ~/mavlink_projects
make build
```

## 3) 통신 구조 (router 중심)

기준 설정 파일: `/etc/mavlink-router/main.conf`

시리얼 입력:
- Pixhawk: `/dev/ttyAMA0 @ 921600`

Router -> App (Normal/fan-out):
- `14550`: `monitoring`
- `14551`: `mav_controller`
- `14552`: `mav_console`
- `14553`: `scan`

App -> Router (Server/ingress):
- `14650`: `monitoring_ingress`
- `14651`: `mav_controller_ingress`
- `14652`: `mav_console_ingress`
- `14653`: `scan_ingress`
- `14660`: 임시 도구(`servo_test`, `motor_init`)

## 4) 데이터 파일 정책

- 실시간/휘발(우선): `/tmp/config.params`
- 영구 백업: `~/mavlink_projects/scan/config.params`
- 모니터링 캐시: `/tmp/monitoring_last`

`scan` 실행 시 파라미터를 수신해 위 2개 경로에 저장합니다.

`servo_test`, `motor_init`은 기본적으로 아래 순서로 파라미터를 찾습니다.
1. `/tmp/config.params`
2. `~/mavlink_projects/scan/config.params`

## 5) 공통 설정 시스템

공통 키 로더 파일: `rocket_mav_common/rocket_mav_common.h`
소스 위치: `src/common/rocket_mav_common.h`

설정 우선순위:
1. 실행 환경변수 (`export KEY=VALUE`)
2. `~/.config/rocket-mav/ports.env`, `~/.config/rocket-mav/rate.env`
3. `/etc/rocket-mav/ports.env`, `/etc/rocket-mav/rate.env`
4. 코드 기본값

주요 키:
- `ROCKET_MAV_SCAN_LISTEN_IP`
- `ROCKET_MAV_SCAN_LISTEN_PORT`
- `ROCKET_MAV_SCAN_TARGET_IP`
- `ROCKET_MAV_SCAN_TARGET_PORT`
- `ROCKET_MAV_SCAN_SYSID`
- `ROCKET_MAV_SCAN_COMPID`
- `ROCKET_MAV_SCAN_TARGET_SYS`
- `ROCKET_MAV_SCAN_TARGET_COMP`
- `ROCKET_MAV_SCAN_SERIAL_BAUD`
- `ROCKET_MAV_TOOLS_TARGET_IP`
- `ROCKET_MAV_TOOLS_TARGET_PORT`
- `ROCKET_MAV_TOOLS_SYSID`
- `ROCKET_MAV_TOOLS_COMPID`
- `ROCKET_MAV_TOOLS_TARGET_SYS`
- `ROCKET_MAV_TOOLS_TARGET_COMP`
- `ROCKET_MAV_STREAM_ODOM_LISTEN_IP`
- `ROCKET_MAV_STREAM_ODOM_LISTEN_PORT`
- `ROCKET_MAV_STREAM_ODOM_OUT_IP`
- `ROCKET_MAV_STREAM_ODOM_OUT_PORT`
- `ROCKET_MAV_STREAM_ODOM_RATE_HZ`
- `ROCKET_MAV_STREAM_ODOM_STATE_PATH`
- `ROCKET_MAV_STREAM_CMD_LISTEN_IP`
- `ROCKET_MAV_STREAM_CMD_LISTEN_PORT`
- `ROCKET_MAV_STREAM_CMD_TARGET_IP`
- `ROCKET_MAV_STREAM_CMD_TARGET_PORT`
- `ROCKET_MAV_STREAM_CMD_RATE_HZ`
- `ROCKET_MAV_STREAM_CMD_STATE_PATH`
- `ROCKET_MAV_PARAMS_TMP_PATH`
- `ROCKET_MAV_PARAMS_PERSIST_PATH`
- `ROCKET_MAV_SCAN_SERIAL_BAUD`
- `ROCKET_MAV_TOOLS_SERIAL_BAUD`
- `ROCKET_MAV_STREAM_ODOM_RATE_HZ`
- `ROCKET_MAV_STREAM_CMD_RATE_HZ`
- `ROCKET_MAV_STREAM_ODOM_TEST_CONSOLE_BPS`
- `ROCKET_MAV_STREAM_CMD_TEST_CONSOLE_BPS`
- `ROCKET_MAV_SERVO_LOOP_HZ`
- `ROCKET_MAV_MOTOR_LOOP_HZ`

## 6) 프로그램별 역할 요약

### `scan` (`src/apps/scan/scan.c`)
- HEARTBEAT/토픽 스캔
- `PARAM_REQUEST_LIST` + 누락 index 재요청(`PARAM_REQUEST_READ`)
- 결과 저장: `/tmp/config.params` + 영구 경로

### `servo_test` (`src/apps/servo_test/servo_test.c`)
- 강제 ARM -> 서보별 sine 테스트 -> 출력 0 리셋 -> 강제 DISARM
- Enter로 다음 채널, `q`로 종료
- 역할명은 config 파라미터(`PWM_MAIN_FUNCx`, `PWM_AUX_FUNCx`)로 표시

### `motor_init` (`src/apps/motor_init/motor_init.c`)
- `servo_test`와 동일한 전송 메커니즘
- 모터/엔진 초기화 절차 인터페이스용으로 분리 운영

### `monitoringd` (`src/apps/monitoringd/monitoringd.c`)
- 배터리 + GPS 위성 수 등 MAVLink 메시지 수신
- `key=value` 형식 문자열을 `/tmp/monitoring_last`에 지속 갱신
- `RDY` 정책:
  - HEARTBEAT 상태를 최우선으로 사용
  - `mode=TEST`에서는 `all_sensors_healthy` 부족만으로 `RDY`를 강등하지 않음
  - `mode!=TEST`에서만 `all_sensors_healthy` 부족 시 `RDY`를 노랑으로 강등
- 모드 캐시 갱신:
  - `mode.env` 변경 감지는 초 단위(`st_mtime`)가 아니라 나노초 단위(`st_mtim`)를 사용
  - 같은 초 내 모드 전환에서도 tmux 표시가 stale 되지 않도록 수정

### `stream_odometry` (`src/apps/stream_odometry/stream_odometry.c`)
- MAVLink `ODOMETRY(331)` 수신 후 `/tmp/stream_odometry.latest` 갱신
- 지정 주기(기본 `100Hz`)로 로컬 UDP 바이너리 스트림 송신
- 패킷 형식:
  - `'$'` + `float q[4]`
  - `float rollspeed, pitchspeed, yawspeed`
  - `float x, y, z, vx, vy, vz`
  - `uint8 quality` + `'\n'`

### `stream_commander` (`src/apps/stream_commander/stream_commander.c`)
- 로컬 UDP 명령 패킷 수신 후 MAVLink actuator 제어 송신
- `arm` 값이 바뀔 때만 arm/disarm 명령 1회 송신
- 모터/서보(1,2,3,4,5,6,7,8)는 지정 주기(기본 `100Hz`)로 반복 송신
- 상태 파일(`ROCKET_MAV_STREAM_CMD_STATE_PATH`)은 속도 우선의 바이너리 포맷(`RSCM`)으로 원자적 갱신
- 패킷 형식:
  - `'$'` + `uint8 arm`
  - `float motor1, motor2, motor3, motor4, servo5, servo6, servo7, servo8`
  - `'\n'`

### `stream_odometry_test` (`src/apps/stream_odometry/stream_odometry_test.c`)
- `stream_odometry` UDP 패킷을 파싱해서 콘솔 표시
- 콘솔 출력은 `5760 b/s` 제한(약 `720 B/s`)으로 과부하 방지

### `stream_commander_test` (`src/apps/stream_commander/stream_commander_test.c`)
- `stream_commander` 입력 패킷을 파싱해서 콘솔 표시
- 콘솔 출력은 `5760 b/s` 제한(약 `720 B/s`)으로 과부하 방지

## 7) 운영 기본 순서

1. `mavlink-router` 구동 확인
2. `scan` 실행으로 최신 파라미터 동기화
3. 필요한 도구(`servo_test`, `motor_init`, `monitoringd`) 실행

예:

```bash
~/Tools/scan -d 5 -D 20
~/Tools/servo_test
~/Tools/motor_init
~/Tools/stream_odometry -q 14554 -r 100
~/Tools/stream_commander -q 14654 -r 100
~/Tools/stream_odometry_test -p 14554
~/Tools/stream_commander_test -p 14654
```

스트림 데몬 자동실행 설정:

```bash
cd ~/mavlink_projects
./scripts/install_rocket_mav.sh \
  --enable-monitoring-service \
  --enable-stream-services
```

## 8) 트러블슈팅 핵심

- 파라미터가 안 오면:
  - 포트 매핑(`14553/14653`) 확인
  - router 점유/중복 수신 경로 확인
  - 필요시 `scan -S /dev/ttyAMA0 -B 921600` 직접 테스트
- `No parameters received`면:
  - 먼저 `scan`으로 `/tmp/config.params` 생성 확인
  - `~/.config/rocket-mav/ports.env` 값이 실제 router 설정과 일치하는지 확인

## 9) Git 정리 안내

Git 작업 가이드는 `GIT_GUIDE.md`를 따릅니다.

## 10) 설치/배포 자동화

설치 스크립트:
- `scripts/install_rocket_mav.sh`

배포 패키지 스크립트:
- `scripts/package_release.sh`

빠른 사용:

```bash
cd ~/mavlink_projects
./scripts/install_rocket_mav.sh
./scripts/package_release.sh
```

## 11) 새 프로젝트 추가 방법

기본 규칙:
- 소스: `src/apps/<app_name>/`
- 공통 헤더: `src/common/rocket_mav_common.h`
- 빌드 산출물: `build/bin/<app_name>`
- 설치 위치: `~/Tools/<app_name>`

새 앱 예시:

```bash
mkdir -p src/apps/my_tool
cat > src/apps/my_tool/my_tool.c <<'EOF'
#include <stdio.h>
int main(void) { puts("hello"); return 0; }
EOF
```

Makefile은 `src/apps/*`를 **자동 탐색**합니다.
새 앱을 추가하면 `make`만으로 자동 빌드됩니다.
