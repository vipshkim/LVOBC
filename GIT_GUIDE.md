# Git Guide (mavlink_projects)

이 워크스페이스는 외부 upstream 저장소(`mavlink`, `mavlink-router`)와
로컬 앱 코드가 함께 있으므로, 아래 방식으로 분리 관리하는 것을 권장합니다.

## 1) 추천 관리 대상

직접 개발 코드만 추적:
- `mav_batt/`
- `scan/`
- `servo_test/`
- `motor_init/`
- `rocket_mav_common/`
- `config/`
- `README.md`
- `GIT_GUIDE.md`

외부 저장소는 추적 제외:
- `mavlink/`
- `mavlink-router/`
- `mavlink_lib/` (필요 시 별도 vendor 전략 사용)
- `mavlink_test/` (필요 시 별도 실험 저장소 권장)

## 2) 최초 초기화

```bash
cd ~/mavlink_projects
git init
```

## 3) 기본 `.gitignore`

```gitignore
# external/upstream repos
mavlink/
mavlink-router/
mavlink_lib/
mavlink_test/

# build/test artifacts
**/build/
**/*.o
**/*.a
**/*.so
**/*.out

# generated params/runtime cache
scan/config.params
dist/
**/.DS_Store
```

## 4) 첫 커밋 예시

```bash
cd ~/mavlink_projects
git add README.md GIT_GUIDE.md \
  mav_batt scan servo_test motor_init rocket_mav_common config \
  .gitignore
git commit -m "chore: organize rocket mavlink workspace and docs"
```

## 5) 브랜치 전략 (간단형)

- `main`: 안정 동작 기준
- 기능 작업: `feat/<name>`
- 긴급 수정: `fix/<name>`

예:

```bash
git checkout -b feat/param-sync-tuning
```

## 6) 일상 루틴

```bash
git status
git add <changed-files>
git commit -m "<type>: <summary>"
```

권장 prefix:
- `feat`: 기능
- `fix`: 버그 수정
- `refactor`: 구조 개선
- `docs`: 문서
- `chore`: 빌드/설정

## 7) 원격 저장소 연결

```bash
git remote add origin <your-repo-url>
git branch -M main
git push -u origin main
```

## 8) 시스템 파일 관리 주의

아래 파일은 시스템 경로라 직접 추적되지 않습니다.
- `/etc/mavlink-router/main.conf`
- `/etc/rocket-mav/ports.env`

현재 템플릿은 이미 `config/` 폴더에 포함되어 있습니다.
