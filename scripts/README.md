# scripts

## install_rocket_mav.sh

프로그램 실행에 필요한 빌드/설치/설정 작업을 자동화합니다.

```bash
cd ~/mavlink_projects
chmod +x scripts/install_rocket_mav.sh
./scripts/install_rocket_mav.sh
```

옵션 예시:

```bash
./scripts/install_rocket_mav.sh \
  --install-system-config \
  --install-router-sample \
  --enable-monitoring-service
```

## package_release.sh

배포용 압축 파일을 생성합니다.

```bash
cd ~/mavlink_projects
chmod +x scripts/package_release.sh
./scripts/package_release.sh
```

생성 위치:
- `dist/rocket-mav-<timestamp>.tar.gz`
- `dist/rocket-mav-<timestamp>.tar.gz.sha256`
