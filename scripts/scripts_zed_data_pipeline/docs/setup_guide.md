# NuRec Pipeline 설정 가이드

ZED 카메라 ROS2 bag 데이터를 3D 메쉬로 변환하는 파이프라인 환경 설정 가이드입니다.

## 사전 요구사항

- Docker + Docker Compose
- NVIDIA GPU + [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- VS Code + [Dev Containers 확장](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- 디스크 여유 공간: 이미지 약 112GB + 데이터/출력 공간

---

## 1. Docker 이미지 저장 (최초 1회)

현재 컨테이너를 이미지로 커밋한 상태에서, 호스트 터미널에서 실행:

```bash
# tar.gz로 압축 저장 (시간 오래 걸리지만 용량 절약)
docker save nurec-pipeline:latest | gzip > /path/to/ssd/nurec-pipeline.tar.gz

# 또는 tar로 저장 (더 빠름)
docker save nurec-pipeline:latest -o /path/to/ssd/nurec-pipeline.tar
```

---

## 2. Docker 이미지 로드

```bash
# gz 압축인 경우
docker load -i /path/to/ssd/nurec-pipeline.tar.gz

# tar인 경우
docker load -i /path/to/ssd/nurec-pipeline.tar

# 로드 확인
docker images | grep nurec-pipeline
```

---

## 3. 프로젝트 폴더 준비

```bash
# 작업 디렉토리 생성
mkdir -p nurec-project/data nurec-project/output
cd nurec-project

# docker-compose.yml과 .devcontainer/ 가 필요합니다.
# 이미지 안에 이미 포함되어 있으므로, 아래 방법 중 하나로 꺼냅니다:

# 방법 1: 임시 컨테이너에서 복사
docker create --name tmp nurec-pipeline:latest
docker cp tmp:/workspace/scripts/docker/docker-compose.yml .
docker cp tmp:/workspace/scripts/.devcontainer .
docker rm tmp

# 방법 2: 직접 생성 (아래 내용 참고)
```

### docker-compose.yml

```yaml
services:
  nurec-pipeline:
    image: nurec-pipeline:latest
    container_name: nurec-pipeline
    stdin_open: true
    tty: true
    runtime: nvidia
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
    volumes:
      - ./data:/workspace/data
      - ./output:/workspace/output
    working_dir: /workspace
```

### .devcontainer/devcontainer.json

```json
{
  "name": "NuRec Pipeline",
  "dockerComposeFile": "../docker-compose.yml",
  "service": "nurec-pipeline",
  "workspaceFolder": "/workspace",
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-python.python",
        "ms-python.vscode-pylance"
      ],
      "settings": {
        "terminal.integrated.defaultProfile.linux": "bash"
      }
    }
  }
}
```

---

## 4. ZED 데이터 준비

ZED ROS2 bag 데이터를 `data/` 폴더에 넣습니다:

```bash
cp -r /path/to/zed_nurec_compressed_optimized_trial_3 ./data/
```

폴더 구조:
```
nurec-project/
├── docker-compose.yml
├── .devcontainer/
│   └── devcontainer.json
├── data/
│   └── zed_nurec_compressed_optimized_trial_3/
│       ├── metadata.yaml
│       └── zed_nurec_compressed_optimized_trial_3_0.db3
└── output/
```

---

## 5. VS Code에서 Dev Container 열기

1. VS Code에서 `nurec-project/` 폴더를 엽니다
2. `Ctrl+Shift+P` → **Dev Containers: Reopen in Container** 선택
3. 컨테이너가 시작되면 VS Code가 자동으로 컨테이너 내부에 연결됩니다

### 터미널에서 직접 실행할 경우

```bash
cd nurec-project
docker compose up -d
docker exec -it nurec-pipeline /bin/bash
```

---

## 6. 파이프라인 실행

컨테이너 내부 터미널에서:

```bash
bash /workspace/scripts/zed_pipeline/run_zed_pipeline.sh \
  /workspace/data/zed_nurec_compressed_optimized_trial_3 \
  /workspace/output/zed_nurec_compressed_optimized_trial_3 \
  --start-time 32 --end-time 100 --skip 4
```

### 파라미터 설명

| 파라미터 | 설명 | 예시 |
|---------|------|------|
| 첫 번째 인자 | ROS2 bag 경로 | `/workspace/data/zed_...` |
| 두 번째 인자 | 출력 경로 | `/workspace/output/zed_...` |
| `--start-time` | 시작 시간 (초) | `32` |
| `--end-time` | 끝 시간 (초) | `100` |
| `--skip` | N 프레임마다 처리 | `4` |

---

## 7. 결과 확인

파이프라인이 완료되면 `output/` 폴더에 결과가 생성됩니다:

```
output/zed_nurec_compressed_optimized_trial_3/
├── extracted/          # 추출된 스테레오 이미지
│   ├── zed_left/       # 왼쪽 카메라 이미지 (*.jpeg)
│   ├── zed_right/      # 오른쪽 카메라 이미지 (*.jpeg)
│   ├── frames_meta.json
│   └── stereo.edex
├── cusfm/              # 카메라 포즈 추정 결과
│   ├── sparse/         # 3D 포인트 클라우드
│   └── keyframes/      # 키프레임 메타데이터
├── depth/              # 깊이맵 (16-bit PNG, mm 단위)
│   └── zed_left/
└── nvblox/             # 최종 3D 메쉬
    ├── mesh.ply        # ← 최종 결과물
    └── occupancy_map.png
```

`mesh.ply` 파일은 MeshLab, CloudCompare, Blender 등에서 열 수 있습니다.

---

## 파이프라인 단계 요약

```
ROS2 bag (.db3)
  → Step 1: 스테레오 이미지 추출 (extract_zed_bag.py)
    → Step 2: 카메라 포즈 추정 (cusfm_cli)
      → Step 3: 깊이 추정 (FoundationStereo)
        → Step 4: 3D 메쉬 생성 (nvblox) → mesh.ply
```

## 트러블슈팅

### GPU를 인식하지 못할 때
```bash
# NVIDIA Container Toolkit 설치 확인
nvidia-container-cli info

# docker compose에서 runtime: nvidia 대신 deploy 설정 사용
# (docker-compose.yml에 이미 두 방식 모두 포함됨)
```

### 컨테이너 내부에서 GPU 확인
```bash
nvidia-smi
```

### 이미지 로드 시 디스크 공간 부족
```bash
# Docker 이미지 저장 위치 변경
# /etc/docker/daemon.json에서 data-root 설정
{
  "data-root": "/path/to/large/disk/docker"
}
sudo systemctl restart docker
```
