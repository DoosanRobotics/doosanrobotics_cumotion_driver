
# Doosan ROS2 + Isaac ROS 통합 환경 구성 가이드
---

## 1. docker / scripts 폴더 배치

먼저, Doosan 통합에 필요한 다음 디렉토리들을
`ros2_ws/src/cumotion/` 에서 **Isaac ROS 워크스페이스의 상위 디렉토리(ISAAC_ROS_WS/../)** 로 복사합니다.

### 복사 대상 폴더

* `docker/`
* `scripts/`

### 복사 명령어 예시

```bash
cp -r ~/ros2_ws/src/cumotion/docker   ~/workspaces/
cp -r ~/ros2_ws/src/cumotion/scripts ~/workspaces/
```

### 복사 후 디렉토리 구조 예시

```
workspaces/
├── isaac_ros-dev/          # ISAAC_ROS_WS
├── docker/
└── scripts/
```

---

## 2. Isaac ROS CLI 설정 파일 생성

다음 명령으로 Isaac ROS CLI 설정 디렉토리를 생성합니다.

```bash
mkdir -p ~/workspaces/isaac_ros-dev/.isaac-ros-cli
```

그 다음 아래 내용을 포함한 `config.yaml` 파일을 생성합니다.

### `~/workspaces/isaac_ros-dev/.isaac-ros-cli/config.yaml`

```yaml
environment:
  mode: docker

docker:
  image:
    additional_image_keys:
      - doosan
```

### 설정 설명

이 설정은 Docker 이미지 빌드 시 다음 순서로 레이어가 결합되도록 합니다:

1. **noble** (기본 OS + CUDA)
2. **ros2_jazzy** (Isaac ROS ROS2 레이어)
3. **doosan** (사용자 정의 Doosan 통합 레이어)

---

## 3. Docker 이미지 생성

설정이 준비되면 다음 명령을 실행하여 통합된 Isaac+Doosan 개발 이미지를 생성합니다.

```bash
isaac-ros activate --build-local
```

### 실행 후 동작

* `noble → ros2_jazzy → doosan` 순으로 Dockerfile 레이어가 병합됨
* 새로운 통합 Docker 이미지가 자동 생성됨
* 컨테이너가 자동 실행되며
  `/usr/local/bin/scripts/entrypoint_additions` 내부 스크립트가 실행됨
* `00-build-doosan.user.sh` 스크립트가 자동 실행되어
  **Isaac ROS + Doosan ROS2 workspace 전체가 자동으로 colcon build 됨**

---
