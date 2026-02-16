# Doosan E0509 Robot Arm Controller — ROS2

## 개요
ROS2(Humble) 기반 두산 E0509 로봇암 End-Effector 위치 제어 프로그램.
PyQt5 GUI를 통해 목표 좌표 입력, 절대/상대 좌표 선택, 속도/가속도 설정,
실시간 Joint 상태 및 위치 모니터링 기능을 제공합니다.

## 개발 환경
- OS: Ubuntu 22.04 (WSL2)
- ROS2: Humble
- Python: 3.10
- GUI: PyQt5
- 로봇: Doosan E0509
- 시뮬레이터: dsr_bringup2 (Docker 에뮬레이터)

## 패키지 구조
```
ros2_ws/
├── src/
│   └── my_ros2_assignment/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       │   └── my_ros2_assignment
│       └── my_ros2_assignment/
│           ├── __init__.py
│           └── my_node.py
├── readme.md
└── requirements.txt
```

## 설치 방법

### 1. ROS2 Humble 설치
```bash
sudo apt update && sudo apt install ros-humble-desktop -y
source /opt/ros/humble/setup.bash
```

### 2. doosan-robot2 설치
```bash
cd ~/ros2_ws/src
git clone -b humble-devel https://github.com/DoosanRobotics/doosan-robot2.git
cd ~/ros2_ws
colcon build
```

### 3. Docker 설치 (에뮬레이터용)
```bash
sudo apt install docker.io -y
sudo service docker start
sudo usermod -aG docker $USER
newgrp docker
```

### 4. PyQt5 설치
```bash
sudo apt install python3-pyqt5 fonts-noto-cjk -y
```

### 5. 패키지 빌드
```bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_assignment
source install/setup.bash
```

## 실행 방법

### 터미널 1 — 로봇 시뮬레이터
```bash
export ROS_DOMAIN_ID=0
source ~/ros2_ws/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py model:=e0509 mode:=virtual
```

### 터미널 2 — GUI 노드
```bash
export ROS_DOMAIN_ID=0
source ~/ros2_ws/install/setup.bash
ros2 run my_ros2_assignment my_node
```

## UI 구성
```
┌─────────────────────────────┬──────────────────────────────┐
│  📍 좌표 기준 설정           │  🔌 연결 / 동작 상태          │
│  절대좌표 / 상대좌표 선택    │  로봇 연결 상태 표시          │
├─────────────────────────────┤  동작 중 / 정지 상태 표시     │
│  🎯 목표지점 목록 [mm]       ├──────────────────────────────┤
│  X / Y / Z 좌표 입력        │  📌 현재 End-Effector 위치    │
│  목표지점 추가 / 삭제        │  X, Y, Z (mm)                │
├─────────────────────────────┼──────────────────────────────┤
│  ⚙️ 이동 파라미터            │  🔩 현재 Joint 각도 [°]      │
│  최대 속도 (m/s)            │  J1 ~ J6 실시간 표시          │
│  최대 가속도 (m/s²)         ├──────────────────────────────┤
├─────────────────────────────┤  📋 실시간 로그               │
│  ▶ 실행  ■ 정지             │  이동 진행 상황 표시          │
│  진행률 표시바               │                              │
└─────────────────────────────┴──────────────────────────────┘
```

## 주요 클래스 설명

| 클래스 | 설명 |
|--------|------|
| `DoosanRobotNode` | ROS2 Node. Joint 구독, FK 계산, MoveLine 서비스 호출 |
| `MotionWorker(QThread)` | 별도 스레드에서 이동 시퀀스 실행 |
| `TargetRow(QWidget)` | 목표지점 한 행 (X/Y/Z 입력 + 삭제 버튼) |
| `MainWindow(QMainWindow)` | 전체 GUI. 좌측(제어), 우측(상태) |

## 동작 로직
1. 사용자가 목표지점 (x,y,z) 입력
2. 절대/상대 좌표 선택
3. 속도/가속도 설정
4. 실행 버튼 클릭 → MotionWorker 스레드 시작
5. dsr_msgs2/srv/MoveLine 서비스 호출
6. RViz에서 로봇 실시간 이동 확인
7. Joint 각도 및 위치 실시간 GUI 업데이트

## 의존성
- ros-humble-rclpy
- ros-humble-sensor-msgs
- ros-humble-geometry-msgs
- ros-humble-trajectory-msgs
- python3-pyqt5
- fonts-noto-cjk
- docker.io
