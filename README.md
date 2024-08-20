
# Work Experience Project

이 프로젝트는 ROS 2 기반의 로봇 내비게이션 및 경로 추종 시스템을 구축하고, 특정 경유지로 이동하도록 명령을 보내는 것을 목표로 합니다. 로봇은 경로를 따라 이동하면서 특정 위치에 도달하면 LED를 제어하는 등의 추가 작업을 수행할 수 있습니다.

## Prerequisites

- ROS 2 Humble
- TurtleBot3 (또는 해당 로봇 모델)
- Gazebo Simulator
- Navigation2 패키지
- Cartographer 패키지

## Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   ```

2. Build the workspace:
   ```bash
   cd <workspace>
   colcon build --packages-select work_experience_project
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

4. Launch the necessary nodes:
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=~/map.yaml
   ```

5. Run the waypoints action client:
   ```bash
   ros2 run work_experience_project waypoints_action_client
   ```

## Steps Taken

### 1. Cartographer를 사용한 지도 생성
- Gazebo 시뮬레이터에서 로봇을 Teleop 키보드를 사용해 조작하여 Cartographer로 지도를 생성했습니다.
- 지도가 완성된 후, 다음 명령어로 지도를 저장했습니다:
  ```bash
  ros2 run nav2_map_server map_saver_cli -f ~/map
  ```

### 2. 경유지 설정 및 로봇 이동
- `waypoints_action_client` 노드를 설정하여 특정 경유지로 이동하도록 로봇을 제어했습니다.
- 설정한 경유지 좌표:
  - 첫 번째 경유지: (1.44, -0.158)
  - 두 번째 경유지: (1.43, 0.353)
  - 세 번째 경유지: (0.122, 0.465)
  - 마지막 경유지(원점): (0.0, 0.0)

### 3. Teleop을 통한 로봇 조작
- 로봇을 Teleop 키보드로 수동 제어하려 했으나, 로봇이 반응하지 않는 문제를 발견했습니다.
- 이 문제를 해결하기 위해 모든 ROS 2 노드를 종료하고, 다시 실행했습니다.

## 패키지 구조 및 코드 구성

### 1. 패키지 구조

`work_experience_project` 패키지는 ROS 2 노드와 CMake 구성 파일을 포함하고 있습니다. 주요 파일과 디렉토리 구조는 다음과 같습니다:

```
work_experience_project/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── waypoints_action_client.cpp
│   ├── main_control_node.cpp
│   └── waypoints_navigation_node.cpp
├── include/
│   └── work_experience_project/
│       └── (header files)
└── launch/
    └── (launch files)
```

### 2. 주요 노드 설명

#### `waypoints_action_client.cpp`

- **역할**: 
  - 로봇이 설정된 경유지로 이동하도록 명령을 보내는 노드입니다. 이동이 시작되면 LED를 켜고, 이동이 완료되면 LED를 끄는 기능을 제공합니다.

- **주요 함수**:
  - `move_to_waypoints`: 미리 설정된 경유지들로 로봇을 이동시키는 함수입니다. 각 경유지에 도달할 때마다 다음 경유지로 이동을 시도하며, 목표 도달 실패 시 로그를 남깁니다.
  - `navigate_to_pose`: `nav2_msgs::action::NavigateToPose` 액션을 사용해 특정 위치로 로봇을 이동시킵니다.
  - `control_led`: `std_srvs::srv::SetBool` 서비스를 사용해 LED를 켜거나 끄는 기능을 제공합니다.

- **경유지 좌표**:
  - 코드에서 설정된 경유지 좌표는 다음과 같습니다:
    - 첫 번째 경유지: `(1.44, -0.158)`
    - 두 번째 경유지: `(1.43, 0.353)`
    - 세 번째 경유지: `(0.122, 0.465)`
    - 마지막 경유지(원점): `(0.0, 0.0)`

#### `main_control_node.cpp`

- **역할**: 
  - 로봇의 상태를 관리하고, 카메라 피드를 실시간으로 표시하며, LED를 수동으로 제어할 수 있는 기능을 제공합니다.

- **주요 기능**:
  - `toggle_led`: 특정 조건에 따라 LED를 수동으로 켜거나 끄는 기능을 제공합니다.
  - `image_callback`: 카메라 피드에서 이미지를 수신하고 이를 처리하여 화면에 표시하는 기능을 제공합니다.

#### `waypoints_navigation_node.cpp`

- **역할**:
  - 로봇의 현재 위치를 주기적으로 퍼블리시하여 다른 노드들이 로봇의 상태를 알 수 있도록 합니다.

- **주요 기능**:
  - `publish_robot_state`: 로봇의 현재 상태(위치, 방향 등)를 퍼블리시하는 함수입니다. 주기적으로 호출되어 로봇의 상태를 다른 노드에 전달합니다.

## Troubleshooting

- **로봇이 Teleop에 반응하지 않는 경우**: 
  - `ros2 topic echo /cmd_vel`을 사용해 토픽이 정상적으로 퍼블리시되는지 확인하세요.
  - 모든 ROS 2 노드를 종료한 후, `pkill -f ros2` 명령어를 사용해 백그라운드에서 실행 중인 모든 ROS 2 프로세스를 종료하고 다시 실행해 보세요.

- **경유지 도달 실패**:
  - RViz에서 로봇의 경로와 위치를 확인해 장애물이 있는지 확인하세요.
  - 경유지 좌표를 약간 수정하여 다시 시도하세요.

## Conclusion & Future Work

현재 프로젝트는 ROS 2 내비게이션 및 경로 추종 기능을 성공적으로 구현했습니다. 앞으로 더 복잡한 경로 계획과 장애물 회피 기능을 추가할 계획입니다.
