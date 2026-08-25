# Ouster OS1-128 단일 라이다 물체 추적 — 설치 및 실행 가이드

새 PC에 처음부터 그대로 재현할 수 있도록 정리한 문서다.
설치 → 소스 패치 → 좌표계 보정 → 실행 → 문제 해결 순서로 되어 있다.

## 검증 환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu 22.04.5 LTS (커널 6.8.0) |
| ROS | ROS 2 Humble |
| 센서 | Ouster OS-1-128 (`OS1-070-128U-AX`), 펌웨어 `v3.1.0` |
| ouster-ros | `0.15.0` (커밋 `aef1a24`) |
| ouster-sdk | `v0.16.2-6-g401c647` (서브모듈) |
| PC IP | `192.168.6.100/24` (유선 `enp59s0`) |
| 센서 IP | `192.168.6.11` |

> 이 문서의 ROI 값과 `yaw` 는 **특정 설치 위치에서 실측한 값**이다.
> 다른 장소에서는 [7. 새 위치에서 재보정](#7-새-위치에서-재보정) 을 반드시 다시 수행해야 한다.

---

## 1. 사전 준비

### 1.1 ROS 2 Humble 설치

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

### 1.2 빌드 및 런타임 의존성

```bash
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-pip git \
  build-essential cmake \
  libeigen3-dev libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev libtins-dev libpcap-dev \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-rviz2 ros-humble-tf2-ros \
  avahi-utils curl iputils-ping

sudo rosdep init 2>/dev/null || true
rosdep update
```

`avahi-utils` 는 센서를 IP 없이 찾을 때 쓴다. `curl` 은 센서 REST API 조회용이다.

### 1.3 Python 패키지 — 버전 충돌 주의

Ubuntu 22.04 기본 `python3-scipy`(1.8.0)는 **NumPy 1.x 로 컴파일**되어 있다.
pip 로 설치된 NumPy 2.x 와 같이 쓰면 import 자체가 실패한다:

```
AttributeError: _ARRAY_API not found
ImportError: numpy.core.multiarray failed to import
```

scipy 를 NumPy 2.x 대응 버전으로 올려서 해결한다:

```bash
python3 -m pip install --user --upgrade "scipy>=1.15" scikit-learn
```

검증:

```bash
python3 -c "import numpy,scipy,sklearn; \
print('numpy',numpy.__version__,'scipy',scipy.__version__,'sklearn',sklearn.__version__)"
# 기대: numpy 2.2.6 scipy 1.15.3 sklearn 1.7.2
```

---

## 2. 네트워크 설정

### 2.1 PC 고정 IP

센서와 같은 대역에 PC 를 둔다. 유선 인터페이스 이름은 `ip -br link` 로 확인한다.

```bash
ip -br link                       # 예: enp59s0
sudo ip addr add 192.168.6.100/24 dev enp59s0
sudo ip link set enp59s0 up
```

NetworkManager 로 영구 설정하려면:

```bash
nmcli con add type ethernet ifname enp59s0 con-name ouster \
  ipv4.method manual ipv4.addresses 192.168.6.100/24
nmcli con up ouster
```

### 2.2 센서 연결 확인

```bash
ping -c 3 192.168.6.11
```

응답이 오면 센서 신원을 확인한다. **핑만으로는 센서 교체 여부를 알 수 없다** —
반드시 시리얼을 본다:

```bash
curl -s http://192.168.6.11/api/v1/sensor/metadata/sensor_info
```

```json
{"prod_line": "OS-1-128", "prod_pn": "OS1-070-128U-AX",
 "prod_sn": "122531002331", "build_rev": "v3.1.0", "status": "RUNNING",
 "initialization_id": 390072}
```

- `prod_sn` 이 바뀌면 다른 센서다.
- `initialization_id` 가 그대로면 재부팅되지 않은 것이다 (전원을 뺐다 꽂으면 바뀐다).

### 2.3 IP 를 모를 때 — mDNS 로 탐색

```bash
avahi-browse -art | grep -i ouster
```

```
+ enp59s0 IPv4 Ouster Sensor 122531002331  _ouster-lidar._tcp  local
   txt = ["path=/api/v1" "sn=122531002331" "pn=840-104682-05"]
```

서브넷을 훑어볼 수도 있다:

```bash
for i in $(seq 1 254); do (ping -c1 -W1 192.168.6.$i >/dev/null 2>&1 && echo "192.168.6.$i 응답") & done; wait
```

---

## 3. 워크스페이스 구성

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
```

### 3.1 중첩 저장소가 있으면 COLCON_IGNORE

`src/` 안에 다른 워크스페이스를 통째로 clone 해 두면 패키지 이름이 겹쳐 빌드가 막힌다:

```
ERROR:colcon:colcon build: Duplicate package names not supported:
- drone_localization:
  - src/DronePositioningSystem/ros2_ws/src/drone_localization
  - src/drone_localization
```

해당 저장소 최상단에 마커 파일을 두면 colcon 이 건너뛴다 (파일 삭제 불필요):

```bash
touch ~/ros2_ws/src/DronePositioningSystem/COLCON_IGNORE
```

---

## 4. ouster-ros 소스 패치 (필수)

### 4.1 증상

펌웨어 **3.2 미만** 센서에서 드라이버가 센서 접속까지 성공한 뒤 즉시 죽는다:

```
terminate called after throwing an instance of 'std::out_of_range'
  what():  Field 'WINDOW' not found in LidarScan.
[ERROR] [os_driver-1]: process has died [exit code -6]
```

### 4.2 원인

- ouster-sdk 는 FW < 3.2 센서에서 `WINDOW` 채널 필드를 **런타임에 제거**한다
  (`ouster_client/src/lidar_scan.cpp` 의 `get_field_types()`).
- 반면 ouster-ros 의 포인트클라우드 변환 테이블
  (`include/ouster_ros/sensor_point_types.h` 의 `Profile_RNG19_RFL8_SIG16_NIR16`)
  은 `WINDOW` 를 **무조건 포함**한다.
- 그 결과 `point_cloud_compose.h` 가 존재하지 않는 필드를 참조해 예외가 발생한다.

업스트림의 FW 3.2 지원 커밋이 ROS 쪽 테이블에 버전 분기를 넣지 않아 생긴 문제로,
FW < 3.2 + `RNG19_RFL8_SIG16_NIR16` 조합이면 누구나 겪는다.

### 4.3 패치

`src/ouster-ros/ouster-ros/src/point_cloud_compose.h` 를 수정한다.

**(1) 헤더 추가**

```cpp
#pragma once

#include <cstddef>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
```

**(2) 0으로 채운 대체 버퍼 헬퍼 추가** — `make_lidar_scan_tuple` 정의 위

```cpp
/**
 * @brief returns a pointer to a zero-filled buffer of at least n elements of
 * type T. Used as a stand-in for LidarScan fields that a profile's field
 * table expects but that the connected sensor doesn't populate (e.g. WINDOW
 * on firmware < 3.2).
 */
template <typename T>
const T* zero_field_buffer(std::size_t n) {
    static thread_local std::vector<T> buf;
    if (buf.size() < n) buf.assign(n, T{});
    return buf.data();
}
```

**(3) 필드 접근을 `has_field()` 로 가드**

```cpp
        // 기존
        // std::get<Index>(tp) = ls.field<FieldType>(Table[Index].first).data();

        // 변경
        std::get<Index>(tp) = ls.has_field(Table[Index].first)
            ? ls.field<FieldType>(Table[Index].first).data()
            : zero_field_buffer<FieldType>(
                  static_cast<std::size_t>(ls.w) * static_cast<std::size_t>(ls.h));
```

**패치 확인**

```bash
grep -n "has_field\|zero_field_buffer" \
  ~/ros2_ws/src/ouster-ros/ouster-ros/src/point_cloud_compose.h
# 69: const T* zero_field_buffer(std::size_t n) {
# 91:         std::get<Index>(tp) = ls.has_field(Table[Index].first)
# 93:             : zero_field_buffer<FieldType>(
```

### 4.4 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select ouster_ros --symlink-install    # 약 40초
```

---

## 5. dual_ouster_driver 패키지

### 5.1 패키지 생성

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python dual_ouster_driver
mkdir -p dual_ouster_driver/launch dual_ouster_driver/rviz
```

### 5.2 `package.xml` 의존성

```xml
  <depend>launch</depend>
  <depend>launch_ros</depend>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>sensor_msgs_py</depend>
  <depend>geometry_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>std_msgs</depend>
  <depend>python3-numpy</depend>
  <depend>python3-sklearn</depend>
```

### 5.3 `setup.py` — 진입점과 데이터 파일

```python
from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'dual_ouster_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'roi_filter = dual_ouster_driver.roi_filter:main',
            'axis_probe = dual_ouster_driver.axis_probe:main',
            'cluster_median = dual_ouster_driver.cluster_median:main',
        ],
    },
    zip_safe=True,
    maintainer='drone',
    maintainer_email='drone@example.com',
    description='Dual Ouster OS1-128 driver launch',
    license='Apache-2.0',
)
```

### 5.4 런치 파일 — 라이프사이클 전이가 핵심

`launch/dual_os1.launch.py`

**빠뜨리면 반드시 막히는 부분:** `os_driver` 는 **라이프사이클 노드**다.
`Node` 로 띄우면 `unconfigured` 상태에서 멈춘 채 센서 접속조차 시도하지 않는다.
로그는 아래에서 끊기고 CPU 0% 로 무한 대기한다:

```
[WARN] lidar port set to zero, the client will assign a random port number!
[WARN] imu port set to zero, the client will assign a random port number!
[INFO] TIME_FROM_ROS_TIME timestamp mode specified. ...
```

`LifecycleNode` + `configure` → `activate` 전이 이벤트를 반드시 넣어야 한다.

```python
#!/usr/bin/env python3

import launch
import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, LogInfo
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition


def generate_launch_description():

    lidar1 = LifecycleNode(
        package='ouster_ros',
        executable='os_driver',
        name='os_driver_lidar1',
        namespace='lidar1',          # 토픽이 /lidar1/points 로 나온다
        output='screen',
        parameters=[
            {
                'sensor_hostname': '192.168.6.11',
                'lidar_mode': '1024x10',
                'timestamp_mode': 'TIME_FROM_ROS_TIME',
                'sensor_frame': 'lidar1_sensor',
                'lidar_frame': 'lidar1',
                'imu_frame': 'lidar1_imu',
                'point_cloud_frame': 'lidar1',
                'point_type': 'xyz',
                'proc_mask': 'PCL|IMU',
                'lidar_port': 0,
                'imu_port': 0,
                'udp_dest': '',
                'attempt_reconnect': True,
            }
        ],
    )

    lidar1_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(lidar1),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    lidar1_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar1, goal_state='inactive',
            entities=[
                LogInfo(msg="os_driver_lidar1 activating..."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lidar1),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
            handle_once=True
        )
    )

    lidar1_finalized_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar1, goal_state='finalized',
            entities=[
                LogInfo(msg="Failed to communicate with lidar1 (192.168.6.11)."),
                EmitEvent(event=launch.events.Shutdown(
                    reason="Couldn't communicate with lidar1"))
            ],
        )
    )

    return LaunchDescription([
        lidar1,
        lidar1_configure_event,
        lidar1_activate_event,
        lidar1_finalized_event,
    ])
```

**주의 두 가지**

1. `sensor_hostname` 값에 **공백이 섞이면 안 된다**. `'192.168.6.11   '` 같은 값은 접속에 실패한다.
2. 토픽 이름은 **`remappings` 가 아니라 `namespace`** 로 잡는다.
   이 버전의 ouster-ros 는 `points`, `imu`, `metadata` 같은 **상대 이름**으로 발행하므로
   (`src/os_cloud_node.cpp` 의 `create_publisher<...>("points", ...)`),
   `/ouster/points` → `/lidar1/points` 식의 리매핑은 매칭될 대상이 없어 무시된다.

### 5.5 노드 소스

아래 세 파일을 `dual_ouster_driver/dual_ouster_driver/` 에 둔다.
전체 소스는 이 저장소의 해당 경로를 그대로 복사하면 된다.

| 파일 | 역할 |
|---|---|
| `roi_filter.py` | 좌표 회전 + 높이 보정 후 관심영역(ROI)만 잘라 재발행 |
| `axis_probe.py` | 8방위 최근접 거리 출력 — 라이다 정면 방향 실측용 |
| `cluster_median.py` | ROI 안을 DBSCAN 군집화 → 가장 큰 군집의 median 좌표 출력 |

### 5.6 RViz 설정

`rviz/roi_view.rviz` 를 만든다. 핵심은 **QoS 를 Best Effort 로 맞추는 것**이다.
포인트클라우드 토픽은 `SensorDataQoS`(Best Effort)로 나가는데
RViz 기본값은 Reliable 이라 그냥 두면 아무것도 안 그려진다:

```
[WARN] New publisher discovered on topic '/lidar1/points', offering incompatible QoS.
       No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

```yaml
    - Class: rviz_default_plugins/PointCloud2
      Enabled: true
      Name: ROIPoints
      Topic:
        Value: /lidar1/roi_points
        Reliability Policy: Best Effort     # <- 필수
      Style: Points
```

### 5.7 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dual_ouster_driver --symlink-install
source install/setup.bash

ros2 pkg executables dual_ouster_driver
# dual_ouster_driver axis_probe
# dual_ouster_driver cluster_median
# dual_ouster_driver roi_filter
```

`~/.bashrc` 에 넣어두면 새 터미널마다 자동 적용된다:

```bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

> `Package 'dual_ouster_driver' not found` 는 대부분 워크스페이스를 source 하지 않아서 난다.

---

## 6. 좌표계 보정 (설치할 때마다 1회)

### 6.1 Ouster 좌표계 개념

센서는 좌표계를 두 벌 갖는다.

| 프레임 | 정의 |
|---|---|
| Sensor frame | `+x` 가 커넥터 쪽 |
| Lidar frame | Sensor frame 을 z축으로 180° 회전 (`+x` 가 커넥터 반대쪽) |

메타데이터의 변환 행렬이 이 관계를 보여준다:

```
lidar_to_sensor_transform = [-1, 0, 0,      0]
                            [ 0,-1, 0,      0]
                            [ 0, 0, 1, 38.195]
```

ouster-ros 는 `point_cloud_frame == sensor_frame` 일 때만 이 변환을 적용한다
(`src/os_static_transforms_broadcaster.h`):

```cpp
bool apply_lidar_to_sensor_transform() const {
    return point_cloud_frame == sensor_frame;
}
```

위 런치 설정은 `point_cloud_frame='lidar1'`, `sensor_frame='lidar1_sensor'` 로 서로 달라
**변환이 적용되지 않는다 = 이미 Lidar frame 이다.**

> **오해 주의:** 프레임을 바꿔도 스캔 내용은 그대로다. 두 프레임 차이는 회전과 오프셋뿐,
> 같은 점에 다른 좌표를 붙일 뿐이다. 라이다는 360° 전방향을 항상 훑으므로
> 커넥터 쪽도 반드시 찍힌다. **특정 방향을 안 찍으려면 프레임이 아니라
> 센서의 `azimuth_window` 를 좁혀야 한다.**

### 6.2 정면 방향 실측

문서만 보고 추정하면 틀리기 쉽다. 반드시 사람이 서서 확인한다.

**1) 드라이버를 띄운다** (아래 [8. 실행](#8-실행) 참고)

**2) 측정 노드 실행**

```bash
ros2 run dual_ouster_driver axis_probe
```

1초마다 8방위 최근접 거리가 출력된다:

```
  방향            최근접거리   점개수
  +x  (  0deg)    3.23 m    3239
  +y  ( 90deg)    0.93 m    5951
  -x  (180deg)    0.51 m    9133
  -y  (270deg)    0.85 m    6193
```

**3) 정면이라 생각하는 위치에 1 m 앞에 10초간 서 있는다**

**4) 거리가 급감하고 점 개수가 폭증한 방향이 정면이다**

```
+x  (0deg)   3.23m -> 0.80m  (-2.43m)   점 3239 -> 7869  (+4630)
   나머지 7방향은 변화 0에 가까움
```

**5) `yaw_deg` 를 정한다** — 정면이 `+x` 가 되도록

| 실측 정면 | `yaw_deg` |
|---|---|
| `+x` | `0` |
| `+y` | `-90` |
| `-x` | `180` |
| `-y` | `90` |

이 설치에서는 정면이 `-x` 여서 **`yaw_deg = 180`** 이 되었다.
`roi_filter.py` 와 `axis_probe.py` 의 기본값이 이 값이며, **두 노드가 같아야 한다.**

### 6.3 설치 높이 측정

바닥 점의 z 최빈값으로 구한다. `lidar_z` 파라미터에 넣는다.

```
바닥 z (라이다 기준) = -0.84 m   ->   lidar_z = 0.84
```

> **수직 사각지대:** OS1 의 수직 시야각은 ±21.2° 다. 높이 0.84 m 에 설치하면
> `0.84 / tan(21.2°) ≈ 2.2 m` 안쪽 바닥은 **원리적으로 보이지 않는다.**
> RViz 탑뷰에서 중앙이 원형으로 뚫려 보이는 것은 정상이다.

---

## 7. 새 위치에서 재보정

라이다를 옮기면 아래를 다시 한다.

1. **정면 방향 확인** — `axis_probe` (6.2)
2. **설치 높이 측정** — `lidar_z` (6.3)
3. **벽 위치 파악 후 ROI 박스 조정**

ROI 를 넉넉히 열고 남는 점을 확인하면서 좁혀 나간다.
빈 방에서 **0점**이 되면 벽·바닥이 모두 제거된 것이다.

```bash
# 넓게 열고 시작
ros2 param set /roi_filter roi_x_max 5.0
ros2 param set /roi_filter roi_y_min -2.5
ros2 param set /roi_filter roi_y_max 2.5
ros2 param set /roi_filter roi_z_min 0.0

# 로그를 보며 벽이 빠질 때까지 조인다
ros2 param set /roi_filter roi_x_max 4.5
ros2 param set /roi_filter roi_z_min 0.25
```

`roi_filter` 는 **매 프레임 파라미터를 다시 읽으므로 재시작이 필요 없다.**

현재 설치 위치의 실측 결과 (참고용):

```
바닥      z ~ 0.2 아래
정면 벽   x ~ 4.7 m
좌측 벽   y ~ +2.3 m
우측 벽   y ~ -2.4 m

-> 적용값: x 0~4.5,  y -2.15~2.1,  z 0.25~2.0   (빈 방에서 0점 확인)
```

---

## 8. 실행

터미널마다 먼저 `source ~/ros2_ws/install/setup.bash` 를 실행한다
(`.bashrc` 에 넣었다면 생략).

### 터미널 1 — 드라이버

```bash
ros2 launch dual_ouster_driver dual_os1.launch.py
```

정상 로그 (센서 초기화에 10~15초 걸린다):

```
[INFO] Contacting sensor 192.168.6.11 ...
[INFO] Sensor 192.168.6.11 configured successfully
[INFO] Starting sensor 192.168.6.11 initialization... Using ports: 0/0
product: OS-1-128, sn: 122531002331, firmware ver: 3.1.0
lidar mode: 1024x10, lidar udp profile: RNG19_RFL8_SIG16_NIR16
[INFO] [launch.user]: os_driver_lidar1 activating...
```

### 터미널 2 — TF

`z` 는 `lidar_z`, 네 번째 인자는 `yaw`(라디안). `yaw_deg=180` → `3.14159`.

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0.84 3.14159 0 0 map lidar1
```

### 터미널 3 — ROI 필터

```bash
ros2 run dual_ouster_driver roi_filter
```

```
ROI filter: /lidar1/points -> /lidar1/roi_points [map], lidar_z=0.84, yaw=180deg (+x=정면)
box x[0.0,4.5] y[-2.2,2.1] z[0.2,2.0]
19프레임: 123976 -> 0점 (0.0%)  |  처리 11.6/12.8ms  |  입력 10.0Hz  |  지연 202/202ms
```

### 터미널 4 — 군집화 + median

```bash
ros2 run dual_ouster_driver cluster_median
```

물체가 있을 때만 출력된다:

```
median   x=  2.44   y=  0.04   z=  0.13   [t=1787231483.102  지연 206ms]
[주기] 처리 0.2/0.4ms(평균/최대)  |  입력 10.0Hz(100ms 간격)  |  지연 223/226ms
```

### 터미널 5 — RViz

```bash
rviz2 -d ~/ros2_ws/src/dual_ouster_driver/rviz/roi_view.rviz
```

| 디스플레이 | 내용 |
|---|---|
| RawLidar1 | 원본 전체 (흐린 무지개색) |
| ROIPoints | ROI 통과 점 (빨강) |
| ClusterMedian | 군집 중심 (초록 구 + 좌표 텍스트) |

---

## 9. 토픽과 파라미터

### 9.1 토픽

| 토픽 | 타입 | 발행 |
|---|---|---|
| `/lidar1/points` | `sensor_msgs/PointCloud2` | 드라이버 (128×1024, 10 Hz) |
| `/lidar1/imu` | `sensor_msgs/Imu` | 드라이버 |
| `/lidar1/metadata` | `std_msgs/String` | 드라이버 |
| `/lidar1/roi_points` | `sensor_msgs/PointCloud2` | `roi_filter` |
| `/lidar1/cluster_poses` | `geometry_msgs/PoseArray` | `cluster_median` |
| `/lidar1/cluster_markers` | `visualization_msgs/MarkerArray` | `cluster_median` |

### 9.2 `roi_filter` 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `input_topic` | `/lidar1/points` | 입력 |
| `output_topic` | `/lidar1/roi_points` | 출력 |
| `output_frame` | `map` | 출력 프레임 |
| `lidar_z` | `0.84` | 설치 높이 (m) |
| `yaw_deg` | `180.0` | z축 회전. 정면이 `+x` 가 되게 |
| `roi_x_min` / `roi_x_max` | `0.0` / `4.5` | 정면 방향 범위 |
| `roi_y_min` / `roi_y_max` | `-2.15` / `2.1` | 좌우 범위 |
| `roi_z_min` / `roi_z_max` | `0.25` / `2.0` | 높이 범위 (`0` = 바닥) |
| `min_range` | `0.1` | 원점 근처 무효점 제거 |

### 9.3 `cluster_median` 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `eps` | `0.20` | DBSCAN 이웃 반경 (m) |
| `min_samples` | `8` | DBSCAN 최소 이웃 수 |
| `voxel_size` | `0.0` | 복셀 다운샘플. `0` = 사용 안 함 |
| `min_points` / `max_points` | `8` / `100000` | 군집 크기 필터 |
| `max_span` | `2.0` | 이보다 넓으면 벽으로 보고 제외 |
| `log_period` | `0.5` | median 출력 주기 (초) |
| `stats_period` | `2.0` | 주기/지연 통계 출력 (초). `0` = 끔 |

### 9.4 실행 중 조절 (재시작 불필요)

```bash
ros2 param set /roi_filter roi_x_max 4.0
ros2 param set /roi_filter roi_z_min 0.15
ros2 param get /roi_filter roi_x_max

ros2 param set /cluster_median eps 0.3
ros2 param set /cluster_median voxel_size 0.05
```

---

## 10. 종료

```bash
# 개별
pkill -f roi_filter
pkill -f cluster_median
pkill -f static_transform_publisher
pkill -f rviz2
pkill -f dual_os1.launch

# 한 번에
pkill -f 'roi_filter|cluster_median|static_transform_publisher|rviz2|dual_os1.launch'

# 드라이버만 남기고 정리 (튜닝 중 유용 — 재시작하면 초기화에 15초)
pkill -f 'roi_filter|cluster_median|rviz2'

# 확인
ps -ef | grep -E "os_driver|roi_filter|cluster_median|rviz2" | grep -v grep
```

> `pkill -f` 는 명령줄 전체를 매칭한다. **스크립트 안에서 쓰면 그 스크립트 자신을 죽일 수 있다.**
> 대화형 터미널에서 직접 치는 것은 안전하다. 포그라운드로 띄운 노드는 `Ctrl+C` 가 낫다.

---

## 11. 문제 해결

### `Package 'dual_ouster_driver' not found`

워크스페이스를 source 하지 않았다.

```bash
source ~/ros2_ws/install/setup.bash
```

### 드라이버가 로그 세 줄 찍고 멈춤 (CPU 0%)

라이프사이클 전이가 안 걸렸다. 상태를 확인한다:

```bash
ros2 lifecycle get /lidar1/os_driver_lidar1
# unconfigured [1]  <- 문제
```

런치 파일에 `configure`/`activate` 이벤트가 있는지 본다 (5.4).
급하면 수동으로 전이시킬 수 있다:

```bash
ros2 lifecycle set /lidar1/os_driver_lidar1 configure
ros2 lifecycle set /lidar1/os_driver_lidar1 activate
```

### `Field 'WINDOW' not found in LidarScan` 로 프로세스 사망

4장 패치가 빠졌거나 빌드가 안 됐다.

```bash
grep -n "has_field" ~/ros2_ws/src/ouster-ros/ouster-ros/src/point_cloud_compose.h
colcon build --packages-select ouster_ros --symlink-install
```

### 노드는 `active` 인데 `/lidar1/points` 가 안 나옴

오래 방치하면 UDP 스트림이 끊긴다. 드라이버를 재시작한다.

```bash
pkill -f dual_os1.launch
ros2 launch dual_ouster_driver dual_os1.launch.py
```

### 노드 이름 충돌

```
WARNING: Be aware that there are nodes in the graph that share an exact name
```

같은 노드를 두 번 띄웠다. 시작 시각으로 구분해 정리한다.

```bash
ps -eo pid,lstart,cmd | grep roi_filter | grep -v grep
kill <오래된_PID>
```

### RViz 에 아무것도 안 보임

원인이 두 가지다.

1. **QoS 불일치** — 로그에 `incompatible QoS` 가 뜬다. 5.6 참고.
2. **TF 없음** — `map` 과 `lidar1` 이 연결되지 않았다.

```bash
ros2 run tf2_ros tf2_echo map lidar1     # 응답 없으면 TF 퍼블리셔를 띄운다
```

### ROI 에 점이 너무 많다

대부분 벽 아니면 바닥이다. 높이 분포를 보면 바로 구분된다.
바닥이면 `roi_z_min` 을 올리고, 벽이면 `roi_x_max` / `roi_y_*` 를 조인다.
7장 절차대로 재보정한다.

### `_ARRAY_API not found` / `numpy.core.multiarray failed to import`

NumPy 2.x 와 시스템 scipy 1.8 의 충돌이다. 1.3 참고.

```bash
python3 -m pip install --user --upgrade "scipy>=1.15"
```

### colcon `Duplicate package names not supported`

`src/` 안에 같은 이름 패키지가 중첩돼 있다. 3.1 참고.

```bash
touch ~/ros2_ws/src/<중첩된_저장소>/COLCON_IGNORE
```

---

## 12. 알려진 제약

| 항목 | 내용 |
|---|---|
| **종단 지연 202 ms** | 라이다 촬영 → median 출력까지. 정지 물체는 무관하나 움직이는 표적은 위치가 뒤처진다 (2 m/s 이면 약 40 cm). 원인 미규명. |
| **바닥 사각지대 ~2.2 m** | 수직 시야각 ±21.2° 의 물리적 한계. 발밑 바닥은 볼 수 없다. |
| **최소 측정 거리 0.5 m** | 센서의 `min_range_threshold_cm: 50` 설정. 더 가까운 물체는 반환되지 않는다. |
| **표면 편향** | 라이다는 자기 쪽 면만 본다. 단일 라이다에서는 상쇄되지 않아 센트로이드가 센서 쪽으로 물체 깊이의 절반가량 치우친다. |
| **ROI 값의 장소 의존성** | 벽 위치 기준으로 맞춘 값이라 라이다를 옮기면 7장 재보정이 필요하다. |
| **DBSCAN 성능** | 복셀 없이 12,000점이면 2~3 Hz 까지 떨어진다. ROI 가 좁으면 문제없고, 넓게 쓸 때는 `voxel_size` 를 `0.05` 로 켠다. |

---

## 13. 빠른 시작 요약

```bash
# 0) 사전 확인
ping -c 3 192.168.6.11
curl -s http://192.168.6.11/api/v1/sensor/metadata/sensor_info

# 1) 빌드
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 2) 실행 (터미널 5개)
ros2 launch dual_ouster_driver dual_os1.launch.py
ros2 run tf2_ros static_transform_publisher 0 0 0.84 3.14159 0 0 map lidar1
ros2 run dual_ouster_driver roi_filter
ros2 run dual_ouster_driver cluster_median
rviz2 -d ~/ros2_ws/src/dual_ouster_driver/rviz/roi_view.rviz
```
