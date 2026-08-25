# 전체 파이프라인 — 라이다 2대

랜선 꽂는 것부터 결과 그림이 나올 때까지, 실제로 도는 것 전부.
각 단계마다 **무엇을 / 어디 있는지 / 왜 그렇게 생겼는지 / 무엇이 문제인지**.

- 설치 절차 → [LIDAR_SETUP.md](LIDAR_SETUP.md)
- 보정 계산 원리 → [CALIBRATION.md](CALIBRATION.md)
- 2대 구성에서 새로 한 것 → [DUAL_LIDAR.md](DUAL_LIDAR.md)
- 명령어만 → [RUN.md](RUN.md)

기준일 2026-08-24. 이 문서의 값은 전부 이 PC 에서 실측한 것이다.

---

## 0. 한눈에

```
 [센서]  OS1-128 x2                    192.168.6.11 / .12  (센서가 스스로 기억)
    |
    |  이더넷 + 24V.  PC 는 192.168.6.100.  같은 /24 라 직접 통신
    |  UDP  (udp_dest 를 PC 주소로 명시해야 함)
    v
 [드라이버]  ouster_ros/os_driver x2    dual_os1.launch.py
    |        라이프사이클 configure -> activate 를 직접 쏴야 함
    |        /lidar1/points   /lidar2/points   각 2MB, 10Hz, BEST_EFFORT
    |        <- Fast DDS SHM 프로파일 없으면 여기서 무너짐 (2대면 1.4Hz)
    v
 [보정 적용]  roi_filter x2             calibration_lidarN.yaml
    |         회전+평행이동 -> map 프레임, ROI 상자로 크롭
    |         /lidar1/roi_points   /lidar2/roi_points
    v
 [군집]  cluster_median x2
    |     DBSCAN -> 최대 군집 -> median
    |     /lidar1/cluster_poses   /lidar2/cluster_poses
    |
    |     ##########  여기가 끊겨 있다. 두 스트림을 합치는 코드가 없다  ##########
    v
 [기록]  flight_recorder            <- 한 토픽만 구독
    |     6상태 칼만필터 -> track.csv  (+ rosbag)
    v
 [분석]  flight_report              -> report.txt, flight_3d.html, flight_2d.png
```

**군집화까지는 2대가 다 도는데, 그 뒤가 1대짜리다.** 이게 지금 가장 큰 구멍이다.

---

## 1. 네트워크 — PC 와 라이다를 잇는다

### 1.1 개념 — 라우터도 DHCP 도 없다

가장 헷갈리는 부분이라 먼저 정리한다.

**라이다는 자기 IP 를 스스로 기억한다.** 공유기가 나눠주는 게 아니다. 센서 안에
`192.168.6.11/24` 이 저장돼 있고, 전원을 넣으면 그 주소로 뜬다.

**PC 는 유선 랜카드에 같은 대역의 주소를 직접 박아둔다.** `192.168.6.100/24`.

```
    PC  192.168.6.100/24  ──────────  라이다  192.168.6.11/24
         (enp59s0)                            192.168.6.12/24
         
    같은 192.168.6.0/24  ->  라우터 없이 서로 직접 통신
```

`/24` 는 앞 세 자리(`192.168.6.`)가 같으면 같은 동네라는 뜻이다. 같은 동네면
**게이트웨이(라우터)를 거치지 않고 바로** 패킷이 오간다.

그래서 이 연결에는:

- **공유기 필요 없음** — 랜선으로 직결하거나 스위치만 거치면 된다
- **DHCP 서버 필요 없음** — 양쪽 다 주소를 고정으로 박았다
- **게이트웨이 설정하면 안 됨** — 설정하면 PC 의 인터넷 기본 경로를 가로챈다

**주소 대역(`192.168.6.x`)은 임의로 고른 것**이다. 다만 두 조건을 지켜야 한다.

1. PC 와 라이다가 **같은 대역**일 것 (`192.168.6.` 부분이 일치)
2. **와이파이 대역과 겹치지 않을 것**

2번이 중요하다. 지금 이 PC 의 와이파이는 `172.20.10.12/28` (아이폰 핫스팟)이라
안 겹친다. 만약 와이파이가 `192.168.6.x` 를 받으면 커널이 어느 쪽으로 보낼지
헷갈려서 라이다가 안 잡힌다. 그럴 땐 라이다 대역을 `192.168.7.x` 같은 것으로
옮긴다(PC·센서 양쪽 다).

### 1.2 배선

```
   라이다 본체
      |
      | (전용 케이블)
      v
   인터커넥터 박스 ── 24V 전원 어댑터
      |
      | (일반 랜선)
      v
   ┌──────────┐
   │  스위치   │   <- 라이다 2대면 필요. PC 유선 포트가 하나뿐이라
   └──────────┘
      |
      | (일반 랜선)
      v
   PC  enp59s0
```

라이다 한 대면 스위치 없이 **PC ↔ 인터커넥터 직결**로도 된다. 요즘 랜카드는
Auto-MDIX 라 크로스 케이블도 필요 없다.

스위치는 **아무 기가비트 스위치**면 된다. 라이다 2대면 40 MB/s 라 100Mbps
스위치로는 부족하다(**기가비트여야 함**).

### 1.3 PC 쪽 설정 — 한 번만 하면 된다

NetworkManager 에 `ouster` 라는 프로파일을 만들어 뒀다. 랜선을 꽂으면 자동으로
올라온다.

**지금 설정 (실측)**

```
connection.interface-name:  enp59s0
connection.autoconnect:     yes
ipv4.method:                manual          <- DHCP 안 씀
ipv4.addresses:             192.168.6.100/24
ipv4.gateway:               --              <- 비워둠. 중요
ipv6.method:                auto
```

**처음 만들 때 (다른 PC 에 옮길 때)**

```bash
# 유선 인터페이스 이름부터 확인 (enp59s0 은 이 PC 기준)
ip -br link show
```

```bash
nmcli con add type ethernet ifname enp59s0 con-name ouster \
  ipv4.method manual \
  ipv4.addresses 192.168.6.100/24 \
  ipv6.method auto \
  autoconnect yes

nmcli con up ouster
```

> **`ipv4.gateway` 를 넣지 말 것.** 넣으면 PC 의 기본 경로가 라이다 쪽으로
> 잡혀서 인터넷이 끊긴다. 라이다는 같은 동네에 있으므로 게이트웨이가 필요없다.

**확인**

```bash
ip -br addr show enp59s0
# enp59s0   UP   192.168.6.100/24 fe80::.../64      <- 이렇게 나와야 정상
```

`DOWN` 이거나 주소가 없으면 아래 1.6 으로.

### 1.4 센서 쪽 설정 — IP 를 바꿔야 할 때

센서는 **자기 IP 를 내부에 저장**한다(`ipv4.override`). 웹 API 로 바꾼다.

**IP 를 아는 경우**

```bash
# 현재 설정 보기
curl -s http://192.168.6.11/api/v1/system/network | python3 -m json.tool
```

```json
{
  "ethaddr": "bc:0f:a7:01:01:11",
  "ipv4": {
    "link_local": "169.254.98.30/16",
    "addr":       "192.168.6.11/24",
    "override":   "192.168.6.11/24"      <- 이게 저장된 고정 주소
  },
  "hostname": "os-122531002331"
}
```

```bash
# 바꾸기
curl -s -X PUT http://192.168.6.11/api/v1/system/network/ipv4/override \
  -H 'Content-Type: application/json' -d '"192.168.6.11/24"'
```

**IP 를 모르는 경우 — mDNS 로 찾는다**

Ouster 는 `os-<시리얼>.local` 로 자기를 광고한다.

```bash
sudo apt install -y avahi-utils     # 없으면
avahi-browse -art | grep -iA3 ouster
```

```
= enp59s0 IPv6 Ouster Sensor 122451001190   _ouster-lidar._tcp   local
   hostname = [os-122451001190.local]
   address  = [fe80::be0f:a7ff:fe00:da36]
   txt = ["sn=122451001190" "pn=840-104682-05"]
```

**IPv4 가 겹쳐도 IPv6 링크로컬은 MAC 기반이라 항상 고유하다.** 그쪽으로 붙는다.

```bash
IP6="fe80::be0f:a7ff:fe00:da36%enp59s0"      # % 뒤는 인터페이스 이름

# 시리얼 먼저 확인. 엉뚱한 센서를 건드리지 않게
curl -s -g "http://[$IP6]/api/v1/sensor/metadata/sensor_info"

# 고정 IP 변경
curl -s -g -X PUT "http://[$IP6]/api/v1/system/network/ipv4/override" \
  -H 'Content-Type: application/json' -d '"192.168.6.12/24"'
```

`curl` 의 `-g` 는 대괄호를 글로브 문자로 해석하지 말라는 뜻이다. IPv6 주소에는
필수다.

바꾼 뒤 5~10초면 새 주소로 붙는다. **IPv6 로 작업하므로 실패해도 복구된다.**

**현재 배치**

| IP | 모델 | 시리얼 | MAC | 토픽 |
|---|---|---|---|---|
| 192.168.6.11 | OS1-070-128U-**AX** | 122531002331 | bc:0f:a7:01:01:11 | `/lidar1/points` |
| 192.168.6.12 | OS1-070-128U-**ASR** | 122451001190 | bc:0f:a7:00:da:36 | `/lidar2/points` |
| 192.168.6.100 | — | PC (enp59s0) | b4:45:06:2f:79:fd | — |

모델이 다르지만 **수직 시야는 같다** (−21.46~+21.12 vs −21.48~+21.18).
`ASR` 은 단거리 버전으로 최대 사거리만 다르다.

### 1.5 확인 사다리 — 위에서부터 하나씩

문제가 생기면 **아래 순서대로** 짚는다. 각 단계가 다른 종류의 고장을 잡아낸다.

**① 물리 링크 — 랜선이 꽂혀 있나**

```bash
cat /sys/class/net/enp59s0/carrier
```

`1` 이면 링크 있음, `0` 이면 없음. **`0` 이면 여기서 멈춘다.** 랜선·스위치 전원·
인터커넥터 전원을 본다. ROS 는 볼 필요도 없다.

```bash
ip -br link show enp59s0
# enp59s0  UP  ...           <- 정상
# enp59s0  DOWN ...          <- NO-CARRIER. 케이블 문제
```

**② PC 주소 — IP 가 붙었나**

```bash
ip -br addr show enp59s0
# enp59s0  UP  192.168.6.100/24 ...
```

주소가 없으면 프로파일이 안 올라온 것이다.

```bash
nmcli con up ouster
```

**③ L2 도달 — 상대가 응답하나 (ARP)**

```bash
ping -c2 192.168.6.11
ip neigh show dev enp59s0
# 192.168.6.11 lladdr bc:0f:a7:01:01:11 REACHABLE
```

`lladdr` 이 보이면 **L2 까지는 붙은 것**이다. ping 이 안 되는데 lladdr 이 보이면
방화벽이나 주소 설정 문제고, lladdr 자체가 없으면 케이블/스위치 문제다.

**④ 응용 계층 — 센서가 살아 있나**

```bash
curl -s http://192.168.6.11/api/v1/sensor/metadata/sensor_info | python3 -m json.tool
```

```json
{ "prod_pn": "OS1-070-128U-AX", "prod_sn": "122531002331",
  "build_rev": "v3.1.0", "status": "RUNNING" }
```

`status` 가 `RUNNING` 이어야 한다. `WARMUP` 이면 조금 기다린다.

**⑤ UDP 목적지 — 데이터를 어디로 쏘고 있나**

```bash
curl -s http://192.168.6.11/api/v1/sensor/config | python3 -m json.tool | grep udp
```

```
"udp_dest": "192.168.6.100",       <- PC 주소여야 정상
"udp_port_lidar": 60522,
"udp_port_imu": 60523
```

**`169.254.x.x` 로 되어 있으면 데이터가 허공으로 간다.** 아래 1.6 참고.

**⑥ 실제 데이터 — 토픽이 흐르나**

```bash
ros2 topic hz /lidar1/points        # 10 근처
```

### 1.6 네트워크 쪽 문제 세 가지

**(a) `carrier=0` — 랜선이 안 꽂혔다**

```
enp59s0   DOWN
carrier=0
```

가장 흔하다. 실제로 이것 때문에 한참 헤맸다. 인터커넥터 전원도 같이 본다.

**(b) 두 센서가 같은 IP**

공장 출하 시 둘 다 `192.168.6.11` 인 경우가 있다. 서브넷을 전부 훑어도 한 대만
응답한다.

```bash
for i in $(seq 1 254); do
  ping -c1 -W1 192.168.6.$i >/dev/null 2>&1 && echo "192.168.6.$i 응답"
done
```

한 대만 나오면 IP 충돌을 의심하고 **1.4 의 mDNS 방법**으로 찾는다.

**(c) `udp_dest` 가 링크로컬 — 제일 찾기 어렵다**

증상은 드라이버가 계속 죽는 것이다.

```
[ERROR] maximum number of allowed errors from sensor::poll_client() reached, performing self reset...
[WARN]  sensor::poll_client()) returned error or timed out
```

**핑도 되고 HTTP 도 되는데 데이터만 안 온다.** 센서 설정을 보면 나온다.

```
192.168.6.11 : udp_dest 169.254.63.240     <- PC 가 아니다
```

`udp_dest: ''`(자동)로 두면 센서가 목적지를 스스로 고르는데, 인터페이스가
여럿이거나 자가 리셋이 끼면 링크로컬을 잡는다. **런치에서 명시하면 해결된다**
(2장 참고).

---

## 2. 소프트웨어 준비

### 2.1 워크스페이스

```
~/ros2_ws/src/
  dual_ouster_driver/          <- 우리 코드
  ouster-ros/                  <- 드라이버 (패치 있음)
  DronePositioningSystem/      <- COLCON_IGNORE (참고용, 빌드 안 함)
  competition_gui/ competition_score/ drone_localization/
```

```bash
cd ~/ros2_ws
colcon build --packages-select dual_ouster_driver --symlink-install
```

`--symlink-install` 은 파이썬 파일을 복사하지 않고 링크한다. 소스를 고치면
다시 빌드하지 않아도 반영된다. **단, `setup.py` 를 고치거나 파일을 새로 만들면
빌드해야 한다.**

### 2.2 ouster-ros 소스 패치 — 없으면 크래시

`src/ouster-ros/ouster-ros/src/point_cloud_compose.h`

FW 3.2 미만 센서는 `WINDOW` 필드를 안 보내는데 상위 코드가 무조건 읽어서
`Field 'WINDOW' not found in LidarScan` 으로 죽는다. **업스트림 버그다.**

```cpp
template <typename T>
const T* zero_field_buffer(std::size_t n) {
    static thread_local std::vector<T> buf;
    if (buf.size() < n) buf.assign(n, T{});
    return buf.data();
}
// map_lidar_scan_fields_to_tuple 안에서
std::get<Index>(tp) = ls.has_field(Table[Index].first)
    ? ls.field<FieldType>(Table[Index].first).data()
    : zero_field_buffer<FieldType>(std::size_t(ls.w) * std::size_t(ls.h));
```

**`ouster-ros` 를 다시 받으면 이 패치가 날아간다.** 현재 커밋 `aef1a24`.

### 2.3 DDS 설정 — 2대에서는 필수

`src/dual_ouster_driver/config/fastdds_large.xml`

한 프레임이 약 2MB. 기본 Fast DDS 는 공유메모리 공간이 작아 못 담고 UDP 로
우회하는데, 2MB 는 **약 1400조각**으로 쪼개지고 **하나만 잃어도 프레임 전체가
버려진다.**

```xml
<transport_descriptor>
  <transport_id>shm_tr</transport_id><type>SHM</type>
  <segment_size>67108864</segment_size>            <!-- 64MB -->
</transport_descriptor>
<transport_descriptor>
  <transport_id>udp_big</transport_id><type>UDPv4</type>
  <sendBufferSize>16777216</sendBufferSize>
  <receiveBufferSize>16777216</receiveBufferSize>
</transport_descriptor>
...
<useBuiltinTransports>false</useBuiltinTransports>   <!-- 기본 전송 끄기 -->
```

실측 차이:

| | 없을 때 | 있을 때 |
|---|---|---|
| 1 대 | 3.5 Hz | 10.0 Hz |
| **2 대** | **2.0 / 1.4 Hz** | **10.01 / 10.01 Hz** |

### 2.4 `~/.bashrc`

```bash
source /opt/ros/humble/setup.bash                  # 118행
source ~/ros2_ws/install/setup.bash                # 119행
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/ros2_ws/src/dual_ouster_driver/config/fastdds_large.xml   # 123행
```

**스크립트에서 `setup.bash` 만 source 하면 세 번째가 빠진다.** 실제로 이것 때문에
2대가 1.4 Hz 로 떨어진 적이 있다.

```bash
echo $FASTRTPS_DEFAULT_PROFILES_FILE     # 비어 있으면 문제
```

---

## 3. 드라이버

### 3.1 코드

`src/dual_ouster_driver/launch/dual_os1.launch.py` (118줄)

```python
SENSORS  = {1: '192.168.6.11', 2: '192.168.6.12'}
UDP_DEST = '192.168.6.100'                  # PC 주소. 1.6(c) 때문에 명시

def make_lidar(idx, host):
    node = LifecycleNode(
        package='ouster_ros', executable='os_driver',
        name=f'os_driver_lidar{idx}',
        namespace=f'lidar{idx}',            # remappings 아님. 3.2 참고
        parameters=[{
            'sensor_hostname': host,
            'lidar_mode': '1024x10',
            'timestamp_mode': 'TIME_FROM_ROS_TIME',
            'point_cloud_frame': f'lidar{idx}',
            'point_type': 'xyz',            # 좌표만. 강도 등은 안 씀
            'proc_mask': 'PCL|IMU',
            'lidar_port': 0, 'imu_port': 0, # 0 = 빈 포트 자동. 2대면 필수
            'udp_dest': UDP_DEST,
            'attempt_reconnect': True,
        }])
    configure = EmitEvent(...TRANSITION_CONFIGURE)
    activate  = RegisterEventHandler(OnStateTransition(goal_state='inactive', ...))
    failed    = RegisterEventHandler(OnStateTransition(goal_state='finalized', ...Shutdown))
    return [node, configure, activate, failed]
```

```bash
ros2 launch dual_ouster_driver dual_os1.launch.py
ros2 launch dual_ouster_driver dual_os1.launch.py enable_lidar2:=false
```

### 3.2 왜 이렇게 생겼나 — 함정 셋

**(1) 라이프사이클 노드다**

`os_driver` 는 평범한 노드가 아니라 `LifecycleNode` 다. 그냥 띄우면
`unconfigured` 상태로 멈춘 채 **CPU 0%, 로그도 없이** 죽은 것처럼 보인다.
`configure` → `activate` 전이를 직접 쏴야 데이터가 나온다.

**(2) `namespace` 로 토픽을 잡는다. `remappings` 는 안 먹는다**

이 버전 ouster-ros 는 **상대 이름**으로 발행해서 `/ouster/points` 같은 절대 경로
remap 이 매칭되지 않는다. 처음에 이것 때문에 토픽이 `/points` 로 나왔다.

**(3) 포트를 0 으로 둔다**

`lidar_port: 0` 은 "빈 포트를 알아서 골라라"다. 2대를 띄울 때 포트를 고정하면
서로 부딪힌다.

### 3.3 중복 실행 — 이 에러가 나오면 거의 항상 이것

```
[ERROR] Failed to make transition 'TRANSITION_CONFIGURE' for LifecycleNode '/lidar1/os_driver_lidar1'
```

같은 드라이버가 이미 돌고 있는데 또 띄운 것이다. 센서는 **동시에 한 클라이언트만**
받는다.

```bash
ros2 node list                                    # 띄우기 전에 확인
ps -ef | grep os_driver | grep -v grep
```

중복이 생기면 **둘 다 끄고 하나만 새로 띄운다.** 한쪽만 죽이려 하면 어느 쪽이
살아남을지 예측이 안 된다(실제로 원래 것까지 같이 내려갔다).

드라이버는 **한 번 켜면 계속 두는 게 맞다.** 재시작에 15초 걸린다.

### 3.4 출력

```
/lidarN/points     sensor_msgs/PointCloud2  frame_id=lidarN  BEST_EFFORT  10Hz  ~2MB
/lidarN/imu        sensor_msgs/Imu
/tf_static         lidarN_sensor -> lidarN       (180도 z회전, z +0.038195)
                   lidarN_sensor -> lidarN_imu
```

`lidarN_sensor` 와 `lidarN` 은 **180도 돌아가 있다.** RViz 에서 Fixed Frame 을
어느 쪽으로 두느냐에 따라 좌표 부호가 뒤집히니 주의.

### 3.5 확인

```bash
ros2 topic hz /lidar1/points
ros2 topic hz /lidar2/points
```

> `ros2 topic hz` 에는 **QoS 옵션이 없다**(Humble). `--qos-reliability` 는
> `echo` 에만 있다. 그냥 `hz` 를 쓰면 된다.

---

## 4. 보정 — 비행 전 1회

### 4.1 코드

`src/dual_ouster_driver/dual_ouster_driver/calibrate.py` (1428줄)

```bash
ros2 run dual_ouster_driver calibrate --ros-args \
  -p lidar_ids:="[1,2]" -p front_axis:=+y -p marker_radius:=0.045 -p countdown:=10
```

### 4.2 단계

| 단계 | 알고리즘 | 결과 |
|---|---|---|
| [1/4] 정면 | 방위별 최소거리 비교 (사람이 서기 전/후) | `yaw_base` |
| [2/4] 높이 | 거리 고리별 z 히스토그램 최빈값, 2회 반복 | `lidar_z` |
| [3/4] M1 | 적응형 바닥컷 → DBSCAN(eps .15) → 원 피팅(반지름 고정) | M1 좌표 |
| [4/4] M2 | 같은 방식 + 다중프레임 그룹화·이상값 제거 | M2 좌표 |
| 계산 | 2D 강체변환 해석해 | `yaw_deg`, `origin_x/y` |
| ROI | 벽 탐색 | `roi_*` |

```python
target = FRONT_AXIS_ROT[front]                        # +y 이면 90
theta  = target - degrees(atan2(dv[1], dv[0]))
yaw    = yaw_base + theta
origin = rot2(theta) @ P1
```

### 4.3 2대 동시 보정

따로 하면 마커를 **4번** 놓는다. 두 번째에 2cm 어긋나면 **두 좌표계가 2cm
어긋난 채 굳고, 나중에 드러나지도 않는다.** 동시에 하면 **2번**만 놓는다.

라이다2 는 정면 개념이 달라 360도로 훑는데 후보가 10개 넘게 나온다. 그래서
**"M1 자리에서 M2 자리로 옮겨간 물체는 마커뿐"** 이라는 성질로 자동 판별한다
(`match_moved`). 합성 시험에서 후보 7개씩 있어도 짝이 1개만 나왔고, 마커를 안
옮긴 경우엔 0개가 나와 실수도 잡아냈다.

**주의: M1·M2 측정 사이에 마커 말고 아무것도 움직이면 안 된다.**

### 4.4 교차검증 — 기준값 없이 할 수 있는 최선

```
[교차검증] 각 라이다가 잰 M1-M2 간격
  라이다1 2.353 m   라이다2 2.363 m
  최대 차이 10.0 mm
```

20mm 이내면 좋다. 두 라이다 사이 거리도 출력하니 줄자로 맞춰본다.

### 4.5 출력과 현재 값

`config/calibration_lidarN.yaml`

```yaml
/**:                          # <- 와일드카드. 5.3 참고
  ros__parameters:
    lidar_z: 0.8388
    yaw_deg: 271.0766
    origin_x: -1.8563
    origin_y: 1.5243
    roi_x_min: -0.20 / roi_x_max: 3.90
    roi_y_min: -1.30 / roi_y_max: 3.10
    roi_z_min: 0.30  / roi_z_max: 2.50
```

```
라이다1  lidar_z 0.8388  yaw 271.0766  origin (-1.8563, +1.5243)  정확도 0.086도
라이다2  lidar_z 0.8917  yaw  94.8580  origin (-1.8150, -3.4120)  정확도 0.110도

두 라이다 사이 4.936 m,  마주 봄 (176.2도 차이),  높이차 5.3 cm
```

**라이다를 건드리면 다시 돌려야 한다.**

---

## 5. ROI 필터 — 보정값을 실제로 적용하는 곳

### 5.1 코드

`src/dual_ouster_driver/dual_ouster_driver/roi_filter.py` (241줄), 라이다마다 하나씩

```python
pts = pts[norm(pts) >= min_range]                  # 1. 자기 몸통 반사 제거 (0.1m)
pts[:, :2] = pts[:, :2] @ rot.T - origin           # 2. 회전 후 평행이동
pts[:, 2] += lidar_z                               #    바닥이 z=0 이 되게
keep = all((pts >= lo) & (pts <= hi), axis=1)      # 3. 축정렬 상자 크롭
for cx, cy in excl:                                # 4. (기본 꺼짐) 라이다 자리 제외
    keep &= hypot(pts[:,0]-cx, pts[:,1]-cy) > excl_r
header.stamp = msg.header.stamp                    # 5. 원본 시각 그대로 전달
```

**순서가 중요하다.** 2번은 "돌리고 나서 빼기"다. 반대로 하면 다른 결과가 나온다.

**5번이 특히 중요하다.** 처리 시각으로 덮으면 지연 200ms 가 그대로 위치 오차가
된다. 5 m/s 면 1 m.

파라미터를 **매 프레임 다시 읽어서** 실행 중 조정이 즉시 먹는다.

```bash
ros2 param set /roi_filter_1 roi_z_min 0.05
```

### 5.2 실측

```
입력 123,750점  ->  출력 10~40점 (물체 하나)
처리 12.2 ms / 최대 16 ms,   지연 204~206 ms
```

### 5.3 문제 — params 파일이 조용히 무시된다

**가장 오래 헤맨 것.** RViz 에서 두 점군이 2m 어긋나고 대칭으로 보였다.

```
실제 실행중 : lidar_z 0.84,  yaw 180.0,  origin (0,0)     <- 전부 기본값
YAML 에는   : lidar_z 0.827, yaw 270.61, origin (-1.85, 1.53)
```

**ROS 2 파라미터 파일은 노드 이름으로 매칭된다.** YAML 키가 `roi_filter:` 인데
런치에서 노드를 `roi_filter_1` 로 지어서 **경고 하나 없이 통째로 무시**됐다.

두 겹으로 막았다.

**(1) `/**:` 와일드카드 키** — 이름·이름공간과 무관하게 매칭. `roi_filter_99` 로
띄워도 먹는 것을 실측 확인했다.

**(2) 경고** — 원점이 정확히 (0,0) 이면 경고한다. 보정하면 정확히 0.000 이 나올
일은 사실상 없다.

```
[WARN] origin 이 정확히 (0,0) 입니다 — 보정값이 안 실렸을 수 있습니다.
```

**확인 습관**: 띄운 뒤 이 한 줄을 본다.

```
[roi_filter_1]: ROI filter: /lidar1/points -> /lidar1/roi_points [map],
                lidar_z=0.84, yaw=271.08deg, origin=(-1.856, 1.524)
```

---

## 6. 군집화 — 위치 하나 뽑기

### 6.1 코드

`src/dual_ouster_driver/dual_ouster_driver/cluster_median.py` (335줄), 라이다마다 하나씩

```python
v = voxel(pts, voxel_size)                    # 0.0 = 꺼짐. 작은 드론이 사라져서 껐다
labels = DBSCAN(eps=0.20, min_samples=3).fit_predict(v)
for label in unique(labels):
    c = v[labels == label]
    if not (min_points <= len(c) <= max_points): continue
    ext = c.max(0) - c.min(0)
    if max(ext[0], ext[1]) > max_span: continue     # 벽 잔재 거르기
    out.append({'median': np.median(c, axis=0), ...})
out = [max(out, key=lambda d: d['n'])]              # 가장 큰 군집 하나
```

**median 이지 평균이 아니다.** 평균은 군집에 섞인 이상점 하나에 끌려가지만
median 은 거의 안 움직인다.

발행 토픽은 입력 이름공간에서 딴다 (`/lidar2/roi_points` → `/lidar2/cluster_*`).

### 6.2 실측

```
처리 4~15 ms,  지연 216~220 ms
```

### 6.3 문제 — 기억이 없다

**매 프레임 독립적으로 "가장 큰 군집"을 고를 뿐, 직전 프레임을 안 본다.**

run2 비행에서 실제로 이렇게 나왔다.

```
z >= 2.40 (천장)   372행  (68%)      <- 대부분 천장을 잡고 있었다
z <  2.40 (드론)   173행  (32%)
27초에 속도 5.26 m/s 스파이크         <- 드론에서 천장으로 갈아탄 순간
```

**"검출 99.6%" 가 사실상 거짓이었다.** → 8.2

---

## 7. 기록·분석

### 7.1 flight_recorder (266줄)

```bash
ros2 launch dual_ouster_driver record_flight.launch.py label:=run6
```

`/lidarN/cluster_poses` 구독 → 6상태 등속도 칼만필터 → `track.csv`

```python
class KF6:   # [x y z vx vy vz]
    def predict(self, dt): F[0,3]=F[1,4]=F[2,5]=dt; x=F@x; P=F@P@F.T + Q*dt
    def update(self, z):   K=P@H.T@inv(H@P@H.T+R); x=x+K@(z-H@x)
```

- 원본(median)과 필터값을 **둘 다** 기록. 어느 쪽이 나은지는 데이터 보고 판단
- 놓친 프레임도 `detected=0` 으로 남긴다. 언제 놓쳤는지 알아야 하므로
- 2초 넘게 못 잡으면 필터 초기화
- rosbag 도 같이 (`/lidarN/points` + `/lidarN/roi_points`)

**필터는 실시간으로 돌지만 어디에도 발행되지 않는다.** CSV 와 2초 로그로만 나온다.
그래서 RViz 구슬은 median 이지 칼만이 아니다.

### 7.2 flight_report (338줄)

```bash
ros2 run dual_ouster_driver flight_report --ros-args -p run_dir:=~/flights/flight_xxx
```

`track.csv` 를 **읽기만** 한다. DBSCAN 도 칼만도 다시 안 돈다.

끊긴 구간 / 이동 범위·거리 / 속도 통계 / 원본 vs 칼만 차이 / 호버 구간
→ `report.txt`, `flight_3d.html`(plotly), `flight_2d.png`(matplotlib)

파라미터를 바꿔 다시 계산하려면 **rosbag 재생**밖에 없다. 그래서 원본을 남긴다.

### 7.3 문제 — 2대를 못 쓴다

`record_flight.launch.py` 는 `lidar_id` 하나만 받고 `flight_recorder` 도 토픽
하나만 구독한다. **지금 비행을 찍으면 라이다 한 대 결과만 남는다.**

---

## 8. 문제 목록 — 심각한 것부터

### 8.1 두 스트림을 합치는 코드가 없다 **(최우선)**

두 라이다가 **각자 따로** 위치를 낸다. 2대를 붙인 이점을 하나도 못 쓰고 있다.

좌표계가 잘 맞는 것은 확인됐다.

```
물체 크기 13~27cm
  라이다1 중심 (+0.769, +1.030, 0.449)
  라이다2 중심 (+0.761, +1.105, 0.475)
  축 수직  +0.8 cm   <- 좌표계 일치도. 훌륭
  축 방향  +7.5 cm   <- surface bias. 두 중점이 참 중심
```

**(A) 점군 병합 후 한 번에 군집화 (권장)**

```
/lidar1/roi_points \
                    +-> [fuse] -> /fused/points -> cluster_median -> /fused/cluster_poses
/lidar2/roi_points /
```

양쪽 면을 다 가진 점군이라 median 이 자연스럽게 중심에 온다(surface bias 소멸).
한쪽이 사각 원뿔에 가려도 다른 쪽 점이 살아있어 추적이 안 끊긴다.
점 수가 2배지만 지금 4~15ms 라 여유 있다.

**(B) 위치 평균** — 간단하고 한쪽이 놓쳐도 버티지만, 한쪽만 잡은 프레임에서
surface bias 가 그대로 남는다.

**어느 쪽이든 타임스탬프 정렬이 필요하다.** 두 라이다는 동기되지 않았고 각각
203~217ms 지연이 있다. `message_filters.ApproximateTimeSynchronizer`,
슬롭은 프레임 간격의 절반인 **50ms**.

### 8.2 추적 연관(association)이 없다

`cluster_median` 이 매 프레임 "가장 큰 군집"을 고를 뿐이라 사람·천장으로 갈아탄다.

**고치는 법**: 직전 추정 위치 주변 게이트 안의 군집만 후보로 삼는다.

```
predict -> 게이트(반경 0.8m) -> 그 안 군집만 후보 -> 가장 가까운 것
게이트 안에 없으면 '놓침' 처리하고 예측만 이어간다
```

지금 칼만이 `flight_recorder` 안에 있어서 `cluster_median` 이 못 쓴다.
**필터를 앞단으로 옮기거나 융합 노드에 넣어야 한다.**

### 8.3 기울기 — 라이다1 이 아직 0.75도

```
라이다1   1.15도 -> 0.75도   (덜 잡힘)
라이다2   2.24도 -> 0.25도   (잘 잡음)
```

보정은 **"바닥이 수평"** 가정 위에 있다. 기울면 **z 가 거리에 비례해 틀어진다.**
x, y 는 거의 영향 없다.

| 기울기 | 4 m | 10 m | 20 m |
|---|---|---|---|
| 0.75도 | 5.2 cm | 13.1 cm | 26.2 cm |
| 0.25도 | 1.7 cm | 4.4 cm | 8.7 cm |

```bash
ros2 run dual_ouster_driver tilt_check      # 2초마다 갱신. 돌리면서 본다
```

수평을 바꾸면 `lidar_z` 가 달라지므로 **재보정 필수.**

### 8.4 조용한 실패가 아직 남아 있다

**roi_filter 기본값이 그럴듯하다.** `lidar_z 0.84`, `yaw_deg 180.0` 은 1대 시절
실측치라 보정이 안 실려도 로그가 정상처럼 보인다. 원점 (0,0) 경고로 막긴 했지만,
기본값을 **명백히 틀린 값**으로 바꾸는 게 맞다.

```python
self.declare_parameter('lidar_z', 0.0)     # 0 = 바닥이 라이다 높이 = 불가능
self.declare_parameter('yaw_deg', 0.0)
```

**ROI 가 두 YAML 에 중복돼 있다.** 같은 비행 영역인데 파일이 둘이라 한쪽만
고치면 두 라이다가 다른 영역을 본다. 실제로 `roi_y_min` 을 양쪽 다 바꿔야 했다.
→ 공통 `roi.yaml` 로 분리하는 게 낫다.

### 8.5 라이다가 서로를 물체로 잡는다

마주 보게 놓으면 상대가 ROI 안에 들어온다.

```
라이다2 가 잡은 것 :  55점  (+1.84,-1.48,0.83)  6x5x6cm
라이다1 의 실제 위치:  (+1.856,-1.524)          <- 5 cm 차이
```

지금은 `roi_y_min` 을 `-1.5 → -1.30` 으로 올려 피했다(**비행 영역 20cm 감소**).

제외 반경 기능이 있지만 **기본 꺼짐**이다.

```bash
ros2 launch dual_ouster_driver dual_view.launch.py exclude_radius:=0.5
```

### 8.6 지연 200ms — 원인 미상

```
센서 촬영 -> roi_filter 도착   204~206 ms
센서 촬영 -> cluster 도착      216~220 ms
```

roi_filter 처리 12ms, cluster 4~15ms 이므로 **우리 코드가 아니다.** 드라이버
내부 버퍼링이나 DDS 쪽으로 보이는데 **측정해본 적이 없다.**

기록용으로는 타임스탬프가 정확하니 문제없지만, **실시간 제어에 쓰려면 파야 한다.**
5 m/s 면 200ms = 1 m.

### 8.7 기준값이 없다

지금 수치는 전부 **"라이다가 무엇을 봤는가"** 이지 **"그게 맞는가"** 가 아니다.
두 라이다 교차검증(8mm)은 일관성이지 정확도가 아니다.

실내는 모캡, 야외는 RTK GPS 로 비교해야 한다.

### 8.8 나머지

- 드론/사람 판별 없음
- 대회장 규모(24x15 m) 미검증 — 지금은 4x4 m 실내
- `record_flight.launch.py` 가 1대 전용
- `verify_pose` 는 만들었으나 한 번도 안 씀

---

## 9. 할 일 순서

```
1.  fuse 노드                   <- 2대의 이점을 쓰려면 이것부터
      (A) 점군 병합,  ApproximateTimeSynchronizer 슬롭 50ms
2.  게이팅                      <- 사람/천장으로 갈아타는 것 막기
      칼만을 앞단으로 옮기고 예측 위치 주변만 후보로
3.  record_flight 2대 대응       <- 융합 토픽 구독
4.  라이다1 수평 (0.75 -> 0.2도) <- 대회장 가기 전 필수
5.  기준값 붙이기                <- 모캡
6.  지연 200ms 원인 추적         <- 실시간 제어 쓸 거면
```

1~3 이 하나로 묶인다. **융합 노드에 칼만과 게이팅을 같이 넣는 게 자연스럽다.**

```
/lidar1/roi_points \
                    +-> [tracker] -> /fused/cluster_poses  (median)
/lidar2/roi_points /              -> /fused/track          (칼만 + 속도)
```

---

## 10. 전체 실행 순서

```bash
# ── 0. 네트워크 (1.5 확인 사다리)
cat /sys/class/net/enp59s0/carrier          # 1 이어야 함
ip -br addr show enp59s0                    # 192.168.6.100/24
ping -c2 192.168.6.11 && ping -c2 192.168.6.12
curl -s http://192.168.6.11/api/v1/sensor/config | grep udp_dest   # PC 주소여야

# ── 1. 환경
echo $FASTRTPS_DEFAULT_PROFILES_FILE        # 경로가 나와야 함
ros2 node list                              # 비어 있어야 함 (중복 실행 방지)

# ── 2. 드라이버 (한 번 켜면 계속 둔다. 재시작 15초)
ros2 launch dual_ouster_driver dual_os1.launch.py
ros2 topic hz /lidar1/points                # 10 근처

# ── 3. 수평 (라이다 건드렸으면)
ros2 run dual_ouster_driver tilt_check

# ── 4. 보정 (라이다 옮겼으면 필수)
ros2 run dual_ouster_driver calibrate --ros-args \
  -p lidar_ids:="[1,2]" -p front_axis:=+y -p marker_radius:=0.045 -p countdown:=10

# ── 5. 겹침 확인
ros2 launch dual_ouster_driver dual_view.launch.py z_min:=0.05

# ── 6. 비행 기록  (주의: 아직 1대만 기록된다)
ros2 launch dual_ouster_driver record_flight.launch.py label:=run6

# ── 7. 분석
ros2 run dual_ouster_driver flight_report --ros-args -p run_dir:=~/flights/flight_xxx
```

### 정리할 때

```bash
# pkill -f roi_filter 는 쓰지 말 것. 명령줄 자체가 패턴에 걸려 자기 자신도 죽인다
ps -eo pid,cmd | grep roi_filter | grep -v grep
kill -INT <PID>

# kill -9 후에는 그래프가 남는다
ros2 daemon stop && rm -f /dev/shm/fastrtps_*
```

---

## 11. 파일 목록

### 실행 코드

| 파일 | 줄 | 역할 |
|---|---|---|
| `launch/dual_os1.launch.py` | 118 | 드라이버 2대 |
| `launch/dual_view.launch.py` | 146 | roi_filter x2 + cluster_median x2 + TF + RViz |
| `launch/record_flight.launch.py` | 95 | 비행 기록 (**1대 전용**) |
| `dual_ouster_driver/roi_filter.py` | 241 | 보정 적용 + ROI 크롭 |
| `dual_ouster_driver/cluster_median.py` | 335 | DBSCAN + median |
| `dual_ouster_driver/calibrate.py` | 1428 | 보정 (단일/다중) |
| `dual_ouster_driver/flight_recorder.py` | 266 | 칼만 + CSV |
| `dual_ouster_driver/flight_report.py` | 338 | 분석·그림 |
| `dual_ouster_driver/tilt_check.py` | 149 | 수평 실시간 확인 |
| `dual_ouster_driver/verify_pose.py` | 334 | 줄자 대비 검증 (미사용) |

### 설정

| 파일 | 역할 |
|---|---|
| `config/calibration_lidar1.yaml` | 라이다1 보정값 + ROI |
| `config/calibration_lidar2.yaml` | 라이다2 보정값 + ROI |
| `config/fastdds_large.xml` | DDS SHM 프로파일 (**2대에서 필수**) |
| `rviz/dual_view.rviz` | 2대 겹쳐 보기 |
| `rviz/roi_view.rviz` | 1대용 |
| `rviz/calib_view.rviz` | 보정 확인용 (Fixed Frame 을 `lidar1` 로 바꿔야 함) |

### 시스템에 있는 것 — 다른 PC 로 옮길 때 빠뜨리기 쉬움

| 위치 | 내용 |
|---|---|
| NetworkManager `ouster` 프로파일 | `enp59s0` 에 `192.168.6.100/24` 고정 |
| `~/.bashrc` 118~123행 | ROS source + `FASTRTPS_DEFAULT_PROFILES_FILE` |
| `src/ouster-ros/.../point_cloud_compose.h` | `WINDOW` 필드 패치 (**재설치 시 날아감**) |
| 센서 내부 | 각 센서의 `ipv4.override` (`.11` / `.12`) |

### 안 쓰는 것

`axis_probe.py`, `room_survey.py`, `marker_fit.py`, `pose_solve.py` —
보정 도구를 만들면서 거쳐간 중간 산물. `calibrate.py` 로 흡수됐다.
