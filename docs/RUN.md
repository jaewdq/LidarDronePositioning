# 실행 명령 모음

설치는 [LIDAR_SETUP.md](LIDAR_SETUP.md), 원리는 [CALIBRATION.md](CALIBRATION.md),
라이다 2대 구성은 [DUAL_LIDAR.md](DUAL_LIDAR.md),
전체 파이프라인과 미해결 문제는 [PIPELINE.md](PIPELINE.md).
이 문서는 **실제로 치는 명령만** 모아둔 것이다. 아래는 라이다 한 대 기준이다.

`~/.bashrc` 에 아래가 이미 들어 있으므로 **새 터미널은 그냥 쓰면 된다.**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/ros2_ws/src/dual_ouster_driver/config/fastdds_large.xml
```

> `FASTRTPS_DEFAULT_PROFILES_FILE` 를 빼먹으면 **10 Hz 가 3.5 Hz 로 떨어진다.**
> 2 MB 짜리 메시지가 UDP 로 1400 조각 나면서 유실되기 때문이다.

---

## 0. 연결 확인

```bash
ping -c 3 192.168.6.11
curl -s http://192.168.6.11/api/v1/sensor/metadata/sensor_info
```

응답이 없으면 IP 부터.

```bash
nmcli con up ouster
# 또는 임시로
sudo ip addr add 192.168.6.100/24 dev enp59s0
```

라이다 IP 를 모를 때:

```bash
for i in $(seq 1 254); do (ping -c1 -W1 192.168.6.$i >/dev/null 2>&1 && echo "192.168.6.$i 응답") & done; wait
```

---

## 1. 드라이버 (터미널 1)

```bash
ros2 launch dual_ouster_driver dual_os1.launch.py
```

**한 번 켜면 계속 둔다.** 재시작에 약 15초 걸리므로 비행 사이에 껐다 켜지 않는다.

정상이면 이런 로그가 나온다.

```
[INFO] Contacting sensor 192.168.6.11 ...
[INFO] Sensor 192.168.6.11 configured successfully
[INFO] os_driver_lidar1 activating...
```

확인:

```bash
ros2 topic list | grep lidar1
ros2 topic echo /lidar1/points --field width --once
```

```bash
ros2 topic hz /lidar1/points
```

`10.0` 근처가 나와야 한다. **3.5 쯤 나오면** `FASTRTPS_DEFAULT_PROFILES_FILE`
이 안 잡힌 것이다 (`echo $FASTRTPS_DEFAULT_PROFILES_FILE` 로 확인).

---

## 2. 보정 (라이다를 옮겼을 때만)

지름을 아는 원기둥 마커 하나를 두 자리에 차례로 놓는다.

```bash
# 터미널 2 — 드라이버는 켜둔 채로
ros2 run dual_ouster_driver calibrate --ros-args -p front_axis:=+y
```

자주 쓰는 인자:

```bash
# 마커 지름을 줄자로 쟀을 때 (반지름 m). 정확도가 확 올라간다
-p marker_radius:=0.15

# 바닥이 기울어 자동 측정이 이상할 때 직접 넣기
-p lidar_z:=1.85

# 정면 확인 단계 건너뛰기 (이미 아는 경우)
-p skip_front_check:=true -p yaw_base:=180.0

# 물체 탐색 범위 (기본 앞 4 m, 좌우 ±2 m)
-p search_fwd_max:=6.0 -p search_side_max:=3.0

# 프레임 수를 늘리면 안정적이지만 오래 걸린다 (기본 25)
-p n_frames:=40

# 라이다 2호기
-p lidar_id:=2
```

끝나면 `src/dual_ouster_driver/config/calibration_lidar1.yaml` 이 갱신된다.
**빌드해야 반영된다.**

```bash
cd ~/ros2_ws && colcon build --packages-select dual_ouster_driver --symlink-install
source ~/ros2_ws/install/setup.bash
```

보는 눈이 필요하면:

```bash
rviz2 -d ~/ros2_ws/src/dual_ouster_driver/rviz/calib_view.rviz
```

---

## 3. 비행 녹화 (터미널 2)

```bash
ros2 launch dual_ouster_driver record_flight.launch.py label:=test1
```

`roi_filter` + `cluster_median` + `flight_recorder` + `ros2 bag` 이 한꺼번에 뜬다.
보정값은 YAML 에서 자동으로 읽으므로 **따로 넣을 필요가 없다.**

**비행이 끝나면 이 터미널에서 Ctrl+C.** 요약이 뜨고 파일이 닫힌다.

| 인자 | 기본값 | 뜻 |
|---|---|---|
| `label` | (없음) | 폴더 이름 꼬리표 |
| `lidar_id` | `1` | 어느 라이다 |
| `out_dir` | `~/flights` | 저장 위치 |
| `bag` | `both` | `raw`(20 MB/s) / `roi`(16 KB/s) / `both` / `none` |
| `config` | (자동) | 다른 보정 YAML |

```bash
# 디스크가 빠듯할 때
ros2 launch dual_ouster_driver record_flight.launch.py label:=t2 bag:=roi

# 라이다 2호기로
ros2 launch dual_ouster_driver record_flight.launch.py lidar_id:=2 label:=t3
```

동시에 보고 싶으면 터미널 3 에서:

```bash
rviz2 -d ~/ros2_ws/src/dual_ouster_driver/rviz/roi_view.rviz
```

---

## 4. 결과 보기

```bash
ros2 run dual_ouster_driver flight_report --ros-args \
  -p run_dir:=$HOME/flights/flight_20260823_200642_test1
```

ROS 없이도 된다.

```bash
python3 ~/ros2_ws/src/dual_ouster_driver/dual_ouster_driver/flight_report.py \
  ~/flights/flight_20260823_200642_test1
```

만들어지는 것:

```
track.csv        프레임마다 한 줄 — 원본/칼만/속도/군집 점수
meta.txt         기록 시각, 프레임 수, 검출률, 놓친 구간
report.txt       요약
flight_3d.html   돌려볼 수 있는 3D 궤적
flight_2d.png    top-down / 고도 / 속도
```

```bash
xdg-open ~/flights/flight_20260823_200642_test1/flight_3d.html
```

---

## 5. 되감아 다시 돌리기

파라미터를 바꿔가며 몇 번이고 다시 분석할 수 있다. **`bag:=raw` 나 `both` 로
녹화했을 때만 가능하다.**

```bash
# 터미널 1 — 드라이버 대신 bag 을 튼다
ros2 bag play ~/flights/bag_20260823_200642_test1 --loop

# 터미널 2 — 파라미터만 바꿔서
ros2 run dual_ouster_driver roi_filter --ros-args \
  --params-file ~/ros2_ws/install/dual_ouster_driver/share/dual_ouster_driver/config/calibration_lidar1.yaml \
  -p roi_z_min:=0.20

ros2 run dual_ouster_driver cluster_median --ros-args \
  -p input_topic:=/lidar1/roi_points -p min_points:=3
```

---

## 6. 파라미터 실시간 조정

`roi_filter` 는 **매 프레임 파라미터를 다시 읽는다.** 껐다 켤 필요 없다.

```bash
ros2 param set /roi_filter roi_z_min 0.20
ros2 param set /roi_filter roi_x_max 5.0
ros2 param get /roi_filter roi_x_min
ros2 param dump /roi_filter
```

표적이 작아 안 잡힐 때 (`cluster_median` 은 재시작해야 한다):

```
voxel_size   0.0    복셀을 켜면 점 수십 개짜리 표적이 통째로 사라진다
min_points   4      멀리 있는 텔로는 4~6점밖에 안 잡힌다
min_samples  3
max_span     1.5    이보다 크면 벽 잔재로 본다
```

---

## 7. 진단

```bash
ros2 node list
ros2 topic list
ros2 topic info /lidar1/roi_points --verbose
ros2 topic hz /lidar1/points
```

**`kill -9` 로 죽였다면 반드시 정리한다.** 안 그러면 `ros2 node list` 가
유령 노드를 계속 보여준다.

```bash
ros2 daemon stop
rm -f /dev/shm/fastrtps_*
ros2 daemon start
```

노드 끄기 — **`pkill -f roi_filter` 는 자기 자신도 죽인다** (명령줄이 패턴에
걸린다). PID 로 끈다.

```bash
ps -eo pid,cmd | grep dual_ouster_driver | grep -v grep
kill -INT <PID>
```

---

## 8. 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select dual_ouster_driver --symlink-install
source install/setup.bash
```

`--symlink-install` 이라 **파이썬 파일은 고쳐도 다시 빌드할 필요가 없다.**
단, `setup.py`, 런치 파일, `config/*.yaml` 을 건드리면 다시 빌드해야 한다.

전체 빌드 (ouster-ros 포함, 처음 한 번):

```bash
cd ~/ros2_ws
touch src/DronePositioningSystem/COLCON_IGNORE
colcon build --symlink-install
```
