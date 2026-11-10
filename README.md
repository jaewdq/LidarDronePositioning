# 실내 LiDAR UAV 추적 — 운용 매뉴얼 (Ouster OS0-SR × 2)

## 전제조건

- Ubuntu 22.04
- ROS 2 Humble (ros-humble-desktop) — 설치: https://docs.ros.org/en/humble/Installation.html
- 나머지(의존성, Ouster 드라이버, PTP, 우리 패키지 빌드, 네트워크 설정)는 setup.sh 가 한다.

※ 아래 경로는 예시다

- 워크스페이스: `~/ros2_ws` (setup.sh 가 물어볼 때 바꿀 수 있음)
- 저장소: `~/drone_project/LidarDronePositioning`
- 라이다 랜카드: `eno1` (PC 마다 다름, `ip -br addr` 로 확인)
- 경로를 바꿨으면 아래 명령의 경로도 같이 바꾼다.

---

## 0. 설치 (처음 한 번)

### 0-1. 저장소, 설치 스크립트

- Ouster 드라이버는 공식 ouster-ros(ros2 브랜치)를 쓴다: https://github.com/ouster-lidar/ouster-ros
- setup.sh 가 받아서 커밋을 고정하고 패치까지 적용한다.

```bash
git clone https://github.com/jaewdq/LidarDronePositioning.git ~/drone_project/LidarDronePositioning
cd ~/drone_project/LidarDronePositioning && ./setup.sh
```

- 묻는 것: 워크스페이스, 라이다 랜카드, PC IP(192.168.6.100/24), 라이다 IP(.11 / .12)
- 하는 것: 의존성·linuxptp 설치, ouster-ros 와 우리 패키지 3개(드라이버, 트래커, 캘리브레이션) 빌드, 네트워크 프로필(ouster), `~/.bashrc` 3줄 추가 (ROS, 워크스페이스, 대용량 점군용 DDS 설정)
- 끝나면 새 터미널을 연다.

### 0-2. 센서 IP (새 센서일 때만)

- 두 센서가 같은 IP 로 출하되기도 한다. 이때는 IPv6 링크로컬로 붙어서 바꾼다.

```bash
avahi-browse -art | grep -iA3 ouster          # 시리얼, IPv6 주소 확인
curl -s -g -X PUT "http://[<IPv6>%eno1]/api/v1/system/network/ipv4/override" \
  -H 'Content-Type: application/json' -d '"192.168.6.12/24"'
```

- 바꾸기 전에 시리얼을 확인한다. LiDAR1 = .11, LiDAR2 = .12

---

## 개념

### PTP

- PC 가 기준 시계(grandmaster), 라이다 2대가 PC 를 따라간다. 오차 약 2 µs.
- 점군 시각 = 센서가 PTP 시계로 잰 스캔 시작 시각 (`TIME_FROM_PTP_1588`).
- ptp4l 이 꺼지면 센서가 기준을 잃는다. 추적 내내 켜 둔다.

### 위상 고정 (phase lock)

- PTP 는 시계만 맞추고 회전은 맞추지 않는다.
- 위상 고정을 켜면 두 센서가 매 0.1 s 정각에 같은 방위를 지난다. 스캔 시작이 0.5 ms 안으로 맞는다.
- 드라이버 launch 에 기본으로 켜져 있다 (`phase_lock:=true`, offset 0, 1024x10).
- 마주 본 두 센서는 필드 가운데를 동시에 지나고, 양 끝에서는 최대 약 23 ms 차이가 난다.

### 융합 방식

- PF (점군 융합): 두 점군을 합친 뒤 군집을 한 번 찾는다.
- MF (관측 융합): 센서마다 위치를 찾고, 불확실성 가중으로 합쳐 KF 를 한 번 갱신한다. **운용 기본값.**
- SU (순차 갱신): 센서마다 찾은 위치로 KF 를 차례로 두 번 갱신한다.

---

## 1. 전원, 연결

- 라이다 인터페이스 박스에 전원을 넣는다.
- 랜선은 스위치를 거쳐 PC 라이다 랜카드(eno1)에 연결한다.

## 2. 배치

- 삼각대 2대를 필드 긴 변 바깥에 마주 보게 둔다. 높이 약 1.25 m. (현재 LiDAR1 (1.35, −1.34), LiDAR2 (1.38, 3.05))
- 각 라이다의 −x 방향(커넥터 반대쪽)이 필드와 상대 센서를 향하게 한다.
- 경사계로 수평을 맞춘다. 남은 기울기는 3단계 지면 보정이 잡는다.

## 터미널 구성

| 터미널 | 내용 | 비고 |
|---|---|---|
| T1 | PTP | 끝까지 켜 둔다 |
| T2 | 드라이버 | 끝까지 켜 둔다 |
| T3 | 캘리브레이션 GUI → 추적 GUI | 하나씩 |
| T4 | 확인 (topic hz, RViz) | 필요할 때 |

- 끌 때는 역순(T3 → T2 → T1)으로 Ctrl+C.

T1 — PTP

```bash
cd ~/drone_project/LidarDronePositioning && sudo ptp4l -f ptp_gm.conf -i eno1 -m
# "assuming the grand master role" 이면 정상
```

T2 — 드라이버 (위상 고정 기본 켬)

```bash
cd ~ && ros2 launch dual_ouster_driver dual_lidar.launch.py
```

T4 — 확인

```bash
ros2 topic hz /lidar1/points ; ros2 topic hz /lidar2/points      # 각각 약 10 Hz
curl -s http://192.168.6.11/api/v1/time/ptp | python3 -m json.tool | grep port_state        # SLAVE
curl -s http://192.168.6.11/api/v1/sensor/config | python3 -m json.tool | grep phase_lock    # true, 0
# .12 도 같은 방법
```

---

## 3. 캘리브레이션 (T3, 센서를 옮길 때마다)

```bash
ros2 run grid_sensor_calibration calibration_gui
```

1. 처음 화면: 센서 2개 선택, 메모 입력, [새 세션 만들기]
    - 결과는 `~/grid_calibration/indoor_cage/<날짜>_rNN/` 에 쌓인다.
2. [1. 지면 보정]: 센서 앞 바닥을 비우고 [측정 시작] → z, roll, pitch
    - 합격 기준(inlier, σ)이 표로 나온다.
3. [2. XY / yaw 보정]
    - 설정: 점 4개 (0,0) (3,0) (0,1.5) (3,1.5), 원기둥 지름 10 cm × 높이 0.5 m → [검출 · 캡처 시작]
    - 캡처: 원기둥을 각 점에 놓고 캡처한다. 흔들림이 크면 다시 찍는다.
    - 4점 → [계산]. 목표 RMS 약 1.5 cm.
4. [3. YAML 적용]: `lidar_localization_os0_indoor.yaml` 선택 → 미리보기 → [적용]
    - 이전 yaml 은 백업이 자동으로 생긴다.

## 4. 캘리브레이션 확인 (RViz)

- 캘리브레이션 GUI 를 끄고 실행한다.

```bash
ros2 launch ~/drone_project/LidarDronePositioning/tools/indoor_view/indoor_view.launch.py
```

- 두 센서 점군(빨강, 파랑)이 바닥, 벽에서 겹치면 정상이다. 필드 사각형, ROI 상자도 보인다.
- 확인이 끝나면 Ctrl+C.

## 5. 추적 운용 (T3)

```bash
python3 ~/drone_project/LidarDronePositioning/tools/indoor_view/uav_tracking_gui.py
```

1. ID, 회차 입력. 융합 방식은 MF 가 기본이다.
2. [START RUN] → 비행 → [FINISH RUN]
3. [화면 열기]: 실시간 RViz (필드, ROI, 궤적, 좌표)
4. 결과: `~/drone_project/flight_logs/indoor_real/<ID>/runNN_MF_<시각>/`
    - `estimated_pose.csv` (t_sec, x, y, z, vx, vy, vz) — 약 10 Hz, 1분에 약 30 KB
    - `경로재생.sh` — 이 비행 재생
    - `run.yaml`, yaml 사본, `tracker.log`

- "원본 점군 녹화"는 성능 검증(모캡 비교) 때만 켠다. 1분에 2~3 GB.

---

## 6. 끝난 뒤 (오프라인)

### 경로 재생

- run 폴더의 `경로재생.sh` 우클릭 → [프로그램으로 실행] (Run as a Program)
- 또는 추적 GUI 의 [경로 재생] (방금 비행, 없으면 폴더 선택)
- 또는 터미널

```bash
python3 ~/drone_project/LidarDronePositioning/tools/indoor_view/path_replay.py <run 폴더>
```

- 재생 창: 재생 / 일시정지, 처음으로, 배속 0.25~4x, 시간 슬라이더, 지금 위치와 속력
- RViz: 회색 = 전체 경로, 초록 = 지금까지, 점 + (x, y, z) + 시각

### PF / MF / SU 비교 재생 — rosbag 을 녹화했을 때만

```bash
python3 -m drone_localization.replay_bag --bag <run 폴더>/bag --out <결과 폴더> --tracker cov \
  --clock arrival --quiet --ros-args --params-file <yaml> -p fusion_mode:=cloud
# MF = position, SU = sequential
```
