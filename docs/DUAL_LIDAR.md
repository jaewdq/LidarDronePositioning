# 라이다 2대 구성

한 대짜리 설치·보정은 [LIDAR_SETUP.md](LIDAR_SETUP.md), [CALIBRATION.md](CALIBRATION.md),
[RUN.md](RUN.md) 에 있다. 연결부터 결과까지 전체 흐름과 미해결 문제는
[PIPELINE.md](PIPELINE.md).

이 문서는 **두 번째 라이다를 붙이면서 새로 한 것**만 다룬다.

작업일 2026-08-24.

---

## 1. 왜 두 대인가

한 대로 실내 비행을 찍어보니 두 가지가 원리적으로 안 됐다.

**머리 위 사각 원뿔.** 수직 시야가 ±21.46도라 라이다 바로 위는 못 본다.
실측 비행(run3)에서 드론이 라이다 위를 지나가며 **4.4초를 통째로 놓쳤다.**

```
비행 고도 1.25 m,  라이다 높이 0.775 m
  ->  사각 반경 (1.25 - 0.775) / tan(21.46도) = 1.21 m
직전 관측 (+2.36,-0.36,1.25)  라이다까지 1.26 m,  그 거리 FOV 상한 1.27 m  여유 2 cm
```

라이다를 높이 달수록 이 원뿔이 커진다. 대회장처럼 2 m 에 달면 더 심해진다.

**surface bias.** 라이다는 물체의 자기 쪽 면만 본다. median 을 쓰면 중심이
반지름만큼 라이다 쪽으로 치우친다. 보정 때는 원 피팅으로 없앨 수 있지만
(마커가 원기둥이므로) 드론에는 못 쓴다.

**두 대를 마주 보게 놓으면 둘 다 해결된다.** 한쪽 사각 원뿔을 다른 쪽이 메우고,
치우침이 서로 반대 방향이라 평균으로 상쇄된다.

---

## 2. 두 번째 라이다 붙이기

### 2.1 IP 충돌 — 스캔에 한 대만 잡힌다

**두 센서 모두 `192.168.6.11` 로 고정돼 있었다.** 그래서 서브넷을 전부 훑어도
한 대만 응답했다.

IP 를 모르는 센서를 찾는 방법은 mDNS 다. Ouster 는 `os-<시리얼>.local` 로 광고한다.

```bash
avahi-browse -art | grep -iA3 ouster
```

```
= enp59s0 IPv6 Ouster Sensor 122451001190   _ouster-lidar._tcp   local
   hostname = [os-122451001190.local]
   address = [fe80::be0f:a7ff:fe00:da36]
   txt = ["sn=122451001190" "pn=840-104682-05"]
```

IPv4 가 겹쳐도 **IPv6 링크로컬은 MAC 기반이라 항상 고유하다.** 그쪽으로 붙어서
설정을 읽고 바꾼다.

```bash
IP6="fe80::be0f:a7ff:fe00:da36%enp59s0"

# 지금 설정 확인 (충돌 확인)
curl -s -g "http://[$IP6]/api/v1/system/network"

# IPv4 고정 주소 변경
curl -s -g -X PUT "http://[$IP6]/api/v1/system/network/ipv4/override" \
  -H 'Content-Type: application/json' -d '"192.168.6.12/24"'
```

> 바꾸기 전에 **시리얼을 확인**해서 엉뚱한 센서를 건드리지 않게 한다.
> IPv6 로 접속하므로 실패해도 링크로컬로 복구할 수 있다.

결과:

| IP | 모델 | 시리얼 | 토픽 |
|---|---|---|---|
| 192.168.6.11 | OS1-070-128U-**AX** | 122531002331 | `/lidar1/points` |
| 192.168.6.12 | OS1-070-128U-**ASR** | 122451001190 | `/lidar2/points` |

### 2.2 모델이 달라도 시야각은 같다

라이다2 는 `ASR` — **단거리(Short Range)** 버전이다. 확인해보니 수직 시야는
사실상 같았다.

```
라이다1   -21.46도 ~ +21.12도
라이다2   -21.48도 ~ +21.18도
```

**차이는 최대 사거리이지 시야각이 아니다.** 그래서 사각지대 계산이 그대로 쓰인다.

### 2.3 udp_dest 를 비워두면 안 된다

두 대를 띄우니 라이다1 이 계속 죽었다.

```
[ERROR] maximum number of allowed errors from sensor::poll_client() reached, performing self reset...
[WARN]  sensor::poll_client()) returned error or timed out
```

센서 설정을 보니 **데이터를 엉뚱한 주소로 쏘고 있었다.**

```
192.168.6.11 : udp_dest 169.254.63.240     <- 링크로컬. PC 가 아니다
192.168.6.12 : udp_dest 192.168.6.100      <- 정상
```

`udp_dest: ''` (자동) 로 두면 센서가 목적지를 스스로 고르는데, 인터페이스가
여럿이거나 자가 리셋이 끼면 링크로컬을 잡는다. **명시해야 한다.**

```python
UDP_DEST = '192.168.6.100'
...
'udp_dest': UDP_DEST,
```

### 2.4 대역폭 — 프로파일 없이는 무너진다

2 대 x 2 MB x 10 Hz = **40 MB/s**. `FASTRTPS_DEFAULT_PROFILES_FILE` 유무의 차이가
한 대일 때보다 훨씬 크다.

| | 없을 때 | 있을 때 |
|---|---|---|
| 1 대 | 3.5 Hz | 10.0 Hz |
| **2 대** | **2.0 / 1.4 Hz** | **10.01 / 10.01 Hz** |

`~/.bashrc` 가 자동으로 넣어주므로 정상적인 터미널에서는 문제없다. 스크립트에서
`setup.bash` 만 source 하면 빠지니 주의.

---

## 3. 런치 파일 재작성

기존 `dual_os1.launch.py` 에는 `lidar2` 블록이 주석으로 있었지만 **`remappings`
방식이라 안 먹는 것**이었다(이 버전 ouster-ros 는 상대 이름으로 발행해서
`/ouster/points` 절대 경로 remap 이 매칭되지 않는다).

동작하는 `namespace` + 라이프사이클 전이 패턴을 함수 하나로 뽑아 두 대를 찍어낸다.

```python
SENSORS = {1: '192.168.6.11', 2: '192.168.6.12'}
UDP_DEST = '192.168.6.100'

def make_lidar(idx, host):
    node = LifecycleNode(..., namespace=f'lidar{idx}', parameters=[{...}])
    configure = EmitEvent(...TRANSITION_CONFIGURE)
    activate  = RegisterEventHandler(OnStateTransition(goal_state='inactive', ...))
    failed    = RegisterEventHandler(OnStateTransition(goal_state='finalized', ...))
    return [node, configure, activate, failed]
```

```bash
ros2 launch dual_ouster_driver dual_os1.launch.py                       # 두 대
ros2 launch dual_ouster_driver dual_os1.launch.py enable_lidar2:=false  # 한 대만
```

한 대가 응답하지 않으면 **전체를 내린다.** 반쪽만 뜬 상태로 두면 나중에 어느
쪽 데이터가 빠진 것인지 헷갈린다.

---

## 4. 동시 보정 — 마커를 두 번만 놓는다

### 4.1 왜 동시에 해야 하나

따로 보정하면 마커를 **4번** 놓게 된다(라이다1 용 M1·M2, 라이다2 용 M1·M2).
두 번째에 마커가 2 cm 어긋나면 **두 좌표계가 2 cm 어긋난 채로 굳고, 그 오차는
나중에 드러나지도 않는다.**

동시에 하면 **2번**만 놓는다. 두 라이다가 물리적으로 같은 점을 동시에 보므로
이 오차가 원천적으로 없다.

### 4.2 원리 — 좌표계는 마커가 정한다

각 라이다가 자기 프레임에서 M1·M2 를 재고, 각자 "M1 을 원점으로, M1→M2 를 +y 로"
푼다. **같은 물리적 점을 봤으니 결과 좌표계가 자동으로 일치한다.**
라이다 간 상대 위치를 따로 잴 필요가 없다.

확인해보니 `yaw_base`(정면 방향)는 최종 결과에 영향이 없다. 회전 전후로 상쇄된다.

```
yaw_base = b 일 때
  theta_b = target - atan2(dv_raw) - b = theta_0 - b
  yaw_b   = b + theta_b = theta_0                    <- b 가 사라짐
  origin_b = rot2(theta_b) · rot2(b) · P1 = origin_0  <- 역시 사라짐
```

정면 확인 단계는 **화면 표시와 탐색 범위용**일 뿐이다. 그래서 두 라이다가 서로
다른 방향을 봐도, 정면 확인을 라이다1 에만 해도 문제없다.

### 4.3 실행

```bash
ros2 run dual_ouster_driver calibrate --ros-args \
  -p lidar_ids:="[1,2]" -p front_axis:=+y -p marker_radius:=0.045 -p countdown:=10
```

```
[1/4] 정면   라이다1 만. 비우고 Enter -> Enter 후 10초 안에 라이다1 앞 1m 에 서기
[2/4] 높이   두 대 자동
[3/4] M1     마커 놓고 -> 라이다1 기준 좌표 입력 (예: 2.6 0)
             라이다2 는 후보만 기록하고 안 물어봄
[4/4] M2     마커 옮기고 -> 라이다1 기준 좌표 입력
             라이다2 는 '움직인 것' 자동 판별 -> 한 번만 확인
```

`lidar_ids` 를 안 주면 예전 단일 라이다 흐름 그대로 돈다.

### 4.4 라이다2 는 마커를 어떻게 찾나 — "움직인 것"

라이다2 는 정면 개념이 라이다1 과 달라서 같은 좌표 입력을 재활용할 수 없다.
360도로 훑으면 **후보가 10개 넘게 나온다.** 실제로 겪은 목록:

```
   [1]  2.85m  폭 0.08  높이 0.42  45점  잔차 10.8mm
   [2]  3.84m  폭 0.10  높이 0.40  56점  잔차 13.9mm     <- [1] 과 구분 불가
   [3]  4.05m  폭 0.17  높이 0.50  43점  잔차 15.4mm
   ... 12개
```

폭·높이·잔차만으로는 의자 다리나 기둥과 구분이 안 된다.

**해법: 방에서 M1 자리에서 M2 자리로 옮겨간 물체는 마커뿐이다.**

```python
def match_moved(m1_cands, m2_cands, ref_gap, still=0.15, gap_tol=0.12):
    a: M1 목록에 있는데 M2 목록에는 그 자리가 비어 있는 것   (떠난 자리)
    b: M2 목록에 새로 생긴 것                                (도착한 자리)
    |a-b| 가 라이다1 이 잰 M1-M2 간격과 맞을 것
```

세 조건을 동시에 만족하는 짝은 거의 유일하다. 합성 시험에서 후보 7개씩 있어도
**짝이 1개만** 나왔고, 마커를 안 옮긴 경우엔 **0개**가 나와 실수도 잡아냈다.

그래서 M1 선택을 M2 측정 뒤로 미룬다. 라이다2 에 대해 **한 번만** 확인하면 되고,
대개 [1]번이 정답이다.

> **주의**: M1·M2 사이에 마커 말고 아무것도 움직이면 안 된다. 사람도 두 측정
> 사이에 같은 자리에 있거나 아예 밖으로 나가 있어야 한다.

### 4.5 교차검증

기준값이 없는 상황에서 얻을 수 있는 가장 실질적인 검증이 **두 라이다가 잰
M1↔M2 간격 비교**다. 같은 두 점을 봤으니 같은 값이 나와야 한다.

```
[교차검증] 각 라이다가 잰 M1-M2 간격
  라이다1 2.353 m   라이다2 2.363 m
  최대 차이 10.0 mm   두 라이다가 같은 것을 보고 있습니다.
```

20 mm 이내면 좋고, 50 mm 넘으면 다른 물체를 골랐거나 마커가 움직인 것이다.

**두 라이다 사이 거리**도 출력한다. 줄자로 재서 맞춰보면 배치 확인이 된다.

---

## 5. 겪은 함정 다섯 가지

### 5.1 params 파일이 조용히 무시된다 — 제일 오래 헤맸다

RViz 에서 두 점군이 **2 m 어긋나고 대칭으로** 보였다. 보정이 틀린 줄 알았는데,
실행 중인 노드 파라미터를 찍어보니 원인이 나왔다.

```
실제 실행중 : lidar_z 0.84,  yaw 180.0,  origin (0,0)     <- 전부 기본값
YAML 에는   : lidar_z 0.827, yaw 270.61, origin (-1.85, 1.53)
```

**ROS 2 파라미터 파일은 노드 이름으로 매칭된다.** YAML 키가 `roi_filter:` 인데
런치에서 노드를 `roi_filter_1` 로 지어서 **경고 하나 없이 통째로 무시**됐다.

두 겹으로 막았다.

**(1) 와일드카드 키** — `calibrate` 가 이제 이렇게 쓴다.

```yaml
/**:
  ros__parameters:
    lidar_z: 0.8388
```

`/**` 는 이름·이름공간과 무관하게 매칭된다. `roi_filter` 로 띄우든
`roi_filter_99` 로 띄우든 먹는다(실측 확인).

**(2) 조용한 실패 차단** — `roi_filter` 가 뜰 때 원점이 정확히 (0,0) 이면 경고한다.

```
[WARN] origin 이 정확히 (0,0) 입니다 — 보정값이 안 실렸을 수 있습니다.
[WARN]   params 파일의 최상위 키가 "/**:" 인지, --params-file 경로가 맞는지 확인하세요.
[WARN]   (지금 이 노드 이름: roi_filter_bare)
```

보정하면 원점이 정확히 0.000 이 나올 일은 사실상 없다.

### 5.2 YAML 이 build/ 에 갇힌다

`calibrate` 가 `calibration_lidar2.yaml` 을 `build/dual_ouster_driver/config/` 에
썼다. `--symlink-install` 이라 `__file__` 이 build 아래를 가리키는데,

- `calibration_lidar1.yaml` 은 **이미 있던 심볼릭 링크**라 쓰기가 src 로 따라갔고
- `calibration_lidar2.yaml` 은 링크가 없어서 **build 에 실제 파일**로 만들어졌다

다음 빌드에 사라진다. `config_dir()` 이 build → src 로 되돌려 확인하게 고쳤다.

### 5.3 rviz 폴더가 설치되지 않는다

`setup.py` 의 `data_files` 에 `launch` 와 `config` 만 있고 `rviz` 가 빠져 있었다.
런치가 `install/.../launch/` 에서 `../rviz` 를 찾으니 없었다.

`glob('rviz/*.rviz')` 를 추가하고, 런치는 `get_package_share_directory` 로 찾는다.

> 소스 경로를 추측하는 우회는 쓰지 말 것. `build/` 와 `install/` 의 폴더 구조가
> 달라서(`share/<pkg>` 가 한 겹 더 있다) 문자열 치환으로는 맞출 수 없다.

### 5.4 라이다가 서로를 물체로 잡는다

마주 보게 놓으면 **상대 라이다가 ROI 안에 들어온다.**

```
라이다2 가 잡은 것 :  55점  (+1.84,-1.48,0.83)  6x5x6cm
라이다1 의 실제 위치:  (+1.856,-1.524),  높이 0.839     <- 5 cm 차이
```

라이다1 이 `y=-1.524` 인데 `roi_y_min` 이 `-1.5` 라 **2.4 cm 차이로 걸쳐** 있었다.
`roi_y_min` 을 `-1.30` 으로 20 cm 올려서 해결했다(비행 영역은 그만큼 줄어든다).

ROI 를 줄이고 싶지 않으면 `roi_filter` 의 제외 반경을 쓸 수 있다(기본 꺼짐).

```bash
ros2 launch dual_ouster_driver dual_view.launch.py exclude_radius:=0.5
```

각 라이다 자리 반경을 ROI 에서 뺀다. 런치가 보정값에서 라이다 위치를 자동으로
읽어 넘긴다.

### 5.5 중복 실행

같은 런치를 두 번 띄우면 노드 이름이 충돌한다. 드라이버는 이렇게 실패한다.

```
[ERROR] Failed to make transition 'TRANSITION_CONFIGURE' for LifecycleNode '/lidar1/os_driver_lidar1'
```

**이 메시지가 나오면 거의 항상 중복 실행이다.** 띄우기 전에 확인한다.

```bash
ros2 node list
```

중복이 생기면 **둘 다 끄고 하나만 새로 띄운다.** 한쪽만 죽이려 하면 어느 쪽이
살아남을지 예측이 안 된다(실제로 원래 것까지 같이 내려갔다).

---

## 6. 기울기 — 새로 발견한 계통 오차

### 6.1 증상

바닥은 평평한데 **라이다가 재는 바닥 높이가 거리에 따라 달라졌다.**

```
2.4~3.0m 지점의 바닥   0.840 m
3.0~4.0m 지점의 바닥   0.816 m
4.0~5.0m 지점의 바닥   0.816 m
5.0~6.5m 지점의 바닥   0.799 m
```

바닥에 평면을 맞추니 법선이 수직에서 벗어나 있었고, **평면 잔차는 7.5 mm** 로
작았다. 즉 **바닥은 평평하고 라이다가 기운 것.**

### 6.2 왜 문제인가

보정은 **"바닥이 수평"** 이라는 가정 위에 서 있다. `lidar_z` 하나만 재면 되는
이유가 그것이다. 기울면 이 가정이 깨지고 **z 가 거리에 비례해 틀어진다.**

| 기울기 | 4 m | 10 m | 20 m |
|---|---|---|---|
| 1.15도 | 8.1 cm | 20.1 cm | 40.3 cm |
| 2.24도 | 15.7 cm | 39.1 cm | 78.3 cm |

**x, y 는 거의 영향받지 않는다.** 기울기는 z 만 틀어놓는다.

### 6.3 실측 도구

```bash
ros2 run dual_ouster_driver tilt_check
```

2초마다 갱신된다. 마운트를 돌리면서 보면 된다.

```
  라이다1  0.76도 기움 (방위 -32 잔차 7.5mm 11046점)   |   라이다2  0.18도 OK  (방위+127 잔차 8.0mm 11189점)
```

- **기울기** 0.2도 미만이면 충분
- **잔차** 15 mm 넘으면 바닥이 평평하지 않은 것이니 다른 자리에서 다시
- **방위** 어느 쪽으로 기울었는지

### 6.4 고친 효과

수평을 잡으니 보정 좌표계에서 **바닥이 실제로 z=0 에 누웠다.**

| | 기울기 | ROI 안 바닥 최대 z |
|---|---|---|
| 라이다1 | 1.15도 → 0.75도 | +444 mm → **+27 mm** |
| 라이다2 | 2.24도 → 0.25도 | +450 mm → **+33 mm** |

**45 cm → 3 cm.** 이 전에는 `roi_z_min` 을 0.40 까지 올려야 바닥이 안 딸려왔는데,
이제 0.05 로도 깨끗하다. 바닥에 놓은 물체를 볼 수 있게 됐다.

보정 정확도도 같이 좋아졌다.

```
라이다1  yaw 정확도 0.096도 -> 0.086도
라이다2  yaw 정확도 0.144도 -> 0.110도
두 라이다 간격 측정 차이  20 mm -> 10 mm
```

> **수평을 바꾸면 `lidar_z` 가 달라지므로 반드시 재보정한다.**

---

## 7. 겹침 확인

### 7.1 실행

```bash
# 터미널 1 — 드라이버 (한 번 켜면 계속 둔다)
ros2 launch dual_ouster_driver dual_os1.launch.py

# 터미널 2 — roi_filter x2 + cluster_median x2 + TF x2 + RViz
ros2 launch dual_ouster_driver dual_view.launch.py z_min:=0.05
```

| 인자 | 기본 | 뜻 |
|---|---|---|
| `lidar_ids` | `1,2` | 어느 라이다들 |
| `z_min` | `0.0` | ROI 바닥. 0 이면 YAML 값 |
| `exclude_radius` | `0.0` | 라이다 자리 제외 반경. 0 이면 끔 |
| `tf` | `true` | map→lidarN 정적 TF (원본 점군 보려면 필요) |
| `rviz` | `true` | RViz 같이 띄우기 |

화면:

| 표시 | 색 |
|---|---|
| `ROI_lidar1` / 클러스터 구슬 | 빨강 |
| `ROI_lidar2` / 클러스터 구슬 | 파랑 |
| `Raw_lidar1/2` | 원본 점군, 기본 꺼둠 |
| Grid | 1 m 격자, 중심이 보정 원점 |

`cluster_median` 에 `marker_color` 파라미터를 추가해서 라이다별로 구슬 색이 다르다.

### 7.2 숫자로 판정하는 법

눈으로 보면 "겹쳤다/안 겹쳤다" 밖에 안 나온다. **두 중심의 차이를 두 성분으로
나눠 봐야 한다.**

```
두 라이다를 잇는 축 방향 성분   <- surface bias.  물체 두께만큼이면 정상
그 축에 수직인 성분             <- 좌표계 불일치.  이게 진짜 오차
```

실측:

```
물체 크기 13~27 cm
  라이다1 중심 (+0.769, +1.030, 0.449)   505점
  라이다2 중심 (+0.761, +1.105, 0.475)   905점

  수평 차이 7.6 cm
    축 방향  +7.5 cm   <- 물체 지름과 일치. 정상
    축 수직  +0.8 cm   <- 좌표계 일치도. 훌륭
    dz       +2.6 cm
```

**축에 수직인 8 mm 가 두 좌표계의 실제 불일치다.** 마커 흔들림이 2~5 mm,
yaw 정확도가 0.1도 수준이었으니 이론적 최선에 가깝다.

**축 방향 7.5 cm 는 오차가 아니라 기대했던 것이다.** 두 중점이 물체의 진짜 중심이다.

```
참 중심 추정 (+0.765, +1.067, 0.462)
```

**두 대를 쓰는 이유가 이 한 줄이다.** 한 대면 7.5 cm 치우친 값을 그대로 받아야 한다.

---

## 8. 현재 보정값

2026-08-24 19:35 측정. 수평 조정 후.

```yaml
# calibration_lidar1.yaml
lidar_z: 0.8388      yaw_deg: 271.0766
origin_x: -1.8563    origin_y: 1.5243
# 라이다 위치 (+1.8563, -1.5243),  원점에서 2.4019 m
# yaw 정확도 0.086도  (마커 간격 2.35 m)

# calibration_lidar2.yaml
lidar_z: 0.8917      yaw_deg: 94.8580
origin_x: -1.8150    origin_y: -3.4120
# 라이다 위치 (+1.8150, +3.4120),  원점에서 3.8647 m
# yaw 정확도 0.110도  (마커 간격 2.36 m)

# 공통 ROI
roi_x: -0.20 ~ 3.90     roi_y: -1.30 ~ 3.10     roi_z: 0.30 ~ 2.50
```

```
두 라이다 사이 거리   4.936 m
서로 마주 봄          yaw 271.08 vs 94.86  (176.2도 차이)
높이차                5.3 cm
```

**이 값은 이 물리적 배치에서만 유효하다.** 라이다를 건드리면 다시 측정.

---

## 9. 추가된 노드·파일

| 이름 | 역할 |
|---|---|
| `tilt_check` | 라이다 수평 실시간 확인 (마운트 돌리며 보는 용도) |
| `verify_pose` | 줄자로 잰 자리와 라이다 값 비교 (만들었으나 아직 안 씀) |
| `dual_view.launch.py` | roi_filter x2 + cluster_median x2 + TF + RViz |
| `rviz/dual_view.rviz` | 두 라이다 겹쳐 보기 (라이다별 색 구분) |

기존 노드에 추가된 파라미터:

| 노드 | 파라미터 | 뜻 |
|---|---|---|
| `calibrate` | `lidar_ids` | 여러 대 동시 보정. 비우면 단일 |
| `cluster_median` | `marker_color` | RViz 구슬 색 [r,g,b] |
| `roi_filter` | `exclude_xy`, `exclude_radius` | 그 자리 반경을 ROI 에서 뺌 |

---

## 10. 아직 안 된 것

### 10.1 두 스트림을 합치는 코드 — 다음 할 일

지금은 두 라이다가 **각자 따로** 위치를 낸다. `cluster_median` 은 토픽 하나만
받고, `flight_recorder` 도 하나만 구독한다. **합치는 코드가 없다.**

방법이 둘이다.

**(A) 점군을 먼저 합치고 한 번에 군집화** — 양쪽 면을 다 가진 점군이 되니
median 이 자연스럽게 중심에 가까워진다. 한쪽이 사각 원뿔에 가려도 다른 쪽 점이
살아있어 추적이 안 끊긴다. **권장.**

**(B) 각자 위치를 뽑아서 평균** — 간단하고, 한쪽이 놓쳐도 버틴다. 다만 한쪽만
잡은 프레임에서는 surface bias 가 그대로 남는다.

어느 쪽이든 **타임스탬프 정렬**이 필요하다. 두 라이다가 정확히 동기되지 않으므로
가까운 프레임끼리 짝지어야 한다(현재 각 라이다 지연 203~217 ms).

### 10.2 남은 것

- 라이다1 기울기 0.75도 (라이다2 는 0.25도로 잡음)
- 기준값(ground truth) 검증 — 실내는 모캡 예정
- 추적 연관/게이팅 — `cluster_median` 은 매 프레임 독립적으로 "가장 큰 군집"을
  고를 뿐이라 사람이 들어오면 갈아탄다
- 드론/사람 판별
- 대회장 규모(24x15 m) 검증

---

## 11. 빠른 참조

```bash
# 라이다 찾기 (IP 모를 때)
avahi-browse -art | grep -iA3 ouster

# 센서 정보 / 네트워크 설정
curl -s http://192.168.6.11/api/v1/sensor/metadata/sensor_info
curl -s http://192.168.6.11/api/v1/sensor/config

# 드라이버
ros2 launch dual_ouster_driver dual_os1.launch.py
ros2 topic hz /lidar1/points          # 10 근처여야 정상

# 수평 확인
ros2 run dual_ouster_driver tilt_check

# 동시 보정
ros2 run dual_ouster_driver calibrate --ros-args \
  -p lidar_ids:="[1,2]" -p front_axis:=+y -p marker_radius:=0.045 -p countdown:=10

# 겹침 확인
ros2 launch dual_ouster_driver dual_view.launch.py z_min:=0.05

# 파라미터 실시간 조정 (roi_filter 는 매 프레임 다시 읽는다)
ros2 param set /roi_filter_1 roi_y_min -1.3
ros2 param dump /roi_filter_1

# kill -9 후 정리
ros2 daemon stop && rm -f /dev/shm/fastrtps_*
```

> `pkill -f roi_filter` 는 쓰지 말 것. 명령줄 자체가 패턴에 걸려 **자기 자신도
> 죽인다.** `ps -eo pid,cmd | grep` 으로 PID 를 찾아 `kill -INT` 한다.
