#!/usr/bin/env bash
#
# 라이다 드론 측위 시스템 설치
#
# 전제: Ubuntu 22.04 + ROS 2 Humble 이 이미 깔려 있음
# 하는 일: 의존성 설치 -> ouster-ros 받아서 패치 -> 워크스페이스 구성 -> 빌드
#          -> 셸 환경 -> 네트워크 프로파일
#
# 여러 번 돌려도 안전하다. 이미 된 단계는 건너뛴다.
#
#   ./setup.sh              대화형 (권장)
#   ./setup.sh --yes        묻지 않고 기본값으로
#   ./setup.sh --no-network 네트워크 설정은 건너뛰기
#
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ouster-ros 를 이 커밋으로 고정한다. 업스트림이 바뀌어도 같은 결과가 나오게.
# 이 커밋에서 patches/0001-window-field-guard.patch 가 깨끗이 적용된다.
OUSTER_REPO="https://github.com/ouster-lidar/ouster-ros.git"
OUSTER_BRANCH="ros2"
OUSTER_COMMIT="aef1a24e291b3cf948cbac3b94db9e4a56d43558"

# 기본값. 대화형에서 바꿀 수 있다.
WS_DEFAULT="$HOME/ros2_ws"
PC_IP_DEFAULT="192.168.6.100/24"
LIDAR1_DEFAULT="192.168.6.11"
LIDAR2_DEFAULT="192.168.6.12"

ASSUME_YES=0
DO_NETWORK=1
for a in "$@"; do
    case "$a" in
        --yes|-y)    ASSUME_YES=1 ;;
        --no-network) DO_NETWORK=0 ;;
        --help|-h)   sed -n '2,14p' "$0"; exit 0 ;;
        *) echo "모르는 인자: $a"; exit 1 ;;
    esac
done

# ---------------------------------------------------------------- 출력 도우미
c_ok()   { printf '\033[32m  OK  \033[0m %s\n' "$*"; }
c_warn() { printf '\033[33m 주의 \033[0m %s\n' "$*"; }
c_err()  { printf '\033[31m 실패 \033[0m %s\n' "$*"; }
c_skip() { printf '\033[90m 건너뜀\033[0m %s\n' "$*"; }
step()   { printf '\n\033[1m[%s] %s\033[0m\n' "$1" "$2"; }

# /dev/tty 는 파일이 있어도 '열 수' 없는 경우가 있다 (제어 터미널이 없을 때).
# -c/-r 검사로는 못 잡으므로 실제로 열어본다. 실패하면 표준입력을 쓴다.
if { true >/dev/tty; } 2>/dev/null; then HAVE_TTY=1; else HAVE_TTY=0; fi

_read_into() {   # _read_into 변수명 "프롬프트"
    if [ "$HAVE_TTY" = 1 ]; then
        read -r -p "$2" "$1" </dev/tty || true
    else
        read -r -p "$2" "$1" || true
    fi
}

ask() {   # ask "질문" "기본값"  ->  stdout 으로 답
    local q="$1" d="${2:-}" a=""
    if [ "$ASSUME_YES" = 1 ]; then echo "$d"; return; fi
    _read_into a "  $q [$d]: "
    echo "${a:-$d}"
}

confirm() {  # confirm "질문"  -> 0 이면 예
    [ "$ASSUME_YES" = 1 ] && return 0
    local a=""
    _read_into a "  $1 [Y/n]: "
    [[ -z "$a" || "$a" =~ ^[Yy] ]]
}

# ================================================================ 0. 사전 점검
step 0/8 "사전 점검"

. /etc/os-release 2>/dev/null || true
if [ "${VERSION_ID:-}" != "22.04" ]; then
    c_warn "Ubuntu 22.04 가 아닙니다 (${VERSION_ID:-알수없음}). 계속하면 문제가 생길 수 있습니다."
    confirm "그래도 계속할까요?" || exit 1
else
    c_ok "Ubuntu 22.04"
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
    c_err "ROS 2 Humble 이 없습니다. 먼저 설치하세요:"
    echo "      https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
    exit 1
fi
c_ok "ROS 2 Humble"

if ! command -v colcon >/dev/null; then
    c_warn "colcon 이 없습니다. 아래에서 같이 설치합니다."
fi

# ================================================================ 1. 설정 수집
step 1/8 "설정"

WS="$(ask '워크스페이스 경로' "$WS_DEFAULT")"
WS="${WS/#\~/$HOME}"

if [ "$DO_NETWORK" = 1 ]; then
    # 유선 인터페이스 자동 감지. 무선/가상은 뺀다.
    mapfile -t NICS < <(ls /sys/class/net | grep -E '^(en|eth)' || true)
    if [ "${#NICS[@]}" -eq 0 ]; then
        c_warn "유선 인터페이스를 못 찾았습니다. 네트워크 설정을 건너뜁니다."
        DO_NETWORK=0
    else
        echo "  감지된 유선 인터페이스:"
        for n in "${NICS[@]}"; do
            printf '    %-12s carrier=%s  %s\n' "$n" \
                "$(cat /sys/class/net/$n/carrier 2>/dev/null || echo '?')" \
                "$(ip -br addr show $n | awk '{$1="";$2="";print}')"
        done
        NIC="$(ask '라이다를 연결할 인터페이스' "${NICS[0]}")"
        PC_IP="$(ask 'PC 에 줄 고정 IP' "$PC_IP_DEFAULT")"
    fi
fi

LIDAR1="$(ask '라이다1 IP' "$LIDAR1_DEFAULT")"
LIDAR2="$(ask '라이다2 IP (한 대만 쓰면 비워두세요)' "$LIDAR2_DEFAULT")"

# 이미 쓰던 워크스페이스면 알려준다. 프롬프트가 실패해 엉뚱한 곳을
# 건드리는 사고를 막는다.
if [ -d "$WS/src" ] && [ -n "$(ls -A "$WS/src" 2>/dev/null)" ]; then
    c_warn "$WS/src 에 이미 내용이 있습니다:"
    ls "$WS/src" | sed 's/^/        /'
fi

echo
echo "  ── 이렇게 진행합니다 ──"
echo "    워크스페이스   $WS"
[ "$DO_NETWORK" = 1 ] && echo "    네트워크       $NIC 에 $PC_IP"
echo "    라이다1        $LIDAR1"
[ -n "$LIDAR2" ] && echo "    라이다2        $LIDAR2"
echo
confirm "계속할까요?" || exit 1

# ================================================================ 2. 의존성
step 2/8 "의존성 설치 (sudo 필요)"

APT_PKGS=(
    # 빌드
    git cmake build-essential python3-pip
    python3-colcon-common-extensions python3-rosdep
    # ouster-ros 가 요구하는 것
    libeigen3-dev libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev
    libtins-dev libpcap-dev libzip-dev
    ros-humble-pcl-ros ros-humble-pcl-conversions
    ros-humble-tf2-eigen ros-humble-cv-bridge
    # 우리 코드 (과학 패키지는 apt 말고 pip 로. 아래 주석 참고)
    ros-humble-rviz2 python3-yaml
    # mDNS 로 라이다 찾기 (IP 모를 때)
    avahi-utils
    # 여러 터미널 띄우기 편하게
    terminator
)

MISSING=()
for p in "${APT_PKGS[@]}"; do
    dpkg -s "$p" >/dev/null 2>&1 || MISSING+=("$p")
done

if [ "${#MISSING[@]}" -eq 0 ]; then
    c_skip "apt 패키지 전부 설치돼 있음"
else
    echo "  설치할 것: ${MISSING[*]}"
    sudo apt-get update
    sudo apt-get install -y "${MISSING[@]}"
    c_ok "apt 패키지 ${#MISSING[@]}개"
fi

# 과학 패키지는 apt 가 아니라 pip 로 깐다.
#
#   apt 판은 numpy 1.21 / scipy 1.8 로 묶여 있는데, 이 조합에서
#   'numpy.core.multiarray failed to import' / '_ARRAY_API not found' 가 난다.
#   실제로 당했다. pip 최신판을 ~/.local 에 깔면 dist-packages 를 가려서
#   해결된다 (sys.path 에서 ~/.local 이 앞선다).
#
#   확인:  python3 -c "import numpy,sklearn; print(numpy.__file__)"
#          -> ~/.local/... 이 나와야 정상
PIP_PKGS=(numpy scipy scikit-learn plotly matplotlib)
echo "  pip 패키지 확인 중..."
if python3 -c "
import sys
import importlib.util as u
mods = ('numpy','scipy','sklearn','plotly','matplotlib')
missing = [m for m in mods if u.find_spec(m) is None]
local = all('/.local/' in (u.find_spec(m).origin or '')
            for m in mods if u.find_spec(m) is not None)
sys.exit(0 if (not missing and local) else 1)
" 2>/dev/null; then
    c_skip "pip 과학 패키지 이미 있음 (~/.local)"
else
    pip3 install --user --upgrade "${PIP_PKGS[@]}"
    c_ok "pip 과학 패키지"
fi

# 제대로 가려졌는지 확인. 여기서 걸러야 나중에 보정이 이상하게 도는 걸 막는다.
python3 - <<'PY' || c_warn "과학 패키지 확인 실패. 위 pip 출력을 보세요."
import importlib
bad = []
for m in ('numpy', 'scipy', 'sklearn'):
    mod = importlib.import_module(m)
    where = 'pip' if '/.local/' in (mod.__file__ or '') else 'apt'
    print(f'      {m:10} {mod.__version__:10} ({where})')
    if where != 'pip':
        bad.append(m)
if bad:
    print(f'      주의: {bad} 가 apt 판입니다. ABI 충돌이 날 수 있습니다.')
PY

# ================================================================ 3. 워크스페이스
step 3/8 "워크스페이스"

mkdir -p "$WS/src"
c_ok "$WS/src"

# 우리 패키지는 저장소를 심볼릭 링크한다. 저장소에서 코드를 고치면 바로 반영된다.
if [ -e "$WS/src/dual_ouster_driver" ]; then
    if [ -L "$WS/src/dual_ouster_driver" ]; then
        c_skip "dual_ouster_driver 링크 이미 있음"
    else
        c_warn "$WS/src/dual_ouster_driver 가 실제 폴더입니다 (링크 아님)."
        c_warn "저장소 사본과 별개로 관리됩니다. 그대로 두겠습니다."
    fi
else
    ln -s "$REPO/src/dual_ouster_driver" "$WS/src/dual_ouster_driver"
    c_ok "dual_ouster_driver -> $REPO/src/dual_ouster_driver (링크)"
fi

# ================================================================ 4. ouster-ros
step 4/8 "ouster-ros 받기 + 패치"

OUSTER_DIR="$WS/src/ouster-ros"
if [ -d "$OUSTER_DIR/.git" ]; then
    c_skip "ouster-ros 이미 있음"
else
    echo "  clone 중 (100MB 정도, 몇 분 걸립니다)..."
    git clone -b "$OUSTER_BRANCH" "$OUSTER_REPO" "$OUSTER_DIR"
    c_ok "clone"
fi

cd "$OUSTER_DIR"
CUR="$(git rev-parse HEAD)"
if [ "$CUR" != "$OUSTER_COMMIT" ]; then
    echo "  커밋 고정: ${OUSTER_COMMIT:0:8}"
    git fetch origin "$OUSTER_BRANCH" --quiet || true
    git checkout --quiet "$OUSTER_COMMIT"
    c_ok "커밋 ${OUSTER_COMMIT:0:8}"
else
    c_skip "커밋 이미 ${OUSTER_COMMIT:0:8}"
fi

echo "  서브모듈 (ouster-sdk)..."
git submodule update --init --recursive --quiet
c_ok "서브모듈"

# 패치: FW 3.2 미만 센서는 WINDOW 필드를 안 보내는데 상위 코드가 무조건 읽어서
# 'Field WINDOW not found in LidarScan' 으로 죽는다. 업스트림 버그다.
PATCH="$REPO/patches/0001-window-field-guard.patch"
if git apply --check --reverse "$PATCH" 2>/dev/null; then
    c_skip "패치 이미 적용돼 있음"
elif git apply --check "$PATCH" 2>/dev/null; then
    git apply "$PATCH"
    c_ok "패치 적용 (WINDOW 필드 가드)"
else
    c_err "패치가 안 맞습니다. ouster-ros 버전이 달라졌을 수 있습니다."
    c_err "  수동 확인: git -C $OUSTER_DIR status"
    exit 1
fi

# ================================================================ 5. 빌드
step 5/8 "빌드"

cd "$WS"
# ROS 의 setup.bash 는 set -u 에서 'AMENT_TRACE_SETUP_FILES: unbound variable'
# 로 죽는다. 그 구간만 잠시 끈다.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init >/dev/null 2>&1 || true
fi
rosdep update --rosdistro humble >/dev/null 2>&1 || c_warn "rosdep update 실패 (무시 가능)"
rosdep install --from-paths src --ignore-src -y --rosdistro humble >/dev/null 2>&1 \
    || c_warn "rosdep install 이 일부 실패했습니다. 빌드가 안 되면 여기를 보세요."

# 워크스페이스 전체가 아니라 필요한 것만 빌드한다. 기존 워크스페이스에
# 다른 패키지가 있을 때 그쪽 빌드 실패에 말려들지 않게.
echo "  colcon build (처음이면 10~20분)..."
if colcon build --symlink-install --packages-up-to ouster_ros dual_ouster_driver; then
    c_ok "빌드"
else
    c_err "빌드 실패. 위 로그를 보세요."
    echo "      자주 있는 원인:"
    echo "        - build/ 에 옛 산물이 남음  ->  rm -rf $WS/build $WS/install 후 재시도"
    echo "        - 의존성 누락              ->  rosdep install --from-paths src --ignore-src -y"
    exit 1
fi

# ================================================================ 6. 셸 환경
step 6/8 "셸 환경"

BRC="$HOME/.bashrc"
DDS_XML="$REPO/src/dual_ouster_driver/config/fastdds_large.xml"

add_line() {   # 없을 때만 추가
    local line="$1" marker="$2"
    if grep -qF "$marker" "$BRC"; then
        c_skip "$marker (이미 있음)"
    else
        printf '%s\n' "$line" >> "$BRC"
        c_ok "$marker 추가"
    fi
}

if confirm "~/.bashrc 에 환경설정 3줄을 추가할까요?"; then
    grep -qF '# --- 라이다 측위 시스템 ---' "$BRC" || \
        printf '\n# --- 라이다 측위 시스템 ---\n' >> "$BRC"
    add_line "source /opt/ros/humble/setup.bash" "source /opt/ros/humble/setup.bash"
    add_line "source $WS/install/setup.bash" "$WS/install/setup.bash"
    # 이게 없으면 2MB 포인트클라우드가 UDP 로 1400조각 나서 프레임이 대량 유실된다.
    # 라이다 2대면 10Hz -> 1.4Hz 까지 떨어진다.
    add_line "export FASTRTPS_DEFAULT_PROFILES_FILE=$DDS_XML" "FASTRTPS_DEFAULT_PROFILES_FILE"
else
    c_skip "~/.bashrc (직접 추가하세요)"
    echo "      source /opt/ros/humble/setup.bash"
    echo "      source $WS/install/setup.bash"
    echo "      export FASTRTPS_DEFAULT_PROFILES_FILE=$DDS_XML"
fi

# ================================================================ 7. 네트워크
step 7/8 "네트워크"

if [ "$DO_NETWORK" != 1 ]; then
    c_skip "네트워크 설정 (--no-network)"
elif ! command -v nmcli >/dev/null; then
    c_warn "nmcli 가 없습니다. 수동으로 $NIC 에 $PC_IP 를 설정하세요."
else
    # 라이다는 자기 IP 를 스스로 기억한다. PC 는 같은 대역 주소를 고정으로 박는다.
    # 같은 /24 라 라우터 없이 직접 통신한다. 게이트웨이는 넣지 않는다 —
    # 넣으면 PC 의 인터넷 기본 경로를 가로챈다.
    if nmcli -t -f NAME con show | grep -qx "ouster"; then
        c_skip "'ouster' 프로파일 이미 있음"
        nmcli -t -f connection.interface-name,ipv4.addresses con show ouster | sed 's/^/      /'
        if confirm "이 프로파일을 $NIC / $PC_IP 로 덮어쓸까요?"; then
            sudo nmcli con mod ouster connection.interface-name "$NIC" \
                 ipv4.method manual ipv4.addresses "$PC_IP" ipv4.gateway ""
            c_ok "프로파일 갱신"
        fi
    else
        sudo nmcli con add type ethernet ifname "$NIC" con-name ouster \
            ipv4.method manual ipv4.addresses "$PC_IP" \
            ipv6.method auto autoconnect yes
        c_ok "'ouster' 프로파일 생성"
    fi

    if [ "$(cat /sys/class/net/$NIC/carrier 2>/dev/null || echo 0)" = "1" ]; then
        sudo nmcli con up ouster >/dev/null 2>&1 && c_ok "프로파일 활성화" || \
            c_warn "활성화 실패. 랜선을 확인하세요."
    else
        c_warn "$NIC 에 랜선이 안 꽂혀 있습니다 (carrier=0). 꽂으면 자동으로 올라옵니다."
    fi
fi

# ================================================================ 8. 확인
step 8/8 "확인"

echo
echo "  ── 지금 상태 ──"
if [ "$DO_NETWORK" = 1 ]; then
    printf '    %-16s %s\n' "$NIC carrier" "$(cat /sys/class/net/$NIC/carrier 2>/dev/null || echo '?')"
    printf '    %-16s %s\n' "$NIC 주소" "$(ip -br addr show $NIC 2>/dev/null | awk '{$1="";$2="";print}')"
fi
for ip in $LIDAR1 $LIDAR2; do
    if ping -c1 -W1 "$ip" >/dev/null 2>&1; then
        c_ok "$ip 응답"
    else
        c_warn "$ip 무응답 (랜선/전원/IP 확인)"
    fi
done

cat <<EOF

════════════════════════════════════════════════════════════════
  설치 완료
════════════════════════════════════════════════════════════════

  새 터미널을 열고 (또는 source ~/.bashrc):

    ros2 launch dual_ouster_driver dual_os1.launch.py
    ros2 topic hz /lidar1/points          # 10 근처면 정상

  그다음 보정을 해야 합니다. 보정값은 설치 장소마다 다릅니다.

    ros2 run dual_ouster_driver calibrate --ros-args \\
      -p lidar_ids:="[1,2]" -p front_axis:=+y -p marker_radius:=0.045 -p countdown:=10

  자세한 것은 문서를 보세요.

    docs/PIPELINE.md      전체 흐름과 미해결 문제
    docs/CALIBRATION.md   보정 원리
    docs/RUN.md           명령어 모음

EOF
