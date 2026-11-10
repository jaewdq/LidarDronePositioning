"""Calibration session folders and the text parsing calibration_gui relies on.

One calibration run = one folder, so measurements from different days, or a
retry the same afternoon, never share files::

    <results_dir>/<site_key>/
        2026-10-31_r01_desk-test/        date, run number of that day, memo
            session.yaml                 site, sensors, memo, created
            step1_ground.yaml            parameters used + z / roll / pitch per sensor
            step2_setup.yaml             grid points, target and nominal parameters
            sensor1_captures.yaml        written by sensor_calibration_node
            step2_capture_notes.yaml     per-capture spread and distance check (GUI)
            sensors_calibration.yaml     written by sensor_calibration_node
            step3_apply.yaml             which tracker YAML was changed, how, backup
            logs/                        every subprocess log + gui.log
            session.lock                 pid of the GUI that has it open

Everything here is plain Python so it can be unit tested without ROS or Tk.
"""

from __future__ import annotations

import math
import os
import re
import socket
from dataclasses import dataclass, field
from datetime import date, datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

SESSION_RE = re.compile(r'^(\d{4}-\d{2}-\d{2})_r(\d{2,})(?:_(.+))?$')

SESSION_FILE = 'session.yaml'
GROUND_FILE = 'step1_ground.yaml'
SETUP_FILE = 'step2_setup.yaml'
NOTES_FILE = 'step2_capture_notes.yaml'
RESULT_FILE = 'sensors_calibration.yaml'
APPLY_FILE = 'step3_apply.yaml'
LOCK_FILE = 'session.lock'


# ----------------------------------------------------------------------
# Folders
# ----------------------------------------------------------------------

def sanitize_memo(memo: str) -> str:
    """Make a memo safe as a folder-name suffix (spaces -> '-', max 30 chars)."""
    memo = re.sub(r'\s+', '-', memo.strip())
    memo = re.sub(r'[^\w\-가-힣]', '', memo)
    return memo[:30]


def new_session_name(root: Path, today: date, memo: str = '') -> str:
    """``YYYY-MM-DD_rNN[_memo]`` with NN one past the day's highest run."""
    day = today.isoformat()
    runs = [int(m.group(2)) for m in (SESSION_RE.match(p.name) for p in _dirs(root))
            if m and m.group(1) == day]
    name = f'{day}_r{max(runs, default=0) + 1:02d}'
    memo = sanitize_memo(memo)
    return f'{name}_{memo}' if memo else name


def create_session(root: Path, name: str, meta: Dict[str, Any]) -> Path:
    """Create the folder (never reusing an existing one) and write session.yaml."""
    path = Path(root) / name
    path.mkdir(parents=True, exist_ok=False)
    (path / 'logs').mkdir()
    match = SESSION_RE.match(name)
    document = dict(meta)
    document.update({
        'name': name,
        'date': match.group(1) if match else '',
        'run': int(match.group(2)) if match else 0,
        'memo': (match.group(3) or '') if match else '',
        'created': datetime.now().isoformat(timespec='seconds'),
    })
    save_yaml(path / SESSION_FILE, document)
    return path


@dataclass
class SessionInfo:
    """What the session list shows for one folder."""

    name: str
    path: Path
    date: str = ''
    run: int = 0
    memo: str = ''
    sensors: List[str] = field(default_factory=list)
    ground: Dict[str, Any] = field(default_factory=dict)      # sensor -> result
    captures: Dict[str, int] = field(default_factory=dict)    # sensor -> count
    rms: Dict[str, float] = field(default_factory=dict)       # sensor -> fit RMS
    applied: str = ''                                         # applied time, '' if not

    @property
    def label(self) -> str:
        text = f'{self.date} {self.run}회차' if self.date else self.name
        return f'{text} ({self.memo})' if self.memo else text


def read_session(path: Path) -> SessionInfo:
    """Summarise one session folder from the files the steps leave behind."""
    path = Path(path)
    meta = load_yaml(path / SESSION_FILE, {})
    match = SESSION_RE.match(path.name)
    info = SessionInfo(
        name=path.name, path=path,
        date=meta.get('date') or (match.group(1) if match else ''),
        run=int(meta.get('run') or (match.group(2) if match else 0)),
        memo=meta.get('memo') or ((match.group(3) or '') if match else ''),
        sensors=list(meta.get('sensors') or []),
    )
    info.ground = dict((load_yaml(path / GROUND_FILE, {}) or {}).get('results') or {})
    for sensor in info.sensors:
        store = load_yaml(path / f'{sensor}_captures.yaml', {}) or {}
        info.captures[sensor] = len(store.get('captured') or [])
    result = (load_yaml(path / RESULT_FILE, {}) or {}).get('sensors') or {}
    for sensor, block in result.items():
        rms = (block.get('calibration') or {}).get('rms_error')
        if rms is not None:
            info.rms[sensor] = float(rms)
    info.applied = str((load_yaml(path / APPLY_FILE, {}) or {}).get('time') or '')
    return info


def list_sessions(root: Path) -> List[SessionInfo]:
    """Sessions under ``root``, newest first (by folder name)."""
    names = sorted((p for p in _dirs(root) if (p / SESSION_FILE).is_file()),
                   key=lambda p: p.name, reverse=True)
    return [read_session(p) for p in names]


def _dirs(root: Path) -> List[Path]:
    root = Path(root)
    return [p for p in root.iterdir() if p.is_dir()] if root.is_dir() else []


# ----------------------------------------------------------------------
# Lock: one GUI per session
# ----------------------------------------------------------------------

def acquire_lock(path: Path) -> Optional[str]:
    """Take the session lock. Returns ``None`` on success, else who holds it.

    A lock left by a process that no longer runs on this host is taken over.
    """
    lock = Path(path) / LOCK_FILE
    holder = load_yaml(lock, None)
    if holder:
        pid, host = int(holder.get('pid', 0)), holder.get('host', '')
        if pid and pid != os.getpid() and host == socket.gethostname() and _alive(pid):
            return f"pid {pid} ({holder.get('since', '')})"
    save_yaml(lock, {'pid': os.getpid(), 'host': socket.gethostname(),
                     'since': datetime.now().isoformat(timespec='seconds')})
    return None


def release_lock(path: Path) -> None:
    lock = Path(path) / LOCK_FILE
    holder = load_yaml(lock, None)
    if holder and int(holder.get('pid', 0)) == os.getpid():
        lock.unlink(missing_ok=True)


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


# ----------------------------------------------------------------------
# YAML helpers
# ----------------------------------------------------------------------

def load_yaml(path: Path, default: Any) -> Any:
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            value = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError):
        return default
    return default if value is None else value


def save_yaml(path: Path, data: Any) -> None:
    """Write atomically so a crash never leaves half a file behind."""
    path = Path(path)
    temporary = path.with_name(f'{path.name}.{os.getpid()}.tmp')
    with open(temporary, 'w', encoding='utf-8') as handle:
        yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False)
    os.replace(temporary, path)


# ----------------------------------------------------------------------
# Target and checks
# ----------------------------------------------------------------------

# A capture whose sensor-to-target distance differs from the nominal layout by
# more than this is probably another object (2026-11-03: 0.66 m, a net post).
# Range, unlike bearing, does not depend on the nominal yaw being right.
RANGE_WARN_M = 0.4
RMS_PASS_M = 0.02
YAW_GAP_WARN_DEG = 10.0


def target_settings(diameter_m: float, height_m: float) -> Dict[str, float]:
    """Detection values for an upright cylinder of the given size.

    The band skips the floor (10 cm) and the top edge (8 cm); the extent limit is
    the band height or the diameter, whichever is larger, plus 10 cm, so bigger
    objects (people, chairs, walls) are rejected. Matches the values used on
    2026-10-29 / 11-03 for the 10 cm x 50 cm cylinder.
    """
    low, high = 0.10, max(0.20, height_m - 0.08)
    return {
        'radius': round(diameter_m / 2.0, 4),
        'band_low': low,
        'band_high': round(high, 3),
        'cluster_max_extent': round(max(high - low, diameter_m) + 0.10, 3),
        'max_fit_residual': 0.03 if diameter_m <= 0.12 else 0.04,
    }


def range_error(measured_xy, grid_xy, nominal) -> float:
    """Measured minus expected sensor-to-target distance [m]."""
    nx, ny = nominal[0], nominal[1]
    expected = math.hypot(grid_xy[0] - nx, grid_xy[1] - ny)
    return math.hypot(measured_xy[0], measured_xy[1]) - expected


def calibration_verdict(solutions: Dict[str, Dict[str, float]],
                        nominal: Dict[str, List[float]],
                        problems: Dict[str, List[str]],
                        sensors: List[str]) -> Dict[str, Any]:
    """Pass/fail plus the numbers worth checking with a tape measure.

    ``nominal`` holds ``[x, y, yaw_deg]`` per sensor, ``problems`` per-sensor
    notes from the captures (bad spread, wrong distance, outlier point).
    """
    lines, ok = [], True
    for name in sensors:
        sol = solutions.get(name)
        if not sol:
            lines.append(f'{name}: 아직 계산 안 됨')
            ok = False
            continue
        nx, ny, nyaw = nominal.get(name, [sol['x'], sol['y'], sol['yaw_deg']])
        moved = math.hypot(sol['x'] - nx, sol['y'] - ny)
        dyaw = (sol['yaw_deg'] - nyaw + 180.0) % 360.0 - 180.0
        good = sol['rms'] <= RMS_PASS_M and not problems.get(name)
        ok = ok and good
        lines.append(f"{name}: RMS {sol['rms'] * 1000:.1f} mm {'합격' if good else '불합격'}"
                     f"   명목 위치와 {moved * 100:.0f} cm, yaw {dyaw:+.1f}° 차이"
                     + (f"   ← {', '.join(problems[name])}" if problems.get(name) else ''))
    solved = [n for n in sensors if solutions.get(n)]
    if len(solved) == 2:
        a, b = (solutions[n] for n in solved)
        gap = abs((a['yaw_deg'] - b['yaw_deg'] + 180.0) % 360.0 - 180.0)
        lines.append(f"센서 사이 거리 {math.hypot(a['x'] - b['x'], a['y'] - b['y']):.3f} m "
                     f"(줄자로 확인)   yaw 차이 {gap:.1f}°"
                     + ('' if abs(gap - 180.0) <= YAW_GAP_WARN_DEG else '  ← 마주 보는 배치면 180° 근처여야 함'))
    return {'ok': ok and len(solved) == len(sensors), 'lines': lines}


# ----------------------------------------------------------------------
# Parsing node output
# ----------------------------------------------------------------------

_GROUND_VALUE_RE = re.compile(r'(\w+)_(z|roll|pitch):\s+(-?\d+\.\d+)')
_INLIER_RE = re.compile(r'inlier frac\s*=\s*([\d.]+)\s*%')
_SIGMA_RE = re.compile(r'residual sigma\s*=\s*([\d.]+)\s*mm')
_STATUS_ROW_RE = re.compile(
    r'^\s*(\d+)\s+\(\s*[-\d.]+,\s*[-\d.]+\)\s+'
    r'\(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)\s+'
    r'(?:([\d.]+) m|--)(?:\s+(?:([\d.]+) m|--))?', re.MULTILINE)
_STATUS_COUNT_RE = re.compile(r'(\d+)/(\d+) target\(s\) captured')
_PREVIEW_RE = re.compile(
    r'(?:preview: )?x=([-\d.]+) m,?\s+y=([-\d.]+) m,?\s+yaw=([-\d.]+) rad '
    r'\(([-\d.]+) deg\),?\s+rms=([\d.]+) m')
_DROP_RE = re.compile(r'dropping point (\d+) takes the RMS from ([\d.]+) m to ([\d.]+) m')
CAPTURE_STD_RE = re.compile(r'\[std ([\d.]+) m')


def parse_ground_output(text: str, label: str) -> Optional[Dict[str, float]]:
    """z / roll / pitch / inlier / sigma from ground_plane_node's report."""
    values = {key: float(v) for lab, key, v in _GROUND_VALUE_RE.findall(text) if lab == label}
    inlier, sigma = _INLIER_RE.search(text), _SIGMA_RE.search(text)
    if len(values) < 3 or not inlier or not sigma:
        return None
    values['inlier'] = float(inlier.group(1))
    values['sigma'] = float(sigma.group(1))
    return values


def parse_status(text: str) -> Dict[str, Any]:
    """Captured rows, residuals and preview fit from the ~/status table.

    Returns ``{'count', 'total', 'rows': {idx: {x, y, z, residual, loo}},
    'preview': {x, y, yaw, yaw_deg, rms} | None, 'drop': idx | None}``.
    """
    out: Dict[str, Any] = {'count': 0, 'total': 0, 'rows': {}, 'preview': None, 'drop': None}
    count = _STATUS_COUNT_RE.search(text)
    if count:
        out['count'], out['total'] = int(count.group(1)), int(count.group(2))
    for m in _STATUS_ROW_RE.finditer(text):
        out['rows'][int(m.group(1))] = {
            'x': float(m.group(2)), 'y': float(m.group(3)), 'z': float(m.group(4)),
            'residual': float(m.group(5)) if m.group(5) else None,
            'loo': float(m.group(6)) if m.group(6) else None,
        }
    preview = _PREVIEW_RE.search(text)
    if preview:
        out['preview'] = parse_solution(preview.group(0))
    drop = _DROP_RE.search(text)
    if drop:
        out['drop'] = int(drop.group(1))
    return out


def parse_solution(text: str) -> Optional[Dict[str, float]]:
    """x / y / yaw / rms from a run_calibration response or status preview."""
    m = _PREVIEW_RE.search(text)
    if not m:
        return None
    return {'x': float(m.group(1)), 'y': float(m.group(2)), 'yaw': float(m.group(3)),
            'yaw_deg': float(m.group(4)), 'rms': float(m.group(5))}
