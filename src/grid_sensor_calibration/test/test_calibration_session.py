"""Tests for calibration session folders and node-output parsing."""

from __future__ import annotations

import os
from datetime import date

from grid_sensor_calibration.calibration_session import (
    LOCK_FILE,
    acquire_lock,
    create_session,
    list_sessions,
    new_session_name,
    parse_ground_output,
    parse_solution,
    parse_status,
    release_lock,
    sanitize_memo,
    save_yaml,
)

# ~/status as the node printed it on 2026-10-29 (one sensor, four points).
STATUS = """
sensor1: 4/4 target(s) captured
  store: /home/jaeyup/grid_calibration/indoor_cage_20261029_211011/sensor1_captures.yaml
  tilt correction: enabled z=1.267 m, roll=0.010077 rad (+0.577 deg), pitch=-0.016381 rad (-0.939 deg)

  idx  grid (x, y)        sensor (x, y, z)              residual   rms if dropped
  ------------------------------------------------------------------------------
    0  (  0.00,  0.00)   ( -1.380, -1.335, -1.006)          0.011 m      0.011 m
    1  (  3.00,  0.00)   ( -1.302,  1.668, -0.981)          0.009 m      0.013 m
    2  (  0.00,  1.50)   ( -2.856, -1.285, -0.992)          0.016 m      0.009 m
    3  (  3.00,  1.50)   ( -2.785,  1.693, -0.992)          0.013 m      0.012 m

  preview: x=1.356 m  y=-1.319 m  yaw=-1.545962 rad (-88.577 deg)  rms=0.0126 m
"""

PARTIAL = """
sensor1: 1/4 target(s) captured
  idx  grid (x, y)        sensor (x, y, z)              residual
  --------------------------------------------------------------
    0  (  0.00,  0.00)   ( -1.380, -1.335, -1.006)              --
    1  (  3.00,  0.00)   -- not captured --
"""

GROUND = """
  inlier frac    =    89.9 %
  residual sigma =       5 mm     (ground roughness + range noise)
    lidar1_z:     1.267
    lidar1_roll:  0.010077
    lidar1_pitch: -0.016381
"""


def test_session_names_count_runs_per_day(tmp_path):
    day = date(2026, 10, 31)
    assert new_session_name(tmp_path, day) == '2026-10-31_r01'
    create_session(tmp_path, '2026-10-31_r01', {'sensors': ['sensor1']})
    create_session(tmp_path, '2026-10-30_r05', {'sensors': ['sensor1']})
    assert new_session_name(tmp_path, day, 'desk test') == '2026-10-31_r02_desk-test'


def test_memo_is_folder_safe():
    assert sanitize_memo(' 책상 위 / 시험 #1 ') == '책상-위--시험-1'
    assert len(sanitize_memo('x' * 50)) == 30


def test_list_sessions_reads_step_files(tmp_path):
    path = create_session(tmp_path, '2026-10-31_r01_memo', {'sensors': ['sensor1']})
    save_yaml(path / 'step1_ground.yaml',
              {'results': {'sensor1': {'z': 1.267, 'ok': True}}})
    save_yaml(path / 'sensor1_captures.yaml', {'captured': [[0, 0, 0], [1, 1, 0]]})
    save_yaml(path / 'sensors_calibration.yaml',
              {'sensors': {'sensor1': {'calibration': {'rms_error': 0.0126}}}})
    [info] = list_sessions(tmp_path)
    assert info.label == '2026-10-31 1회차 (memo)'
    assert info.ground['sensor1']['z'] == 1.267
    assert info.captures == {'sensor1': 2}
    assert info.rms == {'sensor1': 0.0126}
    assert info.applied == ''


def test_lock_blocks_live_holder_and_takes_stale(tmp_path):
    assert acquire_lock(tmp_path) is None
    release_lock(tmp_path)
    assert not (tmp_path / LOCK_FILE).exists()
    import socket
    save_yaml(tmp_path / LOCK_FILE, {'pid': os.getppid(), 'host': socket.gethostname()})
    assert acquire_lock(tmp_path) is not None          # parent process is alive
    save_yaml(tmp_path / LOCK_FILE, {'pid': 2 ** 22 + 12345, 'host': socket.gethostname()})
    assert acquire_lock(tmp_path) is None              # dead holder is taken over


def test_parse_ground():
    values = parse_ground_output(GROUND, 'lidar1')
    assert values == {'z': 1.267, 'roll': 0.010077, 'pitch': -0.016381,
                      'inlier': 89.9, 'sigma': 5.0}
    assert parse_ground_output(GROUND, 'lidar2') is None


def test_parse_status_full_and_partial():
    full = parse_status(STATUS)
    assert (full['count'], full['total']) == (4, 4)
    assert full['rows'][2] == {'x': -2.856, 'y': -1.285, 'z': -0.992,
                               'residual': 0.016, 'loo': 0.009}
    assert full['preview']['rms'] == 0.0126 and full['drop'] is None
    partial = parse_status(PARTIAL)
    assert partial['count'] == 1 and list(partial['rows']) == [0]
    assert partial['rows'][0]['residual'] is None and partial['preview'] is None


def test_parse_solution_from_run_calibration():
    text = ('sensor1: x=1.356 m, y=-1.319 m, yaw=-1.545962 rad (-88.577 deg), '
            'rms=0.0126 m [tilt z=1.267 m, roll=0.010077, pitch=-0.016381]')
    assert parse_solution(text) == {'x': 1.356, 'y': -1.319, 'yaw': -1.545962,
                                     'yaw_deg': -88.577, 'rms': 0.0126}


def test_target_settings_small_cylinder_and_tube():
    from grid_sensor_calibration.calibration_session import target_settings
    small = target_settings(0.10, 0.5)
    assert small == {'radius': 0.05, 'band_low': 0.1, 'band_high': 0.42,
                     'cluster_max_extent': 0.42, 'max_fit_residual': 0.03}
    tube = target_settings(0.30, 1.8)
    assert tube['radius'] == 0.15 and tube['band_high'] == 1.72
    assert tube['cluster_max_extent'] == 1.72 and tube['max_fit_residual'] == 0.04


def test_range_error_flags_the_net_post_capture():
    from grid_sensor_calibration.calibration_session import RANGE_WARN_M, range_error
    nominal = [1.35, 3.02, 90.0]
    # 2026-11-03 sensor2 point 0: cylinder at (-3.06, 1.23) vs net post at (-3.087, 2.500)
    assert abs(range_error((-3.06, 1.23), (0.0, 0.0), nominal)) < RANGE_WARN_M
    assert range_error((-3.087, 2.500), (0.0, 0.0), nominal) > RANGE_WARN_M


def test_verdict_pass_and_sensor_gap():
    from grid_sensor_calibration.calibration_session import calibration_verdict
    sols = {'sensor1': {'x': 1.354, 'y': -1.340, 'yaw_deg': -89.13, 'rms': 0.0158},
            'sensor2': {'x': 1.376, 'y': 3.054, 'yaw_deg': 87.85, 'rms': 0.0152}}
    nominal = {'sensor1': [1.35, -1.2, -90.0], 'sensor2': [1.35, 3.02, 90.0]}
    v = calibration_verdict(sols, nominal, {}, ['sensor1', 'sensor2'])
    assert v['ok'] and '4.394 m' in v['lines'][-1] and '177.0°' in v['lines'][-1]
    v = calibration_verdict(sols, nominal, {'sensor2': ['거리 안 맞는 캡처']}, ['sensor1', 'sensor2'])
    assert not v['ok']
    v = calibration_verdict({'sensor1': sols['sensor1']}, nominal, {}, ['sensor1', 'sensor2'])
    assert not v['ok']
