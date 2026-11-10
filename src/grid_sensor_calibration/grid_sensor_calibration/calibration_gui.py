#!/usr/bin/env python3
"""Field calibration GUI -- sessions, then three steps, each on its own screen.

    처음 화면     새 세션(날짜·그날 회차·메모로 폴더 생성) 또는 이전 세션 이어서
    1. 지면 보정  센서별 측정 파라미터 입력 -> ground_plane_node -> z / roll / pitch
    2. XY / yaw   설정 화면: 기준점 개수·좌표, 원기둥·검출 범위, 명목 위치
                  캡처 화면: 점 선택 -> 캡처 / 다시 찍기 / 마지막 삭제 / 초기화 / 계산
    3. YAML 적용  결과 -> 트래커 파라미터 YAML (미리보기, 백업)

It only drives the existing nodes: each runs as its own
subprocess, the GUI talks to them over their topics and Trigger services, and
Tkinter owns the main thread while rclpy spins in a background one. Everything
a session produces -- parameters used, results, captures, logs -- stays in its
own folder (see calibration_session), so runs never mix.

    ros2 run grid_sensor_calibration calibration_gui
    ros2 run grid_sensor_calibration calibration_gui --ros-args -p site_file:=<site.yaml>

The LiDAR drivers and ptp4l must already be running (README.md, section 3).
"""

from __future__ import annotations

import math
import os
import queue
import re
import shutil
import signal
import subprocess
import threading
from collections import deque
from datetime import date, datetime
from pathlib import Path

import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tkinter.scrolledtext import ScrolledText

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from .apply_calibration_node import apply_to_yaml_text, tracker_values
from .calibration_io import load_sensor_transforms
from .calibration_math import grid_to_sensor
from .calibration_session import (
    APPLY_FILE, CAPTURE_STD_RE, GROUND_FILE, NOTES_FILE, RANGE_WARN_M, RESULT_FILE,
    SESSION_FILE, SETUP_FILE, acquire_lock, calibration_verdict, create_session,
    list_sessions, load_yaml, new_session_name, parse_ground_output, parse_solution,
    parse_status, range_error, read_session, release_lock, save_yaml, target_settings,
)

PACKAGE = 'grid_sensor_calibration'

# Above this spread a capture probably mixed two objects (seen 0.84 m once).
CAPTURE_STD_WARN_M = 0.02
RMS_GOOD_M = 0.02

GROUND_FIELDS = [   # key, header, width
    ('expected_height', '예상 높이 (m)', 8),
    ('num_frames', '프레임 수', 6),
    ('range_max', '최대 거리 (m)', 8),
    ('azimuth_min', '방위 시작 (°)', 8),
    ('azimuth_max', '방위 끝 (°)', 8),
]
TARGET_FIELDS = [   # key, label
    ('radius', '원기둥 반지름 (m)'),
    ('band_low', '바닥 위 높이 하한 (m)'),
    ('band_high', '바닥 위 높이 상한 (m)'),
    ('range_min', '최소 거리 (m)'),
    ('range_max', '최대 거리 (m)'),
    ('azimuth_min', '방위 시작 (°)'),
    ('azimuth_max', '방위 끝 (°)'),
    ('cluster_tolerance', '묶음 거리 (m)'),
    ('cluster_max_extent', '최대 크기 (m)'),
    ('max_fit_residual', '원 맞춤 잔차 상한 (m)'),
    ('capture_sample_count', '캡처 평균 샘플 수'),
]
GREEN, RED, BLUE = '#11772d', '#c0392b', '#1f4e9a'
CAPTURED_INDEX_RE = re.compile(r'captured point (\d+)/')


class CalibrationGUI(Node):

    started_names = set()   # node names passed to ros2_run, see free_node_names

    def __init__(self):

        super().__init__('calibration_gui')

        default_site = os.path.join(
            get_package_share_directory(PACKAGE), 'config', 'site_indoor_cage.yaml')
        self.declare_parameter('site_file', default_site)
        # Preselected sensors for a new session; empty = all in the site file.
        self.declare_parameter('sensors', [''])
        self.declare_parameter('tracker_params_file', '')

        self.site_file = os.path.expanduser(self.get_parameter('site_file').value)
        self.site = load_yaml(Path(self.site_file), None)
        if not self.site:
            raise RuntimeError(f'cannot read site file {self.site_file}')
        self.site_sensors = {s['name']: s for s in self.site['sensors']}
        preselect = [s for s in self.get_parameter('sensors').value if s]
        self.preselect = preselect or list(self.site_sensors)
        self.default_tracker_yaml = os.path.expanduser(
            self.get_parameter('tracker_params_file').value or self.site['tracker_params_file'])
        self.site_root = Path(os.path.expanduser(self.site['results_dir'])) / \
            self.site.get('session_prefix', 'field')

        # ======================================================
        # Session state
        # ======================================================

        self.session = None             # Path of the open session
        self.sensors = []               # site sensor dicts used by the session
        self.ground = {}                # name -> result dict (z, roll, pitch, ...)
        self.setup = None               # step-2 setup dict once detection started
        self.status = {}                # name -> parse_status() of the latest ~/status
        self.solutions = {}             # name -> parse_solution() of run_calibration
        self.processes = {}             # label -> Popen
        self.residual = {}              # name -> last cylinder fit residual [mm]
        self.detections = {n: deque(maxlen=20) for n in self.site_sensors}
        self.events = queue.Queue()     # results from worker threads / service futures
        self.busy = 0                   # outstanding service calls

        # ======================================================
        # ROS: detections, calibration services, capture_index parameter
        # ======================================================

        self.srv_clients = {}
        for name in self.site_sensors:
            self.create_subscription(
                PointStamped, f'/{name}/target_point',
                lambda msg, n=name: self.detections[n].append(
                    (self.now(), msg.point.x, msg.point.y)),
                10)
            clients = {srv: self.create_client(Trigger, f'/{name}_calibration/{srv}')
                       for srv in ('capture_point', 'undo_point', 'reset',
                                   'run_calibration', 'status')}
            clients['set_parameters'] = self.create_client(
                SetParameters, f'/{name}_calibration/set_parameters')
            self.srv_clients[name] = clients

        self.build_gui()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


    # ==========================================================
    # GUI construction
    # ==========================================================

    def build_gui(self):

        self.root = tk.Tk()
        self.root.title('LiDAR Field Calibration')
        self.root.geometry('1100x940')

        tk.Label(self.root, text='LiDAR 필드 캘리브레이션',
                 font=('Arial', 20, 'bold')).pack(pady=(12, 0))
        tk.Label(self.root, text=f"{self.site.get('site_name', '')}   |   현장 설정 {self.site_file}",
                 font=('Arial', 10)).pack()
        self.session_var = tk.StringVar(value='세션 없음 — 새 세션을 만들거나 이전 세션을 여세요')
        tk.Label(self.root, textvariable=self.session_var, font=('Arial', 12, 'bold'),
                 fg=BLUE).pack(pady=(4, 6))

        self.body = tk.Frame(self.root)
        self.body.pack(fill='both', expand=True, padx=14)
        self.pages = {}
        for name, builder in (('home', self.build_home), ('ground', self.build_ground),
                              ('setup', self.build_setup), ('capture', self.build_capture),
                              ('apply', self.build_apply)):
            frame = tk.Frame(self.body)
            builder(frame)
            self.pages[name] = frame

        self.log_box = ScrolledText(self.root, height=8, font=('Courier', 9))
        self.log_box.pack(fill='x', padx=14, pady=(4, 12))

        self.show('home')
        self.root.protocol('WM_DELETE_WINDOW', self.on_gui_close)
        self.root.after(200, self.poll)

    def show(self, name):
        for frame in self.pages.values():
            frame.pack_forget()
        self.pages[name].pack(fill='both', expand=True)
        self.current_page = name

    @staticmethod
    def section(parent, title):
        frame = tk.LabelFrame(parent, text=f' {title} ', font=('Arial', 12, 'bold'))
        frame.pack(fill='x', pady=5)
        return frame

    @staticmethod
    def page_title(parent, title, hint):
        tk.Label(parent, text=title, font=('Arial', 16, 'bold')).pack(anchor='w')
        tk.Label(parent, text=hint, font=('Arial', 10), justify='left', fg='#555',
                 wraplength=1040).pack(anchor='w', pady=(0, 4))

    @staticmethod
    def tree(parent, columns, height):
        tree = ttk.Treeview(parent, columns=[c for c, _ in columns], show='headings',
                            height=height, selectmode='browse')
        for column, width in columns:
            tree.heading(column, text=column)
            tree.column(column, width=width, anchor='center')
        tree.tag_configure('bad', foreground=RED)
        tree.tag_configure('good', foreground=GREEN)
        tree.tag_configure('next', foreground=BLUE)
        tree.pack(fill='x', padx=8, pady=6)
        return tree

    # ------------------------------------------------------
    # Home
    # ------------------------------------------------------

    def build_home(self, page):

        box = self.section(page, '새 캘리브레이션')
        row = tk.Frame(box)
        row.pack(fill='x', padx=8, pady=4)
        tk.Label(row, text='사용할 센서:').pack(side='left')
        self.sensor_vars = {}
        for name, sensor in self.site_sensors.items():
            var = tk.BooleanVar(value=name in self.preselect)
            tk.Checkbutton(row, text=f"{name} ({sensor['frame']}, {sensor['topic']})",
                           variable=var, command=self.update_preview).pack(side='left', padx=6)
            self.sensor_vars[name] = var
        row = tk.Frame(box)
        row.pack(fill='x', padx=8, pady=4)
        tk.Label(row, text='메모 (선택, 예: 케이지 설치 후):').pack(side='left')
        self.memo_var = tk.StringVar()
        entry = tk.Entry(row, textvariable=self.memo_var, width=30)
        entry.pack(side='left', padx=6)
        entry.bind('<KeyRelease>', lambda _e: self.update_preview())
        self.preview_var = tk.StringVar()
        tk.Label(box, textvariable=self.preview_var, font=('Courier', 10), anchor='w',
                 justify='left').pack(fill='x', padx=8)
        tk.Button(box, text='새 세션 만들기', width=18, height=2,
                  command=self.new_session).pack(pady=6)

        box = self.section(page, '이전 세션 이어서 하기')
        self.session_tree = self.tree(box, [('세션', 260), ('센서', 120), ('1. 지면', 150),
                                            ('2. 캡처', 120), ('2. 계산 RMS', 160),
                                            ('3. 적용', 150)], 6)
        self.session_tree.bind('<Double-1>', lambda _e: self.open_selected_session())
        row = tk.Frame(box)
        row.pack(pady=4)
        tk.Button(row, text='선택 세션 열기', width=16, command=self.open_selected_session).pack(side='left', padx=6)
        tk.Button(row, text='목록 새로고침', width=16, command=self.refresh_sessions).pack(side='left', padx=6)

        box = self.section(page, '단계')
        grid = tk.Frame(box)
        grid.pack(pady=8)
        self.step_buttons, self.step_status = [], []
        for col, (text, page_name) in enumerate((('1. 지면 보정\n(z / roll / pitch)', 'ground'),
                                                 ('2. XY / yaw 보정\n(기준점 캡처)', 'setup'),
                                                 ('3. YAML 적용\n(트래커 파라미터)', 'apply'))):
            button = tk.Button(grid, text=text, width=24, height=3, state=tk.DISABLED,
                               command=lambda p=page_name: self.enter(p))
            button.grid(row=0, column=col, padx=10)
            status = tk.Label(grid, text='', font=('Arial', 10), wraplength=230, justify='center')
            status.grid(row=1, column=col, padx=10, pady=4)
            self.step_buttons.append(button)
            self.step_status.append(status)

        self.update_preview()
        self.refresh_sessions()

    def update_preview(self):
        name = new_session_name(self.site_root, date.today(), self.memo_var.get())
        chosen = [n for n, v in self.sensor_vars.items() if v.get()]
        self.preview_var.set(f'만들 폴더: {self.site_root / name}\n센서: {", ".join(chosen) or "(없음)"}')

    def refresh_sessions(self):
        self.session_tree.delete(*self.session_tree.get_children())
        for info in list_sessions(self.site_root):
            ground = ', '.join(f"{n} {r.get('z', 0):.3f}" for n, r in info.ground.items()) or '-'
            captures = ', '.join(f'{n} {c}' for n, c in info.captures.items() if c) or '-'
            rms = ', '.join(f'{n} {v * 1000:.1f} mm' for n, v in info.rms.items()) or '-'
            self.session_tree.insert('', 'end', iid=str(info.path), values=(
                info.label, ', '.join(info.sensors), ground, captures, rms,
                info.applied[11:16] if info.applied else '-'))

    def refresh_steps(self):
        if not self.session:
            return
        info = read_session(self.session)
        ok = [n for n in info.ground if info.ground[n].get('ok')]
        self.step_status[0].config(
            text=('완료: ' + ', '.join(f"{n} z {info.ground[n]['z']:.3f}" for n in info.ground)
                  if info.ground else '아직 안 함') + ('' if len(ok) == len(info.ground) else ' (불합격 있음)'),
            fg=GREEN if info.ground and len(info.ground) >= len(self.sensors) else 'black')
        total = len(load_yaml(self.session / SETUP_FILE, {}).get('grid_points') or [])
        captured = ', '.join(f'{n} {c}/{total or "?"}' for n, c in info.captures.items())
        rms = ', '.join(f'{n} RMS {v * 1000:.1f} mm' for n, v in info.rms.items())
        self.step_status[1].config(text=f"캡처 {captured or '없음'}" + (f'\n{rms}' if rms else ''),
                                   fg=GREEN if info.rms else 'black')
        self.step_status[2].config(text=f'적용함 {info.applied}' if info.applied else '아직 안 함',
                                   fg=GREEN if info.applied else 'black')
        self.step_buttons[1].config(state=tk.NORMAL if self.ground_ready() else tk.DISABLED)
        self.step_buttons[2].config(state=tk.NORMAL if info.rms else tk.DISABLED)

    # ------------------------------------------------------
    # 1. Ground plane
    # ------------------------------------------------------

    def build_ground(self, page):

        self.page_title(page, '1. 지면 보정 — z / roll / pitch',
                        'RANSAC 으로 바닥 평면을 맞춰 센서 높이와 기울기를 구한다. 측정하는 동안 센서 앞 바닥을 '
                        '비워 둔다. 방위는 센서 원시 좌표 기준이고 130 → -130 은 센서 -x 쪽 100° 부채꼴이다. '
                        'inlier 가 낮으면 옆 벽·책상이 섞인 것이니 방위나 최대 거리를 좁힌다.')
        box = self.section(page, '측정 파라미터')
        self.ground_grid = tk.Frame(box)
        self.ground_grid.pack(fill='x', padx=8, pady=4)
        row = tk.Frame(box)
        row.pack(fill='x', padx=8, pady=4)
        self.inlier_var = tk.StringVar()
        self.sigma_var = tk.StringVar()
        tk.Label(row, text='합격 기준: inlier ≥').pack(side='left')
        tk.Entry(row, textvariable=self.inlier_var, width=6).pack(side='left')
        tk.Label(row, text='%   sigma ≤').pack(side='left')
        tk.Entry(row, textvariable=self.sigma_var, width=6).pack(side='left')
        tk.Label(row, text='mm').pack(side='left')

        row = tk.Frame(page)
        row.pack(pady=6)
        self.ground_run_button = tk.Button(row, text='측정 시작', width=18, height=2,
                                           command=self.run_ground)
        self.ground_run_button.pack(side='left', padx=6)
        tk.Button(row, text='← 메뉴', width=12, height=2,
                  command=lambda: self.enter('home')).pack(side='left', padx=6)

        box = self.section(page, '결과 (마지막 측정이 2단계에 쓰인다)')
        self.ground_tree = self.tree(box, [('센서', 120), ('z (m)', 100), ('roll (°)', 100),
                                           ('pitch (°)', 100), ('inlier (%)', 100),
                                           ('sigma (mm)', 100), ('판정', 120), ('측정 시각', 160)], 3)

    def fill_ground_page(self):
        for child in self.ground_grid.winfo_children():
            child.destroy()
        saved = load_yaml(self.session / GROUND_FILE, {})
        params = saved.get('params') or {}
        cfg = self.site['ground']
        headers = ['센서', '토픽'] + [h for _, h, _ in GROUND_FIELDS]
        for col, text in enumerate(headers):
            tk.Label(self.ground_grid, text=text, font=('Arial', 10, 'bold')).grid(row=0, column=col, padx=4)
        self.ground_vars = {}
        for r, sensor in enumerate(self.sensors, start=1):
            name = sensor['name']
            tk.Label(self.ground_grid, text=name).grid(row=r, column=0)
            tk.Label(self.ground_grid, text=sensor['topic']).grid(row=r, column=1)
            self.ground_vars[name] = {}
            for col, (key, _, width) in enumerate(GROUND_FIELDS, start=2):
                default = sensor['expected_height'] if key == 'expected_height' else cfg[key]
                var = tk.StringVar(value=str(params.get(name, {}).get(key, default)))
                tk.Entry(self.ground_grid, textvariable=var, width=width,
                         justify='center').grid(row=r, column=col, padx=4, pady=2)
                self.ground_vars[name][key] = var
        thresholds = saved.get('thresholds') or {}
        self.inlier_var.set(str(thresholds.get('min_inlier_percent', cfg['min_inlier_percent'])))
        self.sigma_var.set(str(thresholds.get('max_sigma_mm', cfg['max_sigma_mm'])))
        self.ground_tree.delete(*self.ground_tree.get_children())
        for name, result in (saved.get('results') or {}).items():
            self.show_ground_row(name, result)

    def show_ground_row(self, name, r):
        if self.ground_tree.exists(name):
            self.ground_tree.delete(name)
        if r is None:
            self.ground_tree.insert('', 'end', iid=name, values=(name, '측정 중...'))
            return
        if 'error' in r:
            self.ground_tree.insert('', 'end', iid=name, tags=('bad',),
                                    values=(name, '실패', '', '', '', '', r['error'][:20], r.get('time', '')))
            return
        self.ground_tree.insert('', 'end', iid=name, tags=('good' if r['ok'] else 'bad',), values=(
            name, f"{r['z']:.3f}", f"{math.degrees(r['roll']):+.3f}", f"{math.degrees(r['pitch']):+.3f}",
            f"{r['inlier']:.1f}", f"{r['sigma']:.0f}", '합격' if r['ok'] else '불합격', r.get('time', '')))

    def run_ground(self):

        try:
            params = {name: {key: float(var.get()) for key, var in fields.items()}
                      for name, fields in self.ground_vars.items()}
            thresholds = {'min_inlier_percent': float(self.inlier_var.get()),
                          'max_sigma_mm': float(self.sigma_var.get())}
        except ValueError:
            messagebox.showerror('입력 오류', '숫자가 아닌 칸이 있습니다.')
            return
        if not self.free_node_names([f"ground_plane_{s['frame']}" for s in self.sensors]):
            return
        self.ground_params, self.ground_thresholds = params, thresholds
        self.ground_pending = {s['name'] for s in self.sensors}
        self.ground_run_button.config(state=tk.DISABLED)
        stamp = datetime.now().strftime('%H%M%S')
        for sensor in self.sensors:
            name, p = sensor['name'], params[sensor['name']]
            self.show_ground_row(name, None)
            command = self.ros2_run('ground_plane_node', {
                'input_topic': sensor['topic'], 'sensor_label': sensor['frame'],
                'expected_height': p['expected_height'], 'num_frames': int(p['num_frames']),
                'range_max': p['range_max'], 'azimuth_min': p['azimuth_min'],
                'azimuth_max': p['azimuth_max'],
            }, node_name=f"ground_plane_{sensor['frame']}")
            log = self.session / 'logs' / f'ground_{name}_{stamp}.log'
            threading.Thread(target=self.ground_worker, args=(sensor, command, log),
                             daemon=True).start()
        self.log('1. 지면 보정 측정 시작 — 센서 앞 바닥을 비워 두세요')

    def ground_worker(self, sensor, command, log):
        try:
            done = subprocess.run(command, capture_output=True, text=True, timeout=90,
                                  env=self.child_env())
            text = done.stdout + done.stderr
        except subprocess.TimeoutExpired:
            text = 'timeout after 90 s'
        log.write_text(text)
        values = parse_ground_output(text, sensor['frame'])
        self.events.put(('ground', sensor['name'], values, text.strip().splitlines()[-1:] or ['']))

    def on_ground(self, name, values, tail):
        stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        if values is None:
            result = {'error': tail[0], 'time': stamp}
            self.log(f'{name} 지면 보정 실패: {tail[0]} (logs/ 확인)')
        else:
            th = self.ground_thresholds
            values['ok'] = (values['inlier'] >= th['min_inlier_percent']
                            and values['sigma'] <= th['max_sigma_mm'])
            values['time'] = stamp
            result = values
            self.ground[name] = values
            self.log(f"{name} 지면: z={values['z']:.3f} roll={values['roll']:.6f} "
                     f"pitch={values['pitch']:.6f} inlier {values['inlier']:.1f}% "
                     f"sigma {values['sigma']:.0f} mm -> {'합격' if values['ok'] else '불합격'}")
        self.show_ground_row(name, result)
        saved = load_yaml(self.session / GROUND_FILE, {})
        saved['params'] = self.ground_params
        saved['thresholds'] = self.ground_thresholds
        saved.setdefault('results', {})
        if values is not None:
            saved['results'][name] = values
        saved.setdefault('history', []).append(
            {'sensor': name, 'params': self.ground_params[name], 'result': result})
        save_yaml(self.session / GROUND_FILE, saved)
        self.ground_pending.discard(name)
        if not self.ground_pending:
            self.ground_run_button.config(state=tk.NORMAL)
            self.refresh_steps()

    def ground_ready(self):
        return all(s['name'] in self.ground for s in self.sensors)

    # ------------------------------------------------------
    # 2a. XY / yaw setup
    # ------------------------------------------------------

    def build_setup(self, page):

        self.page_title(page, '2. XY / yaw 보정 — 설정',
                        '기준점 개수와 필드 좌표를 정한다 (캡처 순서 = 표 순서). 넓게 퍼뜨릴수록 yaw 가 정확하다. '
                        '원통 검출 높이는 1단계에서 잰 센서 높이로 자동 계산된다.')
        box = self.section(page, '기준점 (필드 좌표, m)')
        row = tk.Frame(box)
        row.pack(fill='x', padx=8, pady=4)
        tk.Label(row, text='점 개수:').pack(side='left')
        self.npoints_var = tk.StringVar(value='4')
        spin = tk.Spinbox(row, from_=2, to=12, width=4, textvariable=self.npoints_var,
                          command=self.rebuild_point_rows)
        spin.pack(side='left', padx=4)
        spin.bind('<Return>', lambda _e: self.rebuild_point_rows())
        spin.bind('<FocusOut>', lambda _e: self.rebuild_point_rows())
        tk.Button(row, text='현장 기본값 불러오기', command=self.load_site_points).pack(side='left', padx=12)
        self.points_frame = tk.Frame(box)
        self.points_frame.pack(fill='x', padx=8, pady=4)
        self.point_vars = []

        box = self.section(page, '원기둥 · 검출 범위')
        row = tk.Frame(box)
        row.pack(fill='x', padx=8, pady=(4, 0))
        tk.Label(row, text='타깃 크기 →  지름 (cm)').pack(side='left')
        self.diameter_var = tk.StringVar()
        tk.Entry(row, textvariable=self.diameter_var, width=6, justify='center').pack(side='left', padx=4)
        tk.Label(row, text='높이 (m)').pack(side='left')
        self.height_var = tk.StringVar()
        tk.Entry(row, textvariable=self.height_var, width=6, justify='center').pack(side='left', padx=4)
        tk.Button(row, text='아래 검출 값 자동 계산', command=self.apply_target_size).pack(side='left', padx=10)
        tk.Label(row, text='(작은 원기둥 10 cm / 0.5 m, 대회 지관 30 cm / 1.8 m)',
                 fg='#555').pack(side='left')
        grid = tk.Frame(box)
        grid.pack(fill='x', padx=8, pady=4)
        self.target_vars = {}
        for i, (key, label) in enumerate(TARGET_FIELDS):
            r, c = divmod(i, 3)
            tk.Label(grid, text=label, anchor='e', width=20).grid(row=r, column=2 * c, pady=2)
            var = tk.StringVar()
            var.trace_add('write', lambda *_: self.update_box_info())
            tk.Entry(grid, textvariable=var, width=8, justify='center').grid(row=r, column=2 * c + 1, padx=4)
            self.target_vars[key] = var

        box = self.section(page, '센서 명목 위치 (줄자, 예상 좌표 표시와 설치 오차 보고에만 쓰임)')
        self.nominal_frame = tk.Frame(box)
        self.nominal_frame.pack(fill='x', padx=8, pady=4)
        self.box_info = tk.Label(box, text='', font=('Courier', 10), anchor='w', justify='left')
        self.box_info.pack(fill='x', padx=8, pady=4)

        row = tk.Frame(page)
        row.pack(pady=6)
        tk.Button(row, text='검출 · 캡처 시작 →', width=20, height=2,
                  command=self.start_capture).pack(side='left', padx=6)
        tk.Button(row, text='← 메뉴', width=12, height=2,
                  command=lambda: self.enter('home')).pack(side='left', padx=6)

    def fill_setup_page(self):
        saved = load_yaml(self.session / SETUP_FILE, {})
        points = saved.get('grid_points') or self.site['grid_points']
        self.set_point_rows(points)
        target = dict(self.site['target'])
        band = target.pop('band_above_floor')
        target.update(band_low=band[0], band_high=band[1])
        target.update(saved.get('target') or {})
        for key, var in self.target_vars.items():
            var.set(str(target[key]))
        size = saved.get('target_size') or self.site.get('target_size') or \
            [round(2 * float(target['radius']) * 100, 1), round(float(target['band_high']) + 0.08, 2)]
        self.diameter_var.set(f'{float(size[0]):g}')
        self.height_var.set(f'{float(size[1]):g}')
        for child in self.nominal_frame.winfo_children():
            child.destroy()
        self.nominal_vars = {}
        nominal = saved.get('nominal') or {}
        for r, sensor in enumerate(self.sensors):
            name = sensor['name']
            x, y, yaw = nominal.get(name, sensor['nominal'][:2] + [math.degrees(sensor['nominal'][2])])
            tk.Label(self.nominal_frame, text=f'{name}:').grid(row=r, column=0, padx=4)
            self.nominal_vars[name] = []
            for c, (label, value) in enumerate((('x', x), ('y', y), ('yaw (°)', yaw))):
                tk.Label(self.nominal_frame, text=label).grid(row=r, column=1 + 2 * c)
                var = tk.StringVar(value=f'{float(value):g}')
                tk.Entry(self.nominal_frame, textvariable=var, width=9,
                         justify='center').grid(row=r, column=2 + 2 * c, padx=4, pady=2)
                self.nominal_vars[name].append(var)
        self.update_box_info()

    def apply_target_size(self):
        try:
            diameter = float(self.diameter_var.get()) / 100.0
            height = float(self.height_var.get())
        except ValueError:
            messagebox.showerror('입력 오류', '지름(cm)과 높이(m)를 숫자로 넣으세요.')
            return
        for key, value in target_settings(diameter, height).items():
            self.target_vars[key].set(f'{value:g}')
        self.log(f'타깃 지름 {diameter * 100:g} cm, 높이 {height:g} m → 반지름·검출 높이·최대 크기 자동 설정')

    def set_point_rows(self, points):
        self.npoints_var.set(str(len(points)))
        self.rebuild_point_rows(points)

    def load_site_points(self):
        self.set_point_rows(self.site['grid_points'])

    def rebuild_point_rows(self, points=None):
        try:
            count = max(2, min(12, int(self.npoints_var.get())))
        except ValueError:
            return
        old = [(x.get(), y.get()) for x, y in self.point_vars]
        if points is not None:
            old = [(f'{p[0]:g}', f'{p[1]:g}') for p in points]
        for child in self.points_frame.winfo_children():
            child.destroy()
        self.point_vars = []
        for i in range(count):
            r, c = divmod(i, 4)
            cell = tk.Frame(self.points_frame)
            cell.grid(row=r, column=c, padx=10, pady=2, sticky='w')
            tk.Label(cell, text=f'{i}:', width=3).pack(side='left')
            xv = tk.StringVar(value=old[i][0] if i < len(old) else '')
            yv = tk.StringVar(value=old[i][1] if i < len(old) else '')
            tk.Label(cell, text='x').pack(side='left')
            tk.Entry(cell, textvariable=xv, width=6, justify='center').pack(side='left')
            tk.Label(cell, text='y').pack(side='left')
            tk.Entry(cell, textvariable=yv, width=6, justify='center').pack(side='left')
            self.point_vars.append((xv, yv))

    def update_box_info(self):
        try:
            low = float(self.target_vars['band_low'].get())
            high = float(self.target_vars['band_high'].get())
        except (ValueError, KeyError):
            self.box_info.config(text='')
            return
        lines = []
        for sensor in self.sensors:
            g = self.ground.get(sensor['name'])
            if g:
                lines.append(f"{sensor['name']}: 1단계 z {g['z']:.3f} m → 검출 높이 (센서 기준) "
                             f"[{-g['z'] + low:+.3f}, {-g['z'] + high:+.3f}]")
            else:
                lines.append(f"{sensor['name']}: 1단계 결과 없음")
        self.box_info.config(text='\n'.join(lines))

    def read_setup(self):
        points = []
        for i, (xv, yv) in enumerate(self.point_vars):
            try:
                points.append([float(xv.get()), float(yv.get())])
            except ValueError:
                raise ValueError(f'{i}번 점 좌표가 숫자가 아닙니다')
        if len({tuple(p) for p in points}) != len(points):
            raise ValueError('같은 좌표의 점이 있습니다')
        target = {}
        for key, var in self.target_vars.items():
            try:
                target[key] = float(var.get())
            except ValueError:
                raise ValueError(f'"{dict(TARGET_FIELDS)[key]}" 가 숫자가 아닙니다')
        target['capture_sample_count'] = int(target['capture_sample_count'])
        nominal = {}
        for name, vars_ in self.nominal_vars.items():
            try:
                nominal[name] = [float(v.get()) for v in vars_]
            except ValueError:
                raise ValueError(f'{name} 명목 위치가 숫자가 아닙니다')
        try:
            size = [float(self.diameter_var.get()), float(self.height_var.get())]
        except ValueError:
            size = None
        return {'grid_points': points, 'target': target, 'nominal': nominal, 'target_size': size}

    def start_capture(self):

        try:
            setup = self.read_setup()
        except ValueError as exc:
            messagebox.showerror('입력 오류', str(exc))
            return
        if not self.ground_ready():
            messagebox.showerror('1단계 먼저', '모든 센서의 지면 보정 결과가 있어야 합니다.')
            return
        # Captures are positional: a store recorded against other grid points
        # would be paired with the wrong positions (the node refuses to load it).
        stale = []
        for sensor in self.sensors:
            store = self.session / f"{sensor['name']}_captures.yaml"
            stored = (load_yaml(store, {}) or {}).get('grid_points')
            if stored and [list(map(float, p)) for p in stored] != setup['grid_points']:
                stale.append(store)
        if stale:
            if not messagebox.askyesno(
                    '기준점이 바뀜',
                    '이 세션에 다른 기준점으로 찍은 캡처가 있습니다.\n'
                    '이전 캡처를 logs/ 로 옮기고 새로 시작할까요?'):
                return
            stamp = datetime.now().strftime('%H%M%S')
            for store in stale:
                shutil.move(str(store), str(self.session / 'logs' / f'{store.stem}_{stamp}.yaml'))
                self.log(f'이전 캡처 보관: logs/{store.stem}_{stamp}.yaml')

        labels = [f"target_{s['frame']}" for s in self.sensors] + \
                 [f"{s['name']}_calibration" for s in self.sensors]
        self.stop_capture_nodes()
        if not self.free_node_names(labels):
            return
        setup['started'] = datetime.now().isoformat(timespec='seconds')
        save_yaml(self.session / SETUP_FILE, setup)
        self.setup = setup
        target = setup['target']
        flat_grid = [c for p in setup['grid_points'] for c in p]
        for sensor in self.sensors:
            name, frame, g = sensor['name'], sensor['frame'], self.ground[sensor['name']]
            box = [-30.0, 30.0, -30.0, 30.0, -g['z'] + target['band_low'], -g['z'] + target['band_high']]
            self.residual[name] = None
            self.spawn(f'target_{name}', self.ros2_run('pointcloud_target_node', {
                'input_topic': sensor['topic'], 'output_topic': f'/{name}/target_point',
                'frame_id': frame, 'box': box,
                'range_min': target['range_min'], 'range_max': target['range_max'],
                'azimuth_min': target['azimuth_min'], 'azimuth_max': target['azimuth_max'],
                'cluster_tolerance': target['cluster_tolerance'], 'cluster_min_points': 5,
                'cluster_max_extent': target['cluster_max_extent'], 'selection': 'largest',
                'target_shape': 'cylinder', 'target_radius': target['radius'],
                'max_fit_residual': target['max_fit_residual'], 'voxel_size': 0.0,
            }, node_name=f'target_{frame}'), read_residual_for=name)
            nx, ny, nyaw = setup['nominal'][name]
            self.spawn(f'calib_{name}', self.ros2_run('sensor_calibration_node', {
                'sensor_name': name, 'child_frame': frame,
                'input_topic': f'/{name}/target_point', 'input_message_type': 'point_stamped',
                'capture_sample_count': target['capture_sample_count'], 'auto_calibrate': False,
                'grid_points': flat_grid,
                'calibration_file': str(self.session / RESULT_FILE),
                'nominal_x': nx, 'nominal_y': ny, 'nominal_yaw': math.radians(nyaw),
                'apply_tilt_correction': True,
                'sensor_z': g['z'], 'sensor_roll': g['roll'], 'sensor_pitch': g['pitch'],
            }, node_name=f'{name}_calibration'))
            self.log(f"{name}: 원통 검출 높이 [{box[4]:+.3f}, {box[5]:+.3f}] (센서 기준), "
                     f"기준점 {len(setup['grid_points'])}개")
        self.solutions = {}
        self.status = {}
        self.enter('capture')
        self.wait_for_services(0)

    # ------------------------------------------------------
    # 2b. Capture
    # ------------------------------------------------------

    def build_capture(self, page):

        self.page_title(page, '2. XY / yaw 보정 — 캡처',
                        '표에서 점을 고르고, 원기둥을 그 좌표에 수직으로 세운 뒤, 사람은 시야 밖으로 나가서 아래 검출값이 '
                        '안정되면 [선택한 점 캡처]. 이미 찍은 점을 고르면 다시 찍어 덮어쓴다. 2점부터 잔차가, '
                        '4점부터 "빼면 RMS" 가 나온다 — 한 점만 빼서 RMS 가 크게 줄면 그 점을 다시 찍는다.')
        self.capture_box = self.section(page, '기준점')
        self.capture_tree = None
        self.live_box = self.section(page, '실시간 검출')
        self.live_labels = {}
        row = tk.Frame(page)
        row.pack(pady=6)
        self.capture_buttons = []
        for text, command in (('선택한 점 캡처', self.capture), ('마지막 점 삭제', self.undo),
                              ('전체 초기화', self.reset), ('계산 · 저장', self.solve)):
            button = tk.Button(row, text=text, width=16, height=2, command=command)
            button.pack(side='left', padx=5)
            self.capture_buttons.append(button)
        tk.Button(row, text='← 설정', width=10, height=2,
                  command=lambda: self.enter('setup')).pack(side='left', padx=(20, 5))
        tk.Button(row, text='← 메뉴', width=10, height=2,
                  command=lambda: self.enter('home')).pack(side='left', padx=5)
        self.result_box = self.section(page, '계산 결과 · 판정')
        self.result_labels = {}
        self.verdict_label = None
        self.hint_label = tk.Label(page, text='', fg=RED, font=('Arial', 11), wraplength=1040)
        self.hint_label.pack(fill='x')

    def fill_capture_page(self):
        if self.capture_tree is not None:
            self.capture_tree.destroy()
        columns = [('#', 40), ('필드 (x, y)', 110)]
        for sensor in self.sensors:
            n = sensor['name']
            columns += [(f'{n} 측정 (x, y)', 150), (f'{n} σ (cm)', 80),
                        (f'{n} 잔차 (cm)', 90), (f'{n} 빼면 RMS', 100)]
        self.capture_tree = self.tree(self.capture_box, columns, min(12, len(self.setup['grid_points'])))
        for i, (gx, gy) in enumerate(self.setup['grid_points']):
            self.capture_tree.insert('', 'end', iid=str(i), values=[i, f'({gx:g}, {gy:g})'])
        self.capture_tree.selection_set('0')
        for widget in list(self.live_box.winfo_children()) + list(self.result_box.winfo_children()):
            widget.destroy()
        self.live_labels, self.result_labels = {}, {}
        for sensor in self.sensors:
            self.live_labels[sensor['name']] = tk.Label(self.live_box, text='', font=('Courier', 11),
                                                        anchor='w')
            self.live_labels[sensor['name']].pack(fill='x', padx=8)
            self.result_labels[sensor['name']] = tk.Label(self.result_box, text='계산 전',
                                                          font=('Courier', 11), anchor='w')
            self.result_labels[sensor['name']].pack(fill='x', padx=8)
        self.verdict_label = tk.Label(self.result_box, text='', font=('Courier', 11, 'bold'),
                                      anchor='w', justify='left')
        self.verdict_label.pack(fill='x', padx=8, pady=(4, 2))
        # spread and distance check of each capture, kept in the session so a
        # resumed session still shows them
        notes = load_yaml(self.session / NOTES_FILE, {}) or {}
        self.capture_notes = {s['name']: {int(k): v for k, v in (notes.get(s['name']) or {}).items()}
                              for s in self.sensors}
        self.std_warning = {s['name']: '' for s in self.sensors}
        self.hint_label.config(text='')

    def wait_for_services(self, tries):
        if self.current_page != 'capture' or not self.setup:
            return
        if all(self.srv_clients[s['name']]['status'].service_is_ready() for s in self.sensors):
            self.log('검출·보정 노드 준비됨' + (' — 이 세션의 이전 캡처를 이어서 씁니다'
                                         if any(read_session(self.session).captures.values()) else ''))
            self.refresh_status()
            return
        if tries > 100:
            self.log('보정 노드가 20초 안에 뜨지 않음 — logs/calib_*.log 확인')
            return
        self.root.after(200, lambda: self.wait_for_services(tries + 1))

    def call(self, name, service, kind, request=None, then=None):
        client = self.srv_clients[name][service]
        if not client.service_is_ready():
            self.log(f'{name}: {service} 서비스 없음 (보정 노드가 떠 있는지 확인)')
            return False
        self.busy += 1
        self.set_capture_buttons()
        future = client.call_async(request or Trigger.Request())
        future.add_done_callback(lambda f: self.events.put((kind, name, f.result(), then)))
        return True

    def selected_index(self):
        selection = self.capture_tree.selection() if self.capture_tree else ()
        return int(selection[0]) if selection else None

    def captured_count(self, name):
        return self.status.get(name, {}).get('count', 0)

    def capture(self):
        index = self.selected_index()
        if index is None:
            return
        overwrite = [s['name'] for s in self.sensors if index < self.captured_count(s['name'])]
        if overwrite and not messagebox.askyesno(
                '다시 찍기', f'{index}번 점은 이미 찍혀 있습니다 ({", ".join(overwrite)}). 덮어쓸까요?'):
            return
        for sensor in self.sensors:
            name, count = sensor['name'], self.captured_count(sensor['name'])
            if index > count:
                self.log(f'{name}: {index}번을 찍으려면 {count}번부터 순서대로 찍어야 합니다')
                continue
            if index < count:
                request = SetParameters.Request()
                request.parameters = [Parameter('capture_index', Parameter.Type.INTEGER,
                                                index).to_parameter_msg()]
                self.call(name, 'set_parameters', 'param', request,
                          then=lambda n=name: self.call(n, 'capture_point', 'capture'))
            else:
                self.call(name, 'capture_point', 'capture')

    def undo(self):
        for sensor in self.sensors:
            if self.captured_count(sensor['name']):
                self.call(sensor['name'], 'undo_point', 'undo')

    def reset(self):
        if messagebox.askyesno('전체 초기화', '이 세션의 캡처를 모두 지울까요?\n(노드가 .bak 으로 백업)'):
            for sensor in self.sensors:
                self.call(sensor['name'], 'reset', 'reset')

    def solve(self):
        for sensor in self.sensors:
            if self.captured_count(sensor['name']) >= 2:
                self.call(sensor['name'], 'run_calibration', 'solve')

    def refresh_status(self):
        for sensor in self.sensors:
            self.call(sensor['name'], 'status', 'status')

    def on_service(self, kind, name, response, then):
        self.busy = max(0, self.busy - 1)
        self.set_capture_buttons()
        if kind == 'param':
            if then:
                then()
            return
        message = getattr(response, 'message', '') or ''
        first = message.strip().splitlines()[0] if message.strip() else ''
        if not getattr(response, 'success', False):
            self.log(f'{name} {kind} 실패: {first}')
            return
        if kind == 'status':
            self.status[name] = parse_status(message)
            self.update_capture_table()
            return
        self.log(f'{name}: {first}')
        if kind == 'capture':
            std = CAPTURE_STD_RE.search(message)
            index = CAPTURED_INDEX_RE.search(message)
            xy = re.search(r'sensor \(([-\d.]+), ([-\d.]+),', message)
            if std and index and xy:
                spread, i = float(std.group(1)), int(index.group(1)) - 1
                derr = range_error((float(xy.group(1)), float(xy.group(2))),
                                   self.setup['grid_points'][i], self.setup['nominal'][name])
                self.capture_notes[name][i] = {'std': spread, 'range_error': round(derr, 3)}
                save_yaml(self.session / NOTES_FILE, self.capture_notes)
                warnings = []
                if spread > CAPTURE_STD_WARN_M:
                    warnings.append(f'흔들림 {spread * 100:.1f} cm — 다른 물체와 섞였을 수 있음')
                if abs(derr) > RANGE_WARN_M:
                    warnings.append(f'센서~물체 거리가 예상보다 {derr * 100:+.0f} cm — 다른 물체를 잡았을 가능성 큼 '
                                    '(실시간 검출 좌표가 예상과 맞는지 보고, 검출 범위를 좁혀 다시 찍기)')
                self.std_warning[name] = (f'{name} {i}번: ' + ' / '.join(warnings)) if warnings else ''
        if kind in ('undo', 'reset'):
            self.std_warning[name] = ''
        if kind == 'solve':
            self.solutions[name] = parse_solution(message)
            self.update_capture_table()
            self.refresh_steps()
        if kind in ('capture', 'undo', 'reset', 'solve'):
            self.call(name, 'status', 'status')

    def update_capture_table(self):
        if not self.capture_tree:
            return
        counts = [self.captured_count(s['name']) for s in self.sensors]
        hints = []
        for i, (gx, gy) in enumerate(self.setup['grid_points']):
            values = [i, f'({gx:g}, {gy:g})']
            tags = ()
            for sensor in self.sensors:
                st = self.status.get(sensor['name'], {})
                row = st.get('rows', {}).get(i)
                if row is None:
                    values += ['-', '', '', '']
                    continue
                res = row['residual']
                loo = row['loo']
                note = self.capture_notes[sensor['name']].get(i) if i < self.captured_count(sensor['name']) else None
                std = note['std'] if note else None
                derr = note.get('range_error') if note else None
                values += [f"({row['x']:+.3f}, {row['y']:+.3f})" + (' ✗거리' if derr is not None and abs(derr) > RANGE_WARN_M else ''),
                           f'{std * 100:.1f}' if std is not None else '',
                           f'{res * 100:.1f}' if res is not None else '',
                           f'{loo * 100:.1f} cm' if loo is not None else '']
                if st.get('drop') == i or (std is not None and std > CAPTURE_STD_WARN_M) \
                        or (derr is not None and abs(derr) > RANGE_WARN_M):
                    tags = ('bad',)
                if st.get('drop') == i:
                    hints.append(f"{sensor['name']}: {i}번 점을 빼면 RMS 가 크게 줄어듭니다 — 그 점을 다시 찍으세요")
            if i == min(counts) and not tags:
                tags = ('next',)
            self.capture_tree.item(str(i), values=values, tags=tags)
        nxt = min(counts)
        if nxt < len(self.setup['grid_points']) and self.selected_index() is not None \
                and self.selected_index() < nxt:
            self.capture_tree.selection_set(str(nxt))
        for sensor in self.sensors:
            name = sensor['name']
            sol = self.solutions.get(name)
            preview = self.status.get(name, {}).get('preview')
            if sol:
                good = sol['rms'] <= RMS_GOOD_M
                self.result_labels[name].config(
                    text=(f"{name}: 저장됨  x {sol['x']:.3f} m  y {sol['y']:.3f} m  yaw {sol['yaw_deg']:.2f}°  "
                          f"RMS {sol['rms'] * 1000:.1f} mm  {'좋음' if good else '큼 — 잔차 큰 점 다시 찍기'}"),
                    fg=GREEN if good else RED)
            elif preview:
                self.result_labels[name].config(
                    text=(f"{name}: 미리보기  x {preview['x']:.3f}  y {preview['y']:.3f}  "
                          f"yaw {preview['yaw_deg']:.2f}°  RMS {preview['rms'] * 1000:.1f} mm  "
                          '(아직 저장 안 됨 — [계산 · 저장])'), fg='black')
        hints = [w for w in self.std_warning.values() if w] + hints
        self.hint_label.config(text='\n'.join(hints))
        if self.verdict_label is not None:
            problems = {}
            for sensor in self.sensors:
                name, found = sensor['name'], []
                count = self.captured_count(name)
                notes = [n for i, n in self.capture_notes[name].items() if i < count]
                if any(abs(n.get('range_error') or 0) > RANGE_WARN_M for n in notes):
                    found.append('거리 안 맞는 캡처')
                if any(n['std'] > CAPTURE_STD_WARN_M for n in notes):
                    found.append('흔들린 캡처')
                if self.status.get(name, {}).get('drop') is not None:
                    found.append(f"{self.status[name]['drop']}번 점 튐")
                if count < len(self.setup['grid_points']):
                    found.append(f'{count}/{len(self.setup["grid_points"])}점만 찍음')
                problems[name] = found
            verdict = calibration_verdict(self.solutions, self.setup['nominal'], problems,
                                          [s['name'] for s in self.sensors])
            head = '판정: 합격 — 3. YAML 적용으로' if verdict['ok'] else '판정: 아직 (아래 확인)'
            self.verdict_label.config(text='\n'.join([head] + verdict['lines']),
                                      fg=GREEN if verdict['ok'] else RED)

    def set_capture_buttons(self):
        state = tk.DISABLED if self.busy else tk.NORMAL
        for button in self.capture_buttons:
            button.config(state=state)

    def refresh_live(self):
        if self.current_page != 'capture' or not self.setup:
            return
        index = self.selected_index()
        now = self.now()
        for sensor in self.sensors:
            name = sensor['name']
            recent = [d for d in self.detections[name] if now - d[0] < 2.0]
            expected = ''
            if index is not None:
                nx, ny, nyaw = self.setup['nominal'][name]
                ex, ey = grid_to_sensor(self.setup['grid_points'][index], nx, ny, math.radians(nyaw))
                er = math.hypot(self.setup['grid_points'][index][0] - nx, self.setup['grid_points'][index][1] - ny)
                expected = f'  | {index}번 예상 ({ex:+.2f}, {ey:+.2f}) 거리 {er:.2f} m'
            if not recent:
                text, color = f'{name}: 검출 없음 — 원기둥이 검출 범위 밖이거나 다른 물체와 붙어 있음{expected}', RED
            else:
                xy = np.array([[d[1], d[2]] for d in recent])
                spread = float(np.max(np.linalg.norm(xy - xy.mean(0), axis=1)))
                res = self.residual.get(name)
                rng = float(np.hypot(xy[-1, 0], xy[-1, 1]))
                text = (f'{name}: ({xy[-1, 0]:+.3f}, {xy[-1, 1]:+.3f}) 거리 {rng:.2f} m  흔들림 {spread * 100:.1f} cm  '
                        f"원 맞춤 잔차 {f'{res} mm' if res is not None else '-'}{expected}")
                far = index is not None and abs(rng - er) > RANGE_WARN_M
                if far:
                    text += '  ← 거리 안 맞음'
                color = BLUE if spread < CAPTURE_STD_WARN_M and not far else RED
            self.live_labels[name].config(text=text, fg=color)

    # ------------------------------------------------------
    # 3. YAML
    # ------------------------------------------------------

    def build_apply(self, page):

        self.page_title(page, '3. YAML 적용 — 트래커 파라미터',
                        '이 세션의 결과(x / y / yaw / RMS, 1단계의 z / roll / pitch)를 트래커 YAML 의 lidarN_* 값에 '
                        '써 넣는다. 주석과 다른 줄은 그대로이고, 원래 파일은 옆에 .<시각>.bak 으로, 세션 폴더에도 '
                        '사본으로 남는다. 트래커를 다시 시작해야 반영된다.')
        box = self.section(page, '대상 YAML')
        row = tk.Frame(box)
        row.pack(fill='x', padx=8, pady=4)
        self.yaml_var = tk.StringVar(value=self.default_tracker_yaml)
        tk.Entry(row, textvariable=self.yaml_var, width=110).pack(side='left')
        tk.Button(row, text='찾아보기', command=self.browse_yaml).pack(side='left', padx=6)
        tk.Button(row, text='미리보기 새로고침', command=self.preview_apply).pack(side='left')
        box = self.section(page, '바뀔 값')
        self.apply_tree = self.tree(box, [('키', 260), ('현재 값', 200), ('새 값', 200)], 14)
        self.apply_info = tk.Label(page, text='', font=('Arial', 11), justify='left', wraplength=1040)
        self.apply_info.pack(fill='x', pady=4)
        row = tk.Frame(page)
        row.pack(pady=6)
        tk.Button(row, text='적용', width=18, height=2, command=self.apply_yaml).pack(side='left', padx=6)
        tk.Button(row, text='← 메뉴', width=12, height=2,
                  command=lambda: self.enter('home')).pack(side='left', padx=6)
        self.pending_apply = None

    def browse_yaml(self):
        path = filedialog.askopenfilename(initialfile=self.yaml_var.get(),
                                          filetypes=[('YAML', '*.yaml'), ('all', '*')])
        if path:
            self.yaml_var.set(path)
            self.preview_apply()

    def preview_apply(self):
        self.apply_tree.delete(*self.apply_tree.get_children())
        self.pending_apply = None
        path = os.path.expanduser(self.yaml_var.get())
        try:
            transforms = load_sensor_transforms(str(self.session / RESULT_FILE))
            original = Path(path).read_text(encoding='utf-8')
        except Exception as exc:
            self.apply_info.config(text=f'읽기 실패: {exc}', fg=RED)
            return
        names = {s['name'] for s in self.sensors}
        transforms = {n: tf for n, tf in transforms.items() if n in names}
        values = tracker_values(transforms)
        for sensor in self.sensors:     # result files written before tilt was stored
            g = self.ground.get(sensor['name'], {})
            for key in ('z', 'roll', 'pitch'):
                if key in g:
                    values.setdefault(f"{sensor['frame']}_{key}", g[key])
        new_text, changes, missing = apply_to_yaml_text(original, values)
        for key, old, new in changes:
            self.apply_tree.insert('', 'end', values=(key, old, new),
                                   tags=() if old == new else ('next',))
        notes = [f'결과: {self.session / RESULT_FILE}']
        for name, tf in transforms.items():
            rms = (tf.calibration or {}).get('rms_error')
            if rms is not None:
                notes.append(f'{name} RMS {rms * 1000:.1f} mm' + ('' if rms <= RMS_GOOD_M else '  ← 큼'))
        if missing:
            notes.append('YAML 에 없는 키 (쓰지 않음): ' + ', '.join(missing))
        absent = sorted(names - set(transforms))
        if absent:
            notes.append(f"결과 파일에 {', '.join(absent)} 의 x / y / yaw 가 없습니다 — "
                         '2단계에서 [계산 · 저장] 을 다시 하세요. 이대로 적용하면 그 센서는 위치·yaw 가 바뀌지 않습니다.')
        self.apply_info.config(text='\n'.join(notes), fg=RED if missing or absent else 'black')
        self.pending_apply = (path, original, new_text, changes)

    def apply_yaml(self):
        if not self.pending_apply:
            self.preview_apply()
            if not self.pending_apply:
                return
        path, original, new_text, changes = self.pending_apply
        if Path(path).read_text(encoding='utf-8') != original:
            messagebox.showwarning('파일이 바뀜', 'YAML 이 미리보기 이후 바뀌었습니다. 다시 확인하세요.')
            self.preview_apply()
            return
        if not messagebox.askyesno('YAML 적용', f'{path}\n\n{len(changes)}개 값을 바꿀까요?'):
            return
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        backup = f'{path}.{stamp}.bak'
        shutil.copy2(path, backup)
        session_copy = self.session / f'before_apply_{Path(path).name}'
        shutil.copy2(path, session_copy)
        Path(path).write_text(new_text, encoding='utf-8')
        save_yaml(self.session / APPLY_FILE, {
            'time': datetime.now().isoformat(timespec='seconds'), 'tracker_yaml': path,
            'backup': backup, 'session_copy': str(session_copy),
            'changes': [{'key': k, 'old': o, 'new': n} for k, o, n in changes]})
        self.log(f'3. YAML 적용: {len(changes)}개 값 → {path} (백업 {backup})')
        self.apply_info.config(text=f'적용 완료. 트래커를 다시 시작하면 반영됩니다.\n백업: {backup}', fg=GREEN)
        self.refresh_steps()


    # ==========================================================
    # Sessions and navigation
    # ==========================================================

    def new_session(self):
        chosen = [n for n, v in self.sensor_vars.items() if v.get()]
        if not chosen:
            messagebox.showerror('센서 없음', '사용할 센서를 하나 이상 고르세요.')
            return
        name = new_session_name(self.site_root, date.today(), self.memo_var.get())
        path = create_session(self.site_root, name, {
            'site_name': self.site.get('site_name', ''), 'site_file': self.site_file,
            'sensors': chosen})
        self.open_session(path)
        self.log(f'새 세션: {path}')

    def open_selected_session(self):
        selection = self.session_tree.selection()
        if selection:
            self.open_session(Path(selection[0]))

    def open_session(self, path):
        holder = acquire_lock(path)
        if holder:
            messagebox.showerror('사용 중', f'이 세션은 다른 GUI 가 열고 있습니다: {holder}')
            return
        self.close_session()
        meta = load_yaml(path / SESSION_FILE, {})
        missing = [n for n in meta.get('sensors', []) if n not in self.site_sensors]
        if missing:
            release_lock(path)
            messagebox.showerror('센서 불일치', f'현장 설정에 없는 센서: {missing}')
            return
        self.session = path
        self.sensors = [self.site_sensors[n] for n in meta.get('sensors', [])]
        self.ground = dict((load_yaml(path / GROUND_FILE, {}) or {}).get('results') or {})
        self.setup = None
        self.status, self.solutions = {}, {}
        info = read_session(path)
        self.session_var.set(f'세션: {info.label}   |   센서 {", ".join(meta.get("sensors", []))}   |   {path}')
        for button in self.step_buttons[:1]:
            button.config(state=tk.NORMAL)
        self.refresh_steps()
        self.refresh_sessions()

    def close_session(self):
        self.stop_all()
        if self.session:
            release_lock(self.session)
        self.session = None

    def enter(self, page):
        if page in ('home', 'ground', 'setup') and self.current_page == 'capture':
            self.stop_capture_nodes()
            self.log('검출·보정 노드 정리 (캡처는 세션 폴더에 저장되어 있어 다시 들어오면 이어집니다)')
        if page == 'home':
            self.refresh_sessions()
            self.refresh_steps()
            self.update_preview()
        elif page == 'ground':
            self.fill_ground_page()
        elif page == 'setup':
            self.fill_setup_page()
        elif page == 'capture':
            self.fill_capture_page()
        elif page == 'apply':
            self.preview_apply()
        self.show(page)


    # ==========================================================
    # Processes
    # ==========================================================

    @staticmethod
    def child_env():
        return dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_LOGGING_BUFFERED_STREAM='0')

    @staticmethod
    def ros2_run(executable, params, node_name=None):
        command = ['ros2', 'run', PACKAGE, executable, '--ros-args']
        if node_name:
            CalibrationGUI.started_names.add(node_name)
            command += ['-r', f'__node:={node_name}']
        for key, value in params.items():
            if isinstance(value, bool):
                value = 'true' if value else 'false'
            elif isinstance(value, (list, tuple)):
                value = '[' + ','.join(f'{float(v):.4f}' for v in value) + ']'
            command += ['-p', f'{key}:={value}']
        return command

    def free_node_names(self, names):
        """Refuse to start when a node of the same name already runs elsewhere."""
        running = {n for n, _ in self.get_node_names_and_namespaces()}
        clash = sorted((set(names) & running) - self.started_names)
        if clash:
            messagebox.showerror(
                '같은 이름의 노드가 이미 실행 중',
                '다른 터미널에서 띄운 것으로 보입니다. 먼저 끄세요 (Ctrl+C):\n\n' + '\n'.join(clash))
            return False
        return True

    def spawn(self, label, command, read_residual_for=None):
        self.stop_process(label)
        log_path = self.session / 'logs' / f"{label}_{datetime.now():%H%M%S}.log"
        log_file = open(log_path, 'w')
        log_file.write(' '.join(command) + '\n\n')
        log_file.flush()
        process = subprocess.Popen(command, stdout=subprocess.PIPE if read_residual_for else log_file,
                                   stderr=subprocess.STDOUT, text=True, env=self.child_env(),
                                   start_new_session=True)
        self.processes[label] = process
        if read_residual_for:
            threading.Thread(target=self.read_target_output,
                             args=(process, read_residual_for, log_file), daemon=True).start()

    def read_target_output(self, process, name, log_file):
        with log_file:
            for line in process.stdout:
                log_file.write(line)
                if 'fit residual' in line and 'exceeds' not in line:
                    try:
                        self.residual[name] = int(line.rsplit('fit residual', 1)[1].split('mm')[0])
                    except ValueError:
                        pass
                elif 'no target' in line:
                    self.residual[name] = None

    def stop_process(self, label):
        process = self.processes.pop(label, None)
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass

    def stop_capture_nodes(self):
        for label in [l for l in self.processes if l.startswith(('target_', 'calib_'))]:
            self.stop_process(label)
        self.setup = None

    def stop_all(self):
        for label in list(self.processes):
            self.stop_process(label)


    # ==========================================================
    # Event loop
    # ==========================================================

    def log(self, text):
        line = f'{datetime.now():%H:%M:%S}  {text}'
        self.log_box.insert('end', line + '\n')
        self.log_box.see('end')
        if self.session:
            with open(self.session / 'logs' / 'gui.log', 'a', encoding='utf-8') as handle:
                handle.write(line + '\n')
        self.get_logger().info(text)

    def poll(self):
        try:
            while True:
                event = self.events.get_nowait()
                if event[0] == 'ground':
                    self.on_ground(*event[1:])
                else:
                    self.on_service(*event)
        except queue.Empty:
            pass
        self.refresh_live()
        self.root.after(200, self.poll)

    def on_gui_close(self):
        if self.processes and not messagebox.askyesno('종료', '실행 중인 검출·보정 노드를 끄고 종료할까요?'):
            return
        self.close_session()
        self.root.destroy()

    def run_gui(self):
        self.root.mainloop()


# ==============================================================
# Main
# ==============================================================

def main(args=None):

    rclpy.init(args=args)
    node = CalibrationGUI()

    # rclpy must spin while Tkinter owns the main thread.
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        node.run_gui()
    finally:
        node.close_session()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
