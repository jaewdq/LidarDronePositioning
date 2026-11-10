#!/usr/bin/env python3
"""경로 재생 -- 추적 결과 CSV(estimated_pose.csv)를 RViz 에 시간대로 다시 그린다.

rosbag 없이 운용할 때 비행을 다시 보는 도구다. UAV Tracking 의 [경로 재생] 버튼이
방금 끝낸 run 으로 띄우고, 따로 실행할 수도 있다::

    python3 ~/drone_project/LidarDronePositioning/tools/indoor_view/path_replay.py [run 폴더 | CSV]

인자가 없으면 run 폴더를 고르는 창이 뜬다. CSV 열은 t_sec, x, y, z[, vx, vy, vz] 이고
속도 열이 없는 예전 파일은 위치 차분으로 속도를 계산한다.

화면: indoor_view.launch.py view:=replay (필드, ROI, 축 + 아래 토픽). 필드·ROI 는 run 폴더에
복사된 트래커 YAML 을 쓴다 (없으면 기본 실내 YAML).
  /viz/replay_full   비행 전체 경로 (회색)
  /viz/replay_track  처음부터 지금 시각까지 (초록)
  /viz/replay_drone  지금 위치 점과 (x, y, z), 시각 글자
조작: 재생/일시정지, 처음으로, 배속 0.25~4x, 시간 슬라이더, 다른 파일 열기.
"""

import csv
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import tkinter as tk
from tkinter import filedialog, messagebox

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
DEFAULT_PARAMS = REPO / 'src' / 'drone_localization' / 'params' / 'lidar_localization_os0_indoor.yaml'
RESULTS = Path.home() / 'drone_project' / 'flight_logs' / 'indoor_real'
SPEEDS = ('0.25x', '0.5x', '1x', '2x', '4x')


def load_csv(path):
    """t, xyz (N x 3), vel (N x 3). Rows without a position are skipped."""
    rows = []
    with open(path, newline='') as handle:
        reader = csv.DictReader(handle)
        cols = reader.fieldnames or []
        tcol = 't_sec' if 't_sec' in cols else cols[0]
        has_vel = all(k in cols for k in ('vx', 'vy', 'vz'))
        for r in reader:
            try:
                p = [float(r[tcol]), float(r['x']), float(r['y']), float(r['z'])]
            except (TypeError, ValueError, KeyError):
                continue
            try:
                v = [float(r['vx']), float(r['vy']), float(r['vz'])] if has_vel else [np.nan] * 3
            except (TypeError, ValueError):
                v = [np.nan] * 3
            rows.append(p + v)
    if not rows:
        raise ValueError(f'위치가 들어 있는 행이 없습니다: {path}')
    a = np.array(rows)
    a = a[np.argsort(a[:, 0], kind='stable')]
    t, xyz, vel = a[:, 0], a[:, 1:4], a[:, 4:7]
    missing = np.isnan(vel).any(axis=1)
    if missing.any():                       # 예전 CSV: +-0.2 s 위치 차분
        lo = np.searchsorted(t, t - 0.2)
        hi = np.clip(np.searchsorted(t, t + 0.2, side='right') - 1, 0, len(t) - 1)
        dt = np.maximum(t[hi] - t[lo], 1e-3)
        diff = (xyz[hi] - xyz[lo]) / dt[:, None]
        vel[missing] = diff[missing]
    return t - t[0], xyz, vel, has_vel


def resolve(target):
    """run 폴더 또는 CSV -> (csv 경로, 트래커 yaml 경로)."""
    p = Path(target).expanduser()
    folder = p if p.is_dir() else p.parent
    csv_path = p if p.is_file() else p / 'estimated_pose.csv'
    if not csv_path.is_file():
        raise FileNotFoundError(f'estimated_pose.csv 가 없습니다: {folder}')
    yamls = [y for y in folder.glob('*.yaml') if y.name != 'run.yaml' and 'lidar_localization' in y.name]
    return csv_path, (yamls[0] if yamls else DEFAULT_PARAMS)


class Replay(Node):

    def __init__(self):
        super().__init__('path_replay')
        self.pub_full = self.create_publisher(PathMsg, '/viz/replay_full', 10)
        self.pub_track = self.create_publisher(PathMsg, '/viz/replay_track', 10)
        self.pub_drone = self.create_publisher(MarkerArray, '/viz/replay_drone', 10)
        self.full = PathMsg()
        self.full.header.frame_id = 'map'
        self.poses = []

    def set_flight(self, xyz):
        self.poses = []
        for x, y, z in xyz:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position = Point(x=float(x), y=float(y), z=float(z))
            ps.pose.orientation.w = 1.0
            self.poses.append(ps)
        self.full.poses = self.poses

    def publish_full(self, show):
        self.full.header.stamp = self.get_clock().now().to_msg()
        msg = self.full if show else PathMsg(header=self.full.header)
        self.pub_full.publish(msg)

    def publish_at(self, i, t, xyz, vel):
        stamp = self.get_clock().now().to_msg()
        track = PathMsg()
        track.header.frame_id = 'map'
        track.header.stamp = stamp
        track.poses = self.poses[:i + 1]
        self.pub_track.publish(track)
        x, y, z = (float(v) for v in xyz)
        dot = Marker()
        dot.header.frame_id = 'map'
        dot.header.stamp = stamp
        dot.ns, dot.id, dot.type = 'replay_dot', 0, Marker.SPHERE
        dot.pose.position = Point(x=x, y=y, z=z)
        dot.pose.orientation.w = 1.0
        dot.scale.x = dot.scale.y = dot.scale.z = 0.08
        dot.color.r, dot.color.g, dot.color.b, dot.color.a = 0.2, 1.0, 0.5, 1.0
        txt = Marker()
        txt.header = dot.header
        txt.ns, txt.id, txt.type = 'replay_text', 0, Marker.TEXT_VIEW_FACING
        txt.pose.position = Point(x=x, y=y, z=z + 0.3)
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.07
        txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
        # RViz 글자는 띄어쓰기를 아주 넓게 그린다 — 라이브 화면(indoor_view.py)처럼 띄어쓰기 없이 두 줄로
        txt.text = f'({x:.3f},{y:.3f},{z:.2f})\n{t:.1f}s'
        self.pub_drone.publish(MarkerArray(markers=[dot, txt]))


class ReplayGUI:

    def __init__(self, node, target):
        self.node = node
        self.view = None
        self.view_params = None
        self.t = np.zeros(1)
        self.playing = False
        self.now = 0.0
        self.index = -1
        self.last_wall = time.monotonic()
        self.slider_busy = False
        self.last_full = 0.0

        self.root = tk.Tk()
        self.root.title('경로 재생')
        self.root.geometry('680x330')
        self.file_label = tk.Label(self.root, text='', font=('Arial', 10), fg='#333', wraplength=640)
        self.file_label.pack(pady=(14, 2))
        self.info = tk.Label(self.root, text='', font=('Arial', 10), fg='#555')
        self.info.pack()

        self.slider = tk.Scale(self.root, from_=0, to=1, orient=tk.HORIZONTAL, resolution=0.1,
                               length=620, showvalue=False, command=self.on_slider)
        self.slider.pack(pady=(14, 0))
        self.clock = tk.Label(self.root, text='', font=('Consolas', 13))
        self.clock.pack(pady=4)

        bar = tk.Frame(self.root)
        bar.pack(pady=8)
        tk.Button(bar, text='처음으로', width=9, command=self.rewind).grid(row=0, column=0, padx=4)
        self.play_button = tk.Button(bar, text='▶ 재생', width=11, command=self.toggle)
        self.play_button.grid(row=0, column=1, padx=4)
        self.speed = tk.StringVar(value='1x')
        tk.OptionMenu(bar, self.speed, *SPEEDS).grid(row=0, column=2, padx=4)
        self.show_full = tk.BooleanVar(value=True)
        tk.Checkbutton(bar, text='전체 경로', variable=self.show_full,
                       command=lambda: self.node.publish_full(self.show_full.get())).grid(row=0, column=3, padx=4)
        tk.Button(bar, text='다른 파일 열기', width=12, command=self.choose).grid(row=0, column=4, padx=4)
        self.where = tk.Label(self.root, text='', font=('Consolas', 11))
        self.where.pack(pady=6)

        self.root.protocol('WM_DELETE_WINDOW', self.close)
        if target:
            self.load(target)
        else:
            self.root.after(100, self.choose)
        self.root.after(33, self.tick)

    # ------------------------------------------------------------------
    def choose(self):
        folder = filedialog.askdirectory(title='run 폴더 고르기 (estimated_pose.csv 가 든 폴더)',
                                         initialdir=str(RESULTS if RESULTS.is_dir() else Path.home()))
        if folder:
            self.load(folder)

    def load(self, target):
        try:
            csv_path, params = resolve(target)
            self.t, self.xyz, self.vel, has_vel = load_csv(csv_path)
        except (OSError, ValueError) as exc:
            messagebox.showerror('열기 실패', str(exc))
            return
        self.node.set_flight(self.xyz)
        dur = float(self.t[-1])
        rate = (len(self.t) - 1) / dur if dur > 0 else 0.0
        self.file_label.config(text=str(csv_path))
        self.info.config(text=f'{len(self.t)} 개 추정, {dur:.1f} s ({rate:.1f} Hz), 속도: '
                              f'{"트래커 KF" if has_vel else "위치 차분으로 계산"}, 화면 설정: {params.name}')
        self.slider.config(to=max(dur, 0.1))
        self.start_view(params)
        self.playing = False
        self.play_button.config(text='▶ 재생')
        self.seek(0.0)
        self.node.publish_full(self.show_full.get())

    def start_view(self, params):
        if self.view is not None and self.view.poll() is None and params == self.view_params:
            return
        self.stop_view()
        self.view_params = params
        self.view = subprocess.Popen(
            ['ros2', 'launch', str(HERE / 'indoor_view.launch.py'),
             'start_tracker:=false', 'view:=replay', f'params_file:={params}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

    def stop_view(self):
        if self.view is not None and self.view.poll() is None:
            try:
                os.killpg(os.getpgid(self.view.pid), signal.SIGINT)
                self.view.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
        self.view = None

    # ------------------------------------------------------------------
    def toggle(self):
        if len(self.t) < 2:
            return
        if not self.playing and self.now >= self.t[-1]:
            self.seek(0.0)
        self.playing = not self.playing
        self.last_wall = time.monotonic()
        self.play_button.config(text='⏸ 일시정지' if self.playing else '▶ 재생')

    def rewind(self):
        self.seek(0.0)

    def on_slider(self, value):
        if not self.slider_busy:
            self.seek(float(value))

    def seek(self, t):
        self.now = min(max(t, 0.0), float(self.t[-1]))
        self.index = -1
        self.draw()

    def draw(self):
        i = int(np.searchsorted(self.t, self.now, side='right') - 1)
        i = max(i, 0)
        if i != self.index:
            self.index = i
            self.node.publish_at(i, self.t[i], self.xyz[i], self.vel[i])
            x, y, z = self.xyz[i]
            self.where.config(text=f'x {x:6.3f}  y {y:6.3f}  z {z:5.2f} m   |v| {np.linalg.norm(self.vel[i]):4.2f} m/s')
        self.clock.config(text=f'{self.now:6.1f} / {self.t[-1]:.1f} s')
        self.slider_busy = True
        self.slider.set(self.now)
        self.slider_busy = False

    def tick(self):
        wall = time.monotonic()
        if self.playing:
            self.now += (wall - self.last_wall) * float(self.speed.get().rstrip('x'))
            if self.now >= self.t[-1]:
                self.now = float(self.t[-1])
                self.playing = False
                self.play_button.config(text='▶ 재생')
            self.draw()
        self.last_wall = wall
        if wall - self.last_full > 1.0 and len(self.node.poses):   # 늦게 뜬 RViz 도 전체 경로를 받게
            self.node.publish_full(self.show_full.get())
            if not self.playing and self.index >= 0:
                self.node.publish_at(self.index, self.t[self.index], self.xyz[self.index], self.vel[self.index])
            self.last_full = wall
        self.root.after(33, self.tick)

    def close(self):
        self.stop_view()
        self.root.destroy()


def main():
    rclpy.init()
    node = Replay()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    gui = ReplayGUI(node, sys.argv[1] if len(sys.argv) > 1 else None)
    try:
        gui.root.mainloop()
    finally:
        gui.stop_view()
        executor.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
