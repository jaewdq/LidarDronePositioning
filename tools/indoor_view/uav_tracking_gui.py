#!/usr/bin/env python3
"""UAV Tracking -- indoor run management (START RUN / FINISH RUN, CSV, replay).

ID and run number, fusion structure (PF / MF / SU, default MF), START RUN / FINISH RUN.
START launches the tracker with the chosen fusion_mode; FINISH stops it and writes the
run record. Every run gets its own folder::

    ~/drone_project/flight_logs/indoor_real/<ID>/run<NN>_<PF|MF|SU>_<YYYYmmdd_HHMMSS>/
        estimated_pose.csv      t_sec, x, y, z, vx, vy, vz (/drone/estimated_odometry: KF position, velocity)
        <tracker yaml>          copy of the parameters used
        run.yaml                ID, run, fusion, times, estimates, calibration values
        tracker.log
        경로재생.sh             double-click (Run as a Program) to replay this flight in RViz
        bag/, bag.log           only with "원본 점군 녹화" checked

Operation (default): MF, no rosbag -- a flight is a few hundred kB of CSV, and
[경로 재생] (path_replay.py) plays it back in RViz.
Validation: check "원본 점군 녹화" to keep the raw clouds of both LiDARs (about
2-3 GB per minute) so PF / MF / SU can be compared later on the same flight by
replay_bag and against motion capture.

    python3 ~/drone_project/LidarDronePositioning/tools/indoor_view/uav_tracking_gui.py

PTP and the LiDAR drivers must already run. [화면 열기] starts the light RViz
view (indoor_view.launch.py view:=track start_tracker:=false).
"""

import os
import shutil
import signal
import subprocess
import threading
from datetime import datetime
from pathlib import Path

import tkinter as tk
from tkinter import messagebox

import rclpy
import rclpy.executors
import yaml
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Empty

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
DEFAULT_PARAMS = REPO / 'src' / 'drone_localization' / 'params' / 'lidar_localization_os0_indoor.yaml'
RESULTS = Path.home() / 'drone_project' / 'flight_logs' / 'indoor_real'

# Paper names -> tracker fusion_mode
FUSION = {'PF': 'cloud', 'MF': 'position', 'SU': 'sequential'}
FUSION_TEXT = {'PF': 'PF  점군 융합', 'MF': 'MF  관측 융합', 'SU': 'SU  순차 갱신'}

BAG_TOPICS = ['/lidar1/points', '/lidar2/points', '/lidar1/metadata', '/lidar2/metadata',
              '/tf_static', '/drone/estimated_pose', '/drone/estimated_odometry',
              '/drone/lidar1_pose', '/drone/lidar2_pose']


def tracker_executable():
    """Same choice as indoor_view.launch.py: _cov when the sim repo is built too."""
    lib = Path(get_package_prefix('drone_localization')) / 'lib' / 'drone_localization'
    return 'lidar_drone_tracker_cov' if (lib / 'lidar_drone_tracker_cov').exists() \
        else 'lidar_drone_tracker'


def write_replay_launcher(folder):
    """경로재생.sh next to the CSV: opens path_replay.py on this folder with the same ROS
    workspaces as this GUI (COLCON_PREFIX_PATH), so it runs from the file manager too."""
    prefixes = [p for p in os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep) if p]
    lines = ['#!/usr/bin/env bash', '# 이 비행을 RViz 로 다시 본다 (UAV Tracking 이 만든 파일).',
             'source /opt/ros/humble/setup.bash']
    for prefix in reversed(prefixes):          # overlays last, as in the GUI's shell
        lines.append(f'[ -f "{prefix}/setup.bash" ] && source "{prefix}/setup.bash"')
    lines.append(f'exec python3 "{HERE / "path_replay.py"}" "$(dirname "$(readlink -f "$0")")"')
    path = Path(folder) / '경로재생.sh'
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    path.chmod(0o755)


class UAVTrackingGUI(Node):

    def __init__(self):

        super().__init__('uav_tracking_gui')

        self.declare_parameter('params_file', str(DEFAULT_PARAMS))
        self.declare_parameter('results_dir', str(RESULTS))
        self.params_file = Path(os.path.expanduser(self.get_parameter('params_file').value))
        self.results_dir = Path(os.path.expanduser(self.get_parameter('results_dir').value))
        self.results_dir.mkdir(parents=True, exist_ok=True)
        self.tracker_exe = tracker_executable()

        self.tracker_process = None
        self.bag_process = None
        self.view_process = None
        self.tracker_node_name = 'lidar_drone_tracker'

        self.active = False
        self.start_pending = False
        self.run_folder = None
        self.run_info = None
        self.pose_file = None
        self.pose_count = 0
        self.last_pose = None
        self.lock = threading.Lock()

        # 위치와 속도가 한 메시지에 같이 온다 (트래커 /drone/estimated_odometry)
        self.create_subscription(Odometry, '/drone/estimated_odometry', self.on_odometry, 50)
        # START RUN 때 화면 도구(indoor_view.py)에 이전 비행 궤적을 지우라고 알린다
        self.clear_pub = self.create_publisher(Empty, '/viz/clear', 10)

        self.build_gui()


    # ==========================================================
    # GUI construction
    # ==========================================================

    def build_gui(self):

        self.root = tk.Tk()
        self.root.title('UAV Tracking')
        self.root.geometry('640x660')

        tk.Label(self.root, text='UAV Tracking', font=('Arial', 20, 'bold')).pack(pady=(18, 4))
        tk.Label(self.root, text=f'트래커 설정: {self.params_file.name}   |   실행 파일: {self.tracker_exe}',
                 font=('Arial', 9), fg='#555').pack()

        form = tk.Frame(self.root)
        form.pack(pady=12)
        tk.Label(form, text='ID:', width=14, anchor='e').grid(row=0, column=0, padx=5, pady=8)
        self.id_entry = tk.Entry(form, width=24)
        self.id_entry.grid(row=0, column=1)
        tk.Label(form, text='Run Number:', width=14, anchor='e').grid(row=1, column=0, padx=5, pady=8)
        self.run_entry = tk.Entry(form, width=24)
        self.run_entry.insert(0, '1')
        self.run_entry.grid(row=1, column=1)

        tk.Label(self.root, text='융합 구조', font=('Arial', 12, 'bold')).pack(pady=(6, 2))
        fusion = tk.Frame(self.root)
        fusion.pack(pady=4)
        self.fusion_var = tk.StringVar(value='MF')
        self.fusion_buttons = []
        for col, key in enumerate(('PF', 'MF', 'SU')):
            button = tk.Radiobutton(fusion, text=FUSION_TEXT[key], value=key, variable=self.fusion_var,
                                    indicatoron=0, width=15, height=2, selectcolor='#bfe3c6')
            button.grid(row=0, column=col, padx=6)
            self.fusion_buttons.append(button)

        # 검증 실험(모캡 비교, PF/MF/SU 재생 비교)일 때만 켠다 — 1분에 2~3 GB
        self.bag_var = tk.BooleanVar(value=False)
        self.bag_check = tk.Checkbutton(self.root, text='원본 점군 녹화 (rosbag, 검증 실험용 · 1분에 2~3 GB)',
                                        variable=self.bag_var)
        self.bag_check.pack(pady=(8, 0))

        buttons = tk.Frame(self.root)
        buttons.pack(pady=18)
        self.start_button = tk.Button(buttons, text='START RUN', width=17, height=2,
                                      command=self.start_run)
        self.start_button.grid(row=0, column=0, padx=8)
        self.finish_button = tk.Button(buttons, text='FINISH RUN', width=17, height=2,
                                       state=tk.DISABLED, command=self.finish_run)
        self.finish_button.grid(row=0, column=1, padx=8)
        tk.Button(buttons, text='화면 열기 (RViz)', width=17, height=2,
                  command=self.open_view).grid(row=0, column=2, padx=8)
        self.replay_button = tk.Button(buttons, text='경로 재생', width=17, height=2,
                                       command=self.open_replay)
        self.replay_button.grid(row=1, column=0, columnspan=3, pady=(10, 0))

        tk.Label(self.root, text='System Status', font=('Arial', 13, 'bold')).pack(pady=(10, 4))
        self.status = tk.Label(self.root, text='ID 와 Run Number 를 넣고 융합 구조를 고른 뒤 START RUN',
                               font=('Arial', 12), justify='left', wraplength=580)
        self.status.pack(pady=6)

        self.root.protocol('WM_DELETE_WINDOW', self.on_gui_close)


    # ==========================================================
    # Utility
    # ==========================================================

    @staticmethod
    def safe_name(name):
        return ''.join(c if c.isalnum() or c in '-_' else '_' for c in name.strip())

    def set_inputs(self, state):
        for widget in [self.id_entry, self.run_entry, self.bag_check] + self.fusion_buttons:
            widget.config(state=state)

    def on_odometry(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.last_pose = (p.x, p.y, p.z)
            if self.pose_file is not None:
                self.pose_file.write(f'{stamp:.6f},{p.x:.4f},{p.y:.4f},{p.z:.4f},'
                                     f'{v.x:.4f},{v.y:.4f},{v.z:.4f}\n')
                self.pose_count += 1


    # ==========================================================
    # START RUN
    # ==========================================================

    def start_run(self):

        run_id = self.safe_name(self.id_entry.get())
        try:
            run_number = int(self.run_entry.get())
        except ValueError:
            messagebox.showerror('입력 오류', 'Run Number 는 숫자여야 합니다.')
            return
        if not run_id:
            messagebox.showerror('입력 오류', 'ID 를 넣으세요.')
            return
        fusion = self.fusion_var.get()
        if not self.params_file.is_file():
            messagebox.showerror('설정 없음', f'트래커 YAML 이 없습니다:\n{self.params_file}')
            return
        if self.tracker_node_name in self.get_node_names():
            messagebox.showerror('트래커 실행 중',
                                 '다른 곳에서 트래커가 이미 돌고 있습니다 (indoor_view.launch.py 등).\n'
                                 '먼저 끄세요. 화면만 필요하면 [화면 열기] 를 쓰면 됩니다.')
            return
        existing = sorted((self.results_dir / run_id).glob(f'run{run_number:02d}_*'))
        if existing and not messagebox.askyesno(
                '같은 Run Number', f'{run_id} run {run_number} 기록이 이미 있습니다:\n'
                f'{existing[-1].name}\n\n그래도 새로 기록할까요? (폴더는 따로 생깁니다)'):
            return

        stamp = datetime.now()
        self.run_folder = self.results_dir / run_id / f'run{run_number:02d}_{fusion}_{stamp:%Y%m%d_%H%M%S}'
        self.run_folder.mkdir(parents=True)
        shutil.copy2(self.params_file, self.run_folder / self.params_file.name)
        params = (yaml.safe_load(self.params_file.read_text()) or {}) \
            .get('lidar_drone_tracker', {}).get('ros__parameters', {})
        self.run_info = {
            'id': run_id, 'run': run_number, 'fusion': fusion, 'fusion_mode': FUSION[fusion],
            'start': stamp.isoformat(timespec='seconds'), 'params_file': str(self.params_file),
            'tracker_executable': self.tracker_exe, 'rosbag': bool(self.bag_var.get()),
            'extrinsics': {k: v for k, v in params.items()
                           if k.startswith(('lidar1_', 'lidar2_')) and not k.endswith('_topic')},
        }
        self.write_run_info()
        write_replay_launcher(self.run_folder)

        with self.lock:
            self.pose_file = open(self.run_folder / 'estimated_pose.csv', 'w')
            self.pose_file.write('t_sec,x,y,z,vx,vy,vz\n')
            self.pose_count = 0
            self.last_pose = None

        tracker_command = [
            'ros2', 'run', 'drone_localization', self.tracker_exe, '--ros-args',
            '-r', f'__node:={self.tracker_node_name}',
            '--params-file', str(self.params_file),
            '-p', f'fusion_mode:={FUSION[fusion]}',
            # 화면용 점군은 끄고 추적만 (부하 감소)
            '-p', 'publish_filtered_clouds:=false', '-p', 'publish_world_clouds:=false',
        ]
        self.get_logger().info('Launching tracker: ' + ' '.join(tracker_command))
        self.tracker_process = self.spawn(tracker_command, 'tracker.log')
        if self.run_info['rosbag']:
            bag_command = ['ros2', 'bag', 'record', '-o', str(self.run_folder / 'bag')] + BAG_TOPICS
            self.bag_process = self.spawn(bag_command, 'bag.log')

        self.clear_pub.publish(Empty())
        self.start_pending = True
        self.set_inputs(tk.DISABLED)
        self.start_button.config(state=tk.DISABLED)
        self.status.config(text='STARTING...')
        self.start_wait = 0
        self.root.after(300, self.check_nodes_ready)

    def spawn(self, command, log_name):
        log = open(self.run_folder / log_name, 'w')
        log.write(' '.join(command) + '\n\n')
        log.flush()
        return subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)

    def check_nodes_ready(self):

        if not self.start_pending:
            return
        checks = [(self.tracker_process, '트래커')]
        if self.run_info['rosbag']:
            checks.append((self.bag_process, '녹화'))
        for process, name in checks:
            if process is None or process.poll() is not None:
                self.start_failed(f'{name} 프로세스가 멈췄습니다. {self.run_folder}/ 의 로그를 확인하세요.')
                return
        tracker_ready = self.tracker_node_name in self.get_node_names()
        if tracker_ready:
            self.start_pending = False
            self.active = True
            self.run_started = datetime.now()
            self.finish_button.config(state=tk.NORMAL)
            self.update_running()
            return
        self.start_wait += 1
        if self.start_wait > 100:
            self.start_failed('트래커가 30초 안에 뜨지 않았습니다 (tracker.log 확인).')
            return
        self.status.config(text='STARTING...\n\n트래커: starting' + ('\n녹화: 실행 중' if self.run_info['rosbag'] else ''))
        self.root.after(300, self.check_nodes_ready)

    def start_failed(self, reason):
        self.start_pending = False
        self.stop_run_processes()
        self.close_pose_file()
        self.run_info['end'] = datetime.now().isoformat(timespec='seconds')
        self.run_info['result'] = f'시작 실패: {reason}'
        self.write_run_info()
        self.set_inputs(tk.NORMAL)
        self.start_button.config(state=tk.NORMAL)
        self.status.config(text=f'시작 실패\n\n{reason}')

    def update_running(self):
        if not self.active:
            return
        elapsed = (datetime.now() - self.run_started).total_seconds()
        with self.lock:
            count, pose = self.pose_count, self.last_pose
        where = f'({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.2f})' if pose else '아직 없음'
        self.status.config(text=(
            f"RUNNING — {self.run_info['id']} run {self.run_info['run']}  [{self.run_info['fusion']}]"
            f"{'  + rosbag' if self.run_info['rosbag'] else ''}\n\n"
            f'경과 {elapsed:5.1f} s   추정 {count} 개   현재 위치 {where}\n'
            f'기록: {self.run_folder}'))
        self.root.after(200, self.update_running)


    # ==========================================================
    # FINISH RUN
    # ==========================================================

    def finish_run(self):

        if not self.active:
            return
        self.active = False
        self.finish_button.config(state=tk.DISABLED)
        self.status.config(text='FINISHING... (녹화 마무리 중)')
        self.root.update()
        end = datetime.now()
        self.stop_run_processes()
        self.close_pose_file()
        self.run_info.update({
            'end': end.isoformat(timespec='seconds'),
            'duration_s': round((end - self.run_started).total_seconds(), 1),
            'estimated_poses': self.pose_count,
            'result': 'finished',
        })
        self.write_run_info()
        self.set_inputs(tk.NORMAL)
        self.start_button.config(state=tk.NORMAL)
        self.run_entry.delete(0, tk.END)
        self.run_entry.insert(0, str(self.run_info['run'] + 1))
        self.status.config(text=(
            f"FINISHED — {self.run_info['id']} run {self.run_info['run']} [{self.run_info['fusion']}]  "
            f"{self.run_info['duration_s']} s, 추정 {self.pose_count} 개\n\n저장: {self.run_folder}"))

    def write_run_info(self):
        with open(self.run_folder / 'run.yaml', 'w', encoding='utf-8') as handle:
            yaml.safe_dump(self.run_info, handle, allow_unicode=True, sort_keys=False)

    def close_pose_file(self):
        with self.lock:
            if self.pose_file is not None:
                self.pose_file.close()
                self.pose_file = None


    # ==========================================================
    # Processes
    # ==========================================================

    def open_view(self):
        if self.view_process is not None and self.view_process.poll() is None:
            return
        self.view_process = subprocess.Popen(
            ['ros2', 'launch', str(HERE / 'indoor_view.launch.py'),
             'start_tracker:=false', 'view:=track', f'params_file:={self.params_file}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

    def open_replay(self):
        # 방금 끝낸 run 이 있으면 그것을, 없으면 재생 도구에서 고른다
        target = []
        if not self.active and self.run_folder is not None and (self.run_folder / 'estimated_pose.csv').is_file():
            target = [str(self.run_folder)]
        subprocess.Popen(['python3', str(HERE / 'path_replay.py')] + target,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

    def stop_process(self, process, name, timeout=5.0):
        if process is None or process.poll() is not None:
            return
        try:
            self.get_logger().info(f'Stopping {name}...')
            os.killpg(os.getpgid(process.pid), signal.SIGINT)    # Ctrl+C
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass

    def stop_run_processes(self):
        # the bag first, so it closes cleanly while the tracker still publishes
        self.stop_process(self.bag_process, 'rosbag', timeout=10.0)
        self.stop_process(self.tracker_process, 'tracker')
        self.bag_process = None
        self.tracker_process = None

    def on_gui_close(self):
        if self.active and not messagebox.askyesno(
                'Run 진행 중', '기록 중인 run 이 있습니다.\n마무리(FINISH)하고 종료할까요?'):
            return
        if self.active:
            self.finish_run()
        self.stop_process(self.view_process, 'view')
        self.root.destroy()

    def run_gui(self):
        self.root.mainloop()


# ==============================================================
# Main
# ==============================================================

def main(args=None):

    rclpy.init(args=args)
    node = UAVTrackingGUI()

    # rclpy must spin while Tkinter owns the main thread. Own executor so it can be
    # stopped and joined before shutdown (otherwise the process aborts on exit).
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        node.run_gui()
    finally:
        node.stop_run_processes()
        node.close_pose_file()
        executor.shutdown()
        ros_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
