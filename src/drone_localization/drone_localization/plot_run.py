#!/usr/bin/env python3
"""런 하나를 평면 궤적과 비행 프로필로 확인한다.

GUI가 ~/competition_results/<팀>_<ID>/run_NN/ 에 남긴 CSV를 읽어서 평면 궤적과
고도/속도를 서로 다른 그림으로 그린다. 같은 폴더에 trajectory.png과
flight_profiles.png으로 저장한다. 인자 없이 부르면 가장 최근 런을 집는다.

    ros2 run drone_localization plot_run
    ros2 run drone_localization plot_run --list
    ros2 run drone_localization plot_run --run ~/competition_results/팀_ID/run_01
    ros2 run drone_localization plot_run --save-only

ROI 사각형과 LiDAR 위치는 params yaml에서 읽는다. 하드코딩하면 파라미터를
바꾼 뒤 그림이 조용히 거짓말을 하게 된다.

읽는 파일:
    drone_trajectory.csv    KF 출력. 이 그림의 본체

세 화면을 각각 어떻게 읽는지:

  평면 궤적   필드/검출 ROI와 실제 이동 경로를 비교한다.
  고도        실제 비행 범위를 확대해 수직 변화를 본다.
  속도        이동과 저속/호버 후보 구간을 본다.
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np

DEFAULT_RESULTS = os.path.expanduser('~/competition_results')
TRAJ = 'drone_trajectory.csv'
RAW_DETECTIONS = 'raw_detections.csv'


def find_runs(base):
    runs = glob.glob(os.path.join(base, '*', 'run_*'))
    runs = [r for r in runs if os.path.isfile(os.path.join(r, TRAJ))]
    return sorted(runs, key=os.path.getmtime, reverse=True)


def read_traj(path):
    with open(path) as f:
        rows = [r for r in csv.DictReader(f) if r.get('time_s')]
    if len(rows) < 2:
        return None
    col = lambda k: np.array([float(r[k]) for r in rows])  # noqa: E731
    d = {k: col(k) for k in
         ('time_s', 'x_m', 'y_m', 'z_m', 'vx_mps', 'vy_mps', 'vz_mps',
          'cov_x', 'cov_y', 'cov_z')}
    d['t'] = d['time_s'] - d['time_s'][0]
    return d


def coasting_mask(run_dir, data):
    """Return frames published without a measurement.

    The raw-detection and trajectory writers use the same sensor timestamp on
    measured frames.  That is exact and catches the first coast frames before
    covariance has grown past the legacy 0.01 heuristic.  A trajectory-only
    run remains supported through that legacy fallback.
    """
    raw_path = os.path.join(run_dir, RAW_DETECTIONS)
    if os.path.isfile(raw_path):
        with open(raw_path) as stream:
            measured = {
                int(round(float(row['time_s']) * 1e6))
                for row in csv.DictReader(stream)
                if row.get('time_s')
            }
        stamps = [int(round(value * 1e6)) for value in data['time_s']]
        return np.asarray([stamp not in measured for stamp in stamps], dtype=bool)
    return np.asarray(data['cov_x']) > 0.01


def infer_search_end_index(data, markers, detection_radius=1.0,
                           gap_tolerance=0.35):
    """Infer SEARCH end using the same first-visit rule as the scorer."""
    if not markers:
        return None
    first_visit_ends = []
    for _, marker_x, marker_y in markers:
        distance = np.hypot(data['x_m'] - marker_x, data['y_m'] - marker_y)
        indices = np.flatnonzero(distance <= detection_radius)
        if not len(indices):
            return None
        end = previous = int(indices[0])
        for index in indices[1:]:
            index = int(index)
            if (index == previous + 1 or
                    data['t'][index] - data['t'][previous] <= gap_tolerance):
                end = previous = index
            else:
                break
        first_visit_ends.append(end)
    return max(first_visit_ends)


def read_params(path):
    """ROI와 LiDAR 좌표만 뽑는다. 없으면 오버레이를 생략한다."""
    try:
        import yaml
        with open(path) as f:
            p = yaml.safe_load(f)['lidar_drone_tracker']['ros__parameters']
    except Exception:
        return None
    try:
        return {
            'roi': [(p['roi_x_min'], p['roi_x_max']),
                    (p['roi_y_min'], p['roi_y_max']),
                    (p['roi_z_min'], p['roi_z_max'])],
            'lidars': [(p[f'lidar{i}_x'], p[f'lidar{i}_y'])
                       for i in (1, 2) if f'lidar{i}_x' in p],
            'markers': [(i, p[f'marker{i}_x'], p[f'marker{i}_y'])
                        for i in range(1, 5)
                        if f'marker{i}_x' in p and f'marker{i}_y' in p],
        }
    except KeyError:
        return None


def default_params_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('drone_localization')
        return os.path.join(share, 'params', 'lidar_localization_os1.yaml')
    except Exception:
        return None


def plot(run_dir, params, save_only):
    import matplotlib
    if save_only:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.collections import LineCollection
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    cjk_font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    if os.path.isfile(cjk_font):
        font_manager.fontManager.addfont(cjk_font)
        plt.rcParams['font.family'] = font_manager.FontProperties(
            fname=cjk_font).get_name()

    def draw_aruco(ax, marker_id, cx, cy, size=0.78):
        """Draw a map symbol resembling an ArUco tag (not a decodable tag)."""
        patterns = {
            1: ('1001', '0110', '1100', '0011'),
            2: ('1010', '0011', '1101', '0100'),
            3: ('1100', '0101', '1011', '0010'),
            4: ('1000', '1110', '0011', '0101'),
        }
        bits = patterns.get(marker_id, patterns[1])
        cell = size / 6.0
        left, bottom = cx - size / 2.0, cy - size / 2.0
        for row in range(6):
            for col in range(6):
                black = (row in (0, 5) or col in (0, 5) or
                         bits[row - 1][col - 1] == '1')
                ax.add_patch(plt.Rectangle(
                    (left + col * cell, bottom + (5 - row) * cell),
                    cell, cell, fc='#111111' if black else '#ffffff',
                    ec='none', zorder=7))
        ax.add_patch(plt.Rectangle(
            (left, bottom), size, size, fc='none', ec='#111111',
            lw=1.0, zorder=7.1))
        ax.text(cx, cy + size / 2.0 + 0.14, f'M{marker_id}',
                color='#111827', fontsize=8, weight='bold',
                ha='center', va='bottom', zorder=9,
                bbox={'facecolor': 'white', 'edgecolor': 'none',
                      'alpha': 0.82, 'pad': 0.4})

    d = read_traj(os.path.join(run_dir, TRAJ))
    if d is None:
        print(f'샘플이 부족합니다: {run_dir}')
        return None

    t, x, y, z = d['t'], d['x_m'], d['y_m'], d['z_m']
    cov = d['cov_x']
    speed = np.hypot(d['vx_mps'], d['vy_mps'])
    gaps = np.diff(t)

    team = os.path.basename(os.path.dirname(run_dir))
    run = os.path.basename(run_dir)
    coast_mask = coasting_mask(run_dir, d)
    coasting = int(coast_mask.sum())

    fig = plt.figure(figsize=(13, 9))
    fig.suptitle(
        f'{team} / {run}   ·   {len(t)} samples · {t[-1]:.1f}s · '
        f'{1.0 / np.mean(gaps):.1f} Hz avg · coasting {coasting}/{len(t)}',
        fontsize=13)

    # --- 평면 궤적 -------------------------------------------------
    ax = fig.add_subplot(1, 1, 1)
    field_x0, field_x1 = 0.0, 24.0
    field_y0, field_y1 = 0.0, 15.0
    roi_x0, roi_x1 = field_x0, field_x1
    roi_y0, roi_y1 = field_y0, field_y1
    if params:
        (rx0, rx1), (ry0, ry1), _ = params['roi']
        roi_x0, roi_x1 = rx0, rx1
        roi_y0, roi_y1 = ry0, ry1
        ax.add_patch(plt.Rectangle(
            (rx0, ry0), rx1 - rx0, ry1 - ry0,
            fc='#fff3f3', ec='#d32f2f', ls='--', lw=1.8,
            label=f'ROI {rx1-rx0:g}×{ry1-ry0:g} m'))
        for i, (lx, ly) in enumerate(params['lidars'], start=1):
            ax.plot(lx, ly, '^', c='#111827', ms=10, zorder=8)
            ax.annotate(f'lidar{i}', (lx, ly), textcoords='offset points',
                        xytext=(8, 4), color='#111827', fontsize=8)

    ax.add_patch(plt.Rectangle(
        (field_x0, field_y0), field_x1 - field_x0, field_y1 - field_y0,
        fc='#eef6ff', ec='#111827', lw=2.0, zorder=1,
        label='FIELD 24×15 m'))
    for gx in np.arange(field_x0, field_x1 + 0.01, 3.0):
        ax.plot([gx, gx], [field_y0, field_y1], c='#c7cdd4', lw=0.8,
                zorder=2)
    for gy in np.arange(field_y0, field_y1 + 0.01, 3.0):
        ax.plot([field_x0, field_x1], [gy, gy], c='#c7cdd4', lw=0.8,
                zorder=2)
    if params:
        for marker_id, marker_x, marker_y in params.get('markers', []):
            draw_aruco(ax, marker_id, marker_x, marker_y)

    seg = np.stack([np.c_[x[:-1], y[:-1]], np.c_[x[1:], y[1:]]], axis=1)
    inside_field = (
        (x[1:] >= field_x0) & (x[1:] <= field_x1) &
        (y[1:] >= field_y0) & (y[1:] <= field_y1))
    inside_roi = (
        (x[1:] >= roi_x0) & (x[1:] <= roi_x1) &
        (y[1:] >= roi_y0) & (y[1:] <= roi_y1))
    colors = np.full(len(seg), '#1565c0', dtype=object)
    colors[~inside_roi] = '#d32f2f'
    lc = LineCollection(
        seg, colors=colors.tolist(), linewidths=2.1, zorder=5)
    ax.add_collection(lc)
    if coast_mask.any():
        coast_segments = coast_mask[:-1] | coast_mask[1:]
        coast_lc = LineCollection(
            seg[coast_segments], colors='#f97316', linewidths=1.15,
            zorder=8)
        ax.add_collection(coast_lc)
    ax.plot(x[0], y[0], 'o', c='#2fdf75', mec='#087a36', ms=9, zorder=9)
    ax.annotate('START', (x[0], y[0]), textcoords='offset points',
                xytext=(7, 6), color='#087a36', fontsize=8, weight='bold')
    ax.plot(x[-1], y[-1], 'o', c='#ff4d4f', mec='#c62828', ms=9, zorder=9)
    ax.annotate('END', (x[-1], y[-1]), textcoords='offset points',
                xytext=(7, 6), color='#c62828', fontsize=8, weight='bold')
    x_low = min(float(x.min()), roi_x0) - 0.6
    x_high = max(float(x.max()), roi_x1) + 0.6
    y_low = min(float(y.min()), roi_y0) - 0.6
    y_high = max(float(y.max()), roi_y1) + 0.6
    ax.set_xlim(x_low, x_high)
    ax.set_ylim(y_low, y_high)
    x_ticks = np.arange(0.0, 24.01, 3.0)
    y_ticks = np.arange(0.0, 15.01, 3.0)
    ax.set_xticks(x_ticks)
    ax.set_yticks(y_ticks)
    ax.set_xticklabels(['' if value == 0 else f'{value:.0f}'
                        for value in x_ticks])
    ax.set_yticklabels(['' if value == 0 else f'{value:.0f}'
                        for value in y_ticks])
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.grid(False)
    legend = [
        Patch(fc='#eef6ff', ec='#111827', label='경기장 24×15 m'),
        Patch(fc='#fff3f3', ec='#d32f2f', ls='--', label='검출 ROI'),
        Line2D([0], [0], c='#1565c0', lw=2, label='비행 궤적'),
        Line2D([0], [0], c='#d32f2f', lw=2, label='ROI 밖'),
        Line2D([0], [0], c='#f97316', lw=1.15, label='코스팅'),
        Line2D([0], [0], marker='s', mfc='white', mec='#111', ls='none',
               label='ArUco 마커'),
    ]
    ax.legend(handles=legend, loc='lower left', fontsize=7)

    fig.subplots_adjust(left=0.08, right=0.98, bottom=0.08, top=0.91)
    trajectory_out = os.path.join(run_dir, 'trajectory.png')
    fig.savefig(trajectory_out, dpi=130)

    # --- 고도/속도: 별도 페이지 ------------------------------------
    profiles_fig, (altitude_ax, speed_ax) = plt.subplots(
        1, 2, figsize=(15, 5.5), constrained_layout=True)
    profiles_fig.suptitle(
        f'{team} / {run}   ·   Altitude and horizontal speed', fontsize=13)

    ax = altitude_ax
    ax.fill_between(
        t, 0, 1, where=coast_mask, transform=ax.get_xaxis_transform(),
        color='#f59e0b', alpha=0.16, step='mid', label='coasting')
    ax.plot(t, z, '-', lw=1.8, color='#1565c0')
    z_median = float(np.median(z))
    ax.axhline(z_median, ls='--', c='#52606d', lw=1,
               label=f'median {z_median:.2f} m')
    z_pad = max(0.08, 0.18 * float(np.ptp(z)))
    ax.set_ylim(max(0.0, float(z.min()) - z_pad), float(z.max()) + z_pad)
    ax.set_xlabel('t (s)')
    ax.set_ylabel('Z (m)')
    ax.set_title(
        f'Altitude  ·  min {z.min():.2f} / median {z_median:.2f} / '
        f'max {z.max():.2f} m')
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=8)

    # --- 수평 속도 --------------------------------------------------
    ax = speed_ax
    ax.fill_between(
        t, 0, 1, where=coast_mask, transform=ax.get_xaxis_transform(),
        color='#f59e0b', alpha=0.16, step='mid', label='coasting')
    ax.plot(t, speed, '-', lw=1.7, color='#1565c0', label='horizontal speed')
    slow = speed <= 0.15
    if slow.any():
        ax.scatter(t[slow], speed[slow], s=10, c='#16a34a', zorder=4,
                   label='slow (≤0.15 m/s)')
    speed_median = float(np.median(speed))
    ax.axhline(0.15, ls='--', c='#16a34a', lw=1, alpha=0.8)
    ax.set_xlabel('t (s)')
    ax.set_ylabel('speed (m/s)')
    ax.set_ylim(bottom=0.0)
    ax.set_title(
        f'Horizontal speed  ·  median {speed_median:.2f} / '
        f'max {speed.max():.2f} m/s')
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=8)

    profiles_out = os.path.join(run_dir, 'flight_profiles.png')
    profiles_fig.savefig(profiles_out, dpi=130)

    print(f'런        : {team} / {run}')
    print(f'샘플      : {len(t)}개, {t[-1]:.1f}초, 평균 {1.0/np.mean(gaps):.1f} Hz')
    print(f'간격      : 중앙 {np.median(gaps)*1000:.0f} ms, 최대 {gaps.max()*1000:.0f} ms')
    print(f'범위      : x {x.min():.2f}~{x.max():.2f}  '
          f'y {y.min():.2f}~{y.max():.2f}  z {z.min():.2f}~{z.max():.2f}')
    print(f'속도      : 중앙 {np.median(speed):.2f} m/s, 최대 {speed.max():.2f} m/s')
    print(f'코스팅    : {coasting}/{len(t)} 샘플')
    print(f'저장      : {trajectory_out}')
    print(f'            {profiles_out}')

    if not save_only:
        plt.show()
    return trajectory_out, profiles_out


def main():
    ap = argparse.ArgumentParser(
        description='런 CSV를 궤적 그림으로 만든다.')
    ap.add_argument('--run', help='런 폴더 경로 (기본: 가장 최근 런)')
    ap.add_argument('--results-dir', default=DEFAULT_RESULTS,
                    help=f'런들이 모인 폴더 (기본: {DEFAULT_RESULTS})')
    ap.add_argument('--params', default=None,
                    help='ROI/LiDAR를 읽을 params yaml')
    ap.add_argument('--list', action='store_true', help='런 목록만 출력')
    ap.add_argument('--save-only', action='store_true',
                    help='창을 띄우지 않고 PNG만 저장')
    # ros2 run 이 항상 붙이는 --ros-args 이후는 이 툴과 무관하므로 버린다.
    args, _ = ap.parse_known_args(
        [a for a in sys.argv[1:] if a != '--ros-args'])

    runs = find_runs(args.results_dir)
    if not runs:
        print(f'{args.results_dir} 아래에 {TRAJ} 를 가진 런이 없습니다.')
        return 1

    if args.list:
        print(f'{len(runs)}개 (최근순):')
        for r in runs:
            n = sum(1 for _ in open(os.path.join(r, TRAJ))) - 1
            print(f'  {os.path.basename(os.path.dirname(r))}/'
                  f'{os.path.basename(r)}  ({n} samples)')
        return 0

    run_dir = os.path.expanduser(args.run) if args.run else runs[0]
    if not os.path.isfile(os.path.join(run_dir, TRAJ)):
        print(f'{TRAJ} 가 없습니다: {run_dir}')
        return 1

    params_path = args.params or default_params_path()
    params = read_params(params_path) if params_path else None
    if params is None:
        print('params yaml 을 못 읽어 ROI/LiDAR 오버레이를 생략합니다.')

    return 0 if plot(run_dir, params, args.save_only) else 1


if __name__ == '__main__':
    sys.exit(main())
