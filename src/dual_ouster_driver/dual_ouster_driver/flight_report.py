#!/usr/bin/env python3

"""비행 기록을 읽어 3D 인터랙티브 뷰와 수치 요약을 낸다.

flight_recorder 가 남긴 track.csv 를 읽는다. 라이다 추적만 있고 기준값은
없으므로 "정확도"는 낼 수 없다. 대신 궤적이 어땠는지, 어디서 놓쳤는지,
속도가 어땠는지를 본다.

출력:
  <run_dir>/flight_3d.html      브라우저에서 돌려보는 3D 궤적 (plotly)
  <run_dir>/flight_2d.png       평면도 + 고도-시간 (matplotlib)
  <run_dir>/report.txt          수치 요약
  콘솔에도 같은 요약

사용:
    ros2 run dual_ouster_driver flight_report --ros-args -p run_dir:=~/flights/flight_...
    python3 flight_report.py ~/flights/flight_...        (ROS 없이도 됨)
"""

import csv
import math
import os
import sys

import numpy as np


# ----------------------------------------------------------------------
def load(run_dir):
    path = os.path.join(run_dir, 'track.csv')
    if not os.path.isfile(path):
        raise FileNotFoundError(path)
    cols = {}
    with open(path) as f:
        r = csv.DictReader(f)
        names = r.fieldnames
        for n in names:
            cols[n] = []
        for row in r:
            for n in names:
                v = row[n]
                cols[n].append(float(v) if v not in ('', None) else float('nan'))
    return {k: np.asarray(v) for k, v in cols.items()}


def segments(t, det):
    """검출이 끊긴 구간을 (시작, 끝) 목록으로."""
    out, start = [], None
    for i, d in enumerate(det):
        if d < 0.5 and start is None:
            start = t[i]
        elif d >= 0.5 and start is not None:
            out.append((start, t[i]))
            start = None
    if start is not None:
        out.append((start, t[-1]))
    return out


def hover_spans(t, xyz, speed, v_thresh=0.15, min_sec=1.0):
    """느리게 머문 구간을 찾는다. 대회 채점이 호버 위치 기준이라 필요하다."""
    out, start = [], None
    for i in range(len(t)):
        slow = np.isfinite(speed[i]) and speed[i] <= v_thresh
        if slow and start is None:
            start = i
        elif not slow and start is not None:
            if t[i - 1] - t[start] >= min_sec:
                out.append((start, i - 1))
            start = None
    if start is not None and t[-1] - t[start] >= min_sec:
        out.append((start, len(t) - 1))
    res = []
    for a, b in out:
        seg = xyz[a:b + 1]
        seg = seg[np.isfinite(seg).all(axis=1)]
        if len(seg) < 3:
            continue
        res.append({
            't0': float(t[a]), 't1': float(t[b]),
            'dur': float(t[b] - t[a]),
            'pos': np.median(seg, axis=0),
            'spread': float(np.max(np.linalg.norm(seg - np.median(seg, axis=0), axis=1))),
            'n': len(seg),
        })
    return res


# ----------------------------------------------------------------------
def make_report(d, run_dir):
    t = d['t_sec']
    det = d['detected']
    raw = np.column_stack([d['raw_x'], d['raw_y'], d['raw_z']])
    kf = np.column_stack([d['kf_x'], d['kf_y'], d['kf_z']])
    speed = d['speed']

    ok = np.isfinite(raw).all(axis=1)
    kok = np.isfinite(kf).all(axis=1)
    dur = float(t[-1] - t[0]) if len(t) > 1 else 0.0
    gaps = segments(t, det)
    gap_dur = sum(b - a for a, b in gaps)

    L = []
    L.append('=' * 62)
    L.append('  비행 기록 요약')
    L.append('=' * 62)
    L.append('')
    L.append('[기본]')
    L.append(f'  기록 폴더        {run_dir}')
    L.append(f'  비행 시간        {dur:.1f} s')
    L.append(f'  프레임           {len(t)}  ({len(t)/dur:.1f} Hz)' if dur > 0
             else f'  프레임           {len(t)}')
    L.append(f'  검출             {int(ok.sum())} / {len(t)}  '
             f'({100*ok.mean():.1f} %)')

    L.append('')
    L.append('[놓친 구간]')
    if not gaps:
        L.append('  없음 — 끊김 없이 추적됨')
    else:
        L.append(f'  {len(gaps)}회,  합계 {gap_dur:.1f} s ({100*gap_dur/dur:.1f}%)')
        longest = max(gaps, key=lambda g: g[1] - g[0])
        L.append(f'  가장 긴 구간     {longest[0]:.1f} ~ {longest[1]:.1f} s '
                 f'({longest[1]-longest[0]:.2f} s)')
        for a, b in gaps[:10]:
            L.append(f'     {a:7.2f} ~ {b:7.2f} s   ({b-a:.2f} s)')
        if len(gaps) > 10:
            L.append(f'     ... 외 {len(gaps)-10}회')

    if ok.sum() >= 2:
        p = raw[ok]
        L.append('')
        L.append('[이동 범위]  라이다 원본 기준')
        for i, ax in enumerate('xyz'):
            L.append(f'  {ax}   {p[:,i].min():+7.2f} ~ {p[:,i].max():+7.2f} m'
                     f'   (폭 {np.ptp(p[:,i]):.2f})')
        path_len = float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))
        L.append(f'  이동 거리 합계   {path_len:.1f} m')

    if kok.sum() >= 2:
        sp = speed[np.isfinite(speed)]
        L.append('')
        L.append('[속도]  칼만필터 기준')
        L.append(f'  평균             {sp.mean():.2f} m/s')
        L.append(f'  중앙값           {np.median(sp):.2f} m/s')
        L.append(f'  최대             {sp.max():.2f} m/s')
        L.append(f'  95 백분위        {np.percentile(sp,95):.2f} m/s')

        # 원본과 KF 의 차이 = 필터가 얼마나 깎았는지
        both = ok & kok
        if both.sum() >= 5:
            diff = np.linalg.norm(raw[both] - kf[both], axis=1)
            L.append('')
            L.append('[원본 vs 칼만필터]')
            L.append(f'  차이 평균        {diff.mean()*1000:.0f} mm')
            L.append(f'  차이 최대        {diff.max()*1000:.0f} mm')
            L.append('  차이가 크면 관측이 튀었거나 필터가 과하게 눌렀다는 뜻입니다.')

    hv = hover_spans(t, kf if kok.sum() > ok.sum() else raw, speed)
    L.append('')
    L.append('[호버 구간]  속도 0.15 m/s 이하로 1초 이상')
    if not hv:
        L.append('  없음')
    else:
        L.append(f'  {len(hv)}개')
        L.append(f'     {"시각":>14}  {"길이":>6}  {"위치 (x, y, z)":>26}  {"퍼짐":>7}')
        for h in hv:
            L.append(f'   {h["t0"]:6.1f}~{h["t1"]:6.1f}s {h["dur"]:6.1f}s  '
                     f'({h["pos"][0]:+7.3f},{h["pos"][1]:+7.3f},{h["pos"][2]:6.3f})'
                     f'  {h["spread"]*100:6.1f}cm')
        L.append('  "퍼짐" 은 그 구간 점들이 중앙값에서 벗어난 최대 거리입니다.')

    L.append('')
    L.append('[주의]')
    L.append('  기준값(ground truth) 이 없으므로 정확도는 평가할 수 없습니다.')
    L.append('  여기 수치는 "라이다가 무엇을 봤는가" 이지 "그게 맞는가" 가 아닙니다.')
    L.append('=' * 62)
    return '\n'.join(L), hv, gaps


# ----------------------------------------------------------------------
def plot_3d(d, run_dir, hv):
    try:
        import plotly.graph_objects as go
    except ImportError:
        print('  plotly 없음 — 3D 생략 (pip install --user plotly)')
        return None

    raw = np.column_stack([d['raw_x'], d['raw_y'], d['raw_z']])
    kf = np.column_stack([d['kf_x'], d['kf_y'], d['kf_z']])
    t = d['t_sec']
    ok = np.isfinite(raw).all(axis=1)
    kok = np.isfinite(kf).all(axis=1)

    fig = go.Figure()
    if ok.sum():
        fig.add_trace(go.Scatter3d(
            x=raw[ok, 0], y=raw[ok, 1], z=raw[ok, 2],
            mode='markers', name='라이다 원본',
            marker=dict(size=2, color=t[ok], colorscale='Viridis',
                        colorbar=dict(title='시간 (s)')),
            hovertemplate='t=%{marker.color:.1f}s<br>'
                          '(%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'))
    if kok.sum():
        fig.add_trace(go.Scatter3d(
            x=kf[kok, 0], y=kf[kok, 1], z=kf[kok, 2],
            mode='lines', name='칼만필터',
            line=dict(width=3, color='crimson')))
    if hv:
        P = np.array([h['pos'] for h in hv])
        fig.add_trace(go.Scatter3d(
            x=P[:, 0], y=P[:, 1], z=P[:, 2], mode='markers+text',
            name='호버 지점',
            marker=dict(size=8, color='gold', symbol='diamond'),
            text=[f'{i+1}' for i in range(len(hv))], textposition='top center'))

    # 바닥면 표시
    if ok.sum():
        gx = [raw[ok, 0].min() - 0.5, raw[ok, 0].max() + 0.5]
        gy = [raw[ok, 1].min() - 0.5, raw[ok, 1].max() + 0.5]
        fig.add_trace(go.Surface(
            x=gx, y=gy, z=[[0, 0], [0, 0]], showscale=False,
            opacity=0.15, colorscale=[[0, 'gray'], [1, 'gray']],
            name='바닥', hoverinfo='skip'))

    fig.update_layout(
        title='비행 궤적 (마우스로 회전/확대)',
        scene=dict(xaxis_title='x (m)', yaxis_title='y (m)', zaxis_title='z (m)',
                   aspectmode='data'),
        margin=dict(l=0, r=0, t=40, b=0))
    out = os.path.join(run_dir, 'flight_3d.html')
    fig.write_html(out, include_plotlyjs='cdn')
    return out


def plot_2d(d, run_dir, gaps):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    raw = np.column_stack([d['raw_x'], d['raw_y'], d['raw_z']])
    kf = np.column_stack([d['kf_x'], d['kf_y'], d['kf_z']])
    t, speed = d['t_sec'], d['speed']
    ok = np.isfinite(raw).all(axis=1)
    kok = np.isfinite(kf).all(axis=1)

    fig, ax = plt.subplots(1, 3, figsize=(16, 5))

    if ok.sum():
        s = ax[0].scatter(raw[ok, 0], raw[ok, 1], c=t[ok], s=6, cmap='viridis')
        fig.colorbar(s, ax=ax[0], label='time (s)')
    if kok.sum():
        ax[0].plot(kf[kok, 0], kf[kok, 1], '-', color='crimson', lw=1, label='KF')
        ax[0].legend()
    ax[0].set_title('top-down  (x-y)')
    ax[0].set_xlabel('x (m)'); ax[0].set_ylabel('y (m)')
    ax[0].axis('equal'); ax[0].grid(alpha=.3)

    if ok.sum():
        ax[1].plot(t[ok], raw[ok, 2], '.', ms=3, label='raw')
    if kok.sum():
        ax[1].plot(t[kok], kf[kok, 2], '-', color='crimson', lw=1, label='KF')
    for a, b in gaps:
        ax[1].axvspan(a, b, color='red', alpha=.15)
    ax[1].set_title('altitude  (red = lost)')
    ax[1].set_xlabel('t (s)'); ax[1].set_ylabel('z (m)')
    ax[1].legend(); ax[1].grid(alpha=.3)

    sp = np.isfinite(speed)
    if sp.sum():
        ax[2].plot(t[sp], speed[sp], '-', lw=1)
    for a, b in gaps:
        ax[2].axvspan(a, b, color='red', alpha=.15)
    ax[2].set_title('speed')
    ax[2].set_xlabel('t (s)'); ax[2].set_ylabel('m/s')
    ax[2].grid(alpha=.3)

    fig.tight_layout()
    out = os.path.join(run_dir, 'flight_2d.png')
    fig.savefig(out, dpi=120)
    plt.close(fig)
    return out


# ----------------------------------------------------------------------
def run(run_dir):
    run_dir = os.path.abspath(os.path.expanduser(run_dir))
    d = load(run_dir)
    if len(d['t_sec']) == 0:
        print('  기록이 비어 있습니다.')
        return 1

    text, hv, gaps = make_report(d, run_dir)
    print(text)
    with open(os.path.join(run_dir, 'report.txt'), 'w') as f:
        f.write(text + '\n')

    p3 = plot_3d(d, run_dir, hv)
    p2 = plot_2d(d, run_dir, gaps)
    print('\n  생성된 파일')
    print(f'    {os.path.join(run_dir, "report.txt")}')
    if p2:
        print(f'    {p2}')
    if p3:
        print(f'    {p3}')
        print(f'\n  3D 보기:  xdg-open {p3}')
    print()
    return 0


def main(args=None):
    # ROS 없이도 쓸 수 있게: 인자로 경로를 주면 그대로 처리한다.
    argv = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    plain = [a for a in argv if not a.startswith('-')]
    if plain and os.path.isdir(os.path.expanduser(plain[0])):
        sys.exit(run(plain[0]))

    import rclpy
    from rclpy.node import Node
    rclpy.init(args=args)
    node = Node('flight_report')
    node.declare_parameter('run_dir', '')
    rd = str(node.get_parameter('run_dir').value)
    node.destroy_node()
    rclpy.shutdown()
    if not rd:
        print('run_dir 을 지정하세요:')
        print('  ros2 run dual_ouster_driver flight_report --ros-args '
              '-p run_dir:=~/flights/flight_YYYYmmdd_HHMMSS')
        sys.exit(1)
    sys.exit(run(rd))


if __name__ == '__main__':
    main()
