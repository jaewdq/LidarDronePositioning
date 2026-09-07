#!/usr/bin/env python3
"""Create a score-independent HTML report from one recorded run."""

import argparse
import base64
import html
import json
import os
import sys

import numpy as np
import yaml

from drone_localization.plot_run import (
    DEFAULT_RESULTS,
    TRAJ,
    default_params_path,
    find_runs,
    plot,
    coasting_mask,
    read_params,
    read_traj,
)


def _image_data(path):
    with open(path, 'rb') as stream:
        encoded = base64.b64encode(stream.read()).decode('ascii')
    return f'data:image/png;base64,{encoded}'


def default_scenario_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('drone_localization')
        return os.path.join(share, 'params', 'run_scenario.yaml')
    except Exception:
        return None


def read_scenario(path):
    if not path or not os.path.isfile(path):
        return None
    with open(path, encoding='utf-8') as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        return None
    name = str(value.get('name', '비행 시나리오')).strip()
    steps = [str(step).strip() for step in value.get('steps', [])
             if str(step).strip()]
    if not steps:
        return None
    return {'name': name, 'steps': steps}


def _trajectory_json(data, speed, params, coast):
    """Return compact, browser-safe JSON for the embedded 3D viewer."""
    gaps = np.r_[0.0, np.diff(data['t'])]
    payload = {
        't': data['t'].tolist(),
        'x': data['x_m'].tolist(),
        'y': data['y_m'].tolist(),
        'z': data['z_m'].tolist(),
        'speed': speed.tolist(),
        'covX': data['cov_x'].tolist(),
        'covY': data['cov_y'].tolist(),
        'covZ': data['cov_z'].tolist(),
        'coast': np.asarray(coast, dtype=bool).tolist(),
        'gap': gaps.tolist(),
        'field': {'x0': 0.0, 'x1': 24.0, 'y0': 0.0, 'y1': 15.0},
        'roi': None,
        'markers': [],
        'lidars': [],
    }
    if params:
        (x0, x1), (y0, y1), (z0, z1) = params['roi']
        payload['roi'] = {
            'x0': x0, 'x1': x1, 'y0': y0, 'y1': y1,
            'z0': z0, 'z1': z1,
        }
        payload['markers'] = [
            {'id': marker_id, 'x': marker_x, 'y': marker_y}
            for marker_id, marker_x, marker_y in params.get('markers', [])
        ]
        payload['lidars'] = [
            {'id': index, 'x': lidar_x, 'y': lidar_y}
            for index, (lidar_x, lidar_y) in enumerate(
                params.get('lidars', []), start=1)
        ]
    return json.dumps(
        payload, ensure_ascii=False, allow_nan=False,
        separators=(',', ':')).replace('</', '<\\/')


def _coasting_intervals(data, mask):
    """연속된 코스팅 샘플을 시간/위치 구간으로 묶는다."""
    mask = np.asarray(mask, dtype=bool)
    indices = np.flatnonzero(mask)
    if not len(indices):
        return []
    groups = np.split(indices, np.where(np.diff(indices) > 1)[0] + 1)
    intervals = []
    for group in groups:
        first, last = int(group[0]), int(group[-1])
        intervals.append({
            'first': first,
            'last': last,
            'samples': len(group),
            't0': float(data['t'][first]),
            't1': float(data['t'][last]),
            'x0': float(data['x_m'][first]),
            'y0': float(data['y_m'][first]),
            'x1': float(data['x_m'][last]),
            'y1': float(data['y_m'][last]),
        })
    return intervals


def write_html_report(run_dir, params=None, scenario=None):
    """Write flight_report.html using only trajectory data, without scoring."""
    trajectory_path = os.path.join(run_dir, TRAJ)
    data = read_traj(trajectory_path)
    if data is None:
        raise ValueError(f'샘플이 부족합니다: {trajectory_path}')

    trajectory_path, profiles_path = plot(run_dir, params, save_only=True)
    t = data['t']
    gaps = np.diff(t)
    speed = np.hypot(data['vx_mps'], data['vy_mps'])
    coasting = coasting_mask(run_dir, data)
    coast_intervals = _coasting_intervals(data, coasting)
    duration = float(t[-1])
    average_hz = float((len(t) - 1) / duration) if duration > 0 else 0.0
    team = os.path.basename(os.path.dirname(run_dir))
    run_name = os.path.basename(run_dir)
    trajectory_json = _trajectory_json(data, speed, params, coasting)
    inside_field = (
        (data['x_m'] >= 0.0) & (data['x_m'] <= 24.0) &
        (data['y_m'] >= 0.0) & (data['y_m'] <= 15.0))
    field_time = float(np.sum(gaps * inside_field[:-1]))
    previous_inside = np.r_[False, inside_field[:-1]]
    field_entries = int(np.count_nonzero(inside_field & ~previous_inside))
    field_exits = int(np.count_nonzero(~inside_field & previous_inside))
    slow = speed <= 0.15
    slow_time = float(np.sum(gaps * slow[:-1]))
    horizontal_distance = float(np.sum(np.hypot(
        np.diff(data['x_m']), np.diff(data['y_m']))))

    summary_rows = [
        ('기록 폴더', run_dir),
        ('팀 / 런', f'{team} / {run_name}'),
        ('기록 시간', f'{duration:.1f} s'),
        ('샘플', f'{len(t)}'),
        ('평균 주기', f'{average_hz:.2f} Hz'),
        ('샘플 간격',
         f'중앙 {np.median(gaps) * 1000:.0f} ms / '
         f'최대 {np.max(gaps) * 1000:.0f} ms'),
        ('코스팅', f'{int(coasting.sum())} / {len(t)} 샘플'),
        ('속도',
         f'중앙 {np.median(speed):.2f} m/s / '
         f'최대 {np.max(speed):.2f} m/s'),
        ('X 범위', f'{np.min(data["x_m"]):.2f} ~ {np.max(data["x_m"]):.2f} m'),
        ('Y 범위', f'{np.min(data["y_m"]):.2f} ~ {np.max(data["y_m"]):.2f} m'),
        ('Z 범위', f'{np.min(data["z_m"]):.2f} ~ {np.max(data["z_m"]):.2f} m'),
        ('시작 위치',
         f'({data["x_m"][0]:.2f}, {data["y_m"][0]:.2f}, '
         f'{data["z_m"][0]:.2f}) m'),
        ('종료 위치',
         f'({data["x_m"][-1]:.2f}, {data["y_m"][-1]:.2f}, '
         f'{data["z_m"][-1]:.2f}) m'),
        ('수평 이동 거리', f'{horizontal_distance:.1f} m'),
        ('필드 내부 체류', f'{field_time:.1f} s'),
        ('필드 진입 / 이탈', f'{field_entries} / {field_exits}회'),
        ('호버링 후보 시간',
         f'{slow_time:.1f} s (수평속도 ≤ 0.15 m/s)'),
    ]
    summary_html = ''.join(
        f'<tr><th>{html.escape(label)}</th><td>{html.escape(value)}</td></tr>'
        for label, value in summary_rows
    )
    if coast_intervals:
        coast_items = ''.join(
            '<li>'
            f'C{number}: {item["t0"]:.1f}–{item["t1"]:.1f} s '
            f'({item["samples"]}샘플), '
            f'({item["x0"]:.2f}, {item["y0"]:.2f}) → '
            f'({item["x1"]:.2f}, {item["y1"]:.2f}) m'
            '</li>'
            for number, item in enumerate(coast_intervals, start=1))
        coast_bars = ''.join(
            '<span class="coast-bar" '
            f'style="left:{100.0 * item["t0"] / max(duration, 1e-9):.3f}%;'
            f'width:{max(0.35, 100.0 * (item["t1"] - item["t0"]) / max(duration, 1e-9)):.3f}%" '
            f'title="C{number}: {item["t0"]:.1f}–{item["t1"]:.1f} s"></span>'
            for number, item in enumerate(coast_intervals, start=1))
        coast_html = (
            '<div class="coast-summary"><strong>코스팅 발생 구간</strong>'
            '<div class="coast-timeline">'
            f'{coast_bars}</div><div class="timeline-labels">'
            f'<span>0 s</span><span>{duration:.1f} s</span></div>'
            f'<ul>{coast_items}</ul>'
            '</div>')
    else:
        coast_html = '<div class="coast-summary"><strong>코스팅 없음</strong></div>'

    if scenario:
        scenario_steps = ''.join(
            f'<li>{html.escape(step)}</li>' for step in scenario['steps'])
        scenario_html = (
            '<section class="card scenario">'
            f'<h2>{html.escape(scenario["name"])}</h2>'
            f'<ol>{scenario_steps}</ol>'
            '</section>')
    else:
        scenario_html = ''

    page = f'''<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{html.escape(team)} {html.escape(run_name)} 비행 보고서</title>
  <style>
    body {{ margin: 0; background: #f3f5f7; color: #17202a;
            font-family: system-ui, sans-serif; }}
    main {{ max-width: 1180px; margin: auto; padding: 28px; }}
    .card {{ background: white; border-radius: 12px; padding: 20px;
             margin-bottom: 18px; box-shadow: 0 2px 10px #0001; }}
    h1, h2 {{ margin-top: 0; }}
    .note {{ color: #566573; }}
    table {{ border-collapse: collapse; width: 100%; }}
    th, td {{ border-bottom: 1px solid #dfe6e9; padding: 8px 10px;
              text-align: right; }}
    th:first-child, td:first-child {{ text-align: left; }}
    img {{ width: 100%; height: auto; }}
    .viewer {{ position: relative; width: 100%; height: 620px;
               background: #fff; border: 1px solid #d8dee4;
               border-radius: 10px; overflow: hidden; }}
    #flight3d {{ display: block; width: 100%; height: 100%; cursor: grab; }}
    #flight3d.dragging {{ cursor: grabbing; }}
    .viewer2d {{ position: relative; width: 100%; height: 680px;
                 background: #fff; border: 1px solid #d8dee4;
                 border-radius: 10px; overflow: hidden; }}
    #flight2d {{ display: block; width: 100%; height: 100%; cursor: pointer; }}
    #play-hint {{ position: absolute; left: 50%; top: 50%;
                  transform: translate(-50%, -50%); pointer-events: none;
                  padding: 10px 16px; border-radius: 8px;
                  background: #fffffff0; border: 1px solid #cbd5e1;
                  color: #334155; font-weight: 650; }}
    .print-only {{ display: none; }}
    .viewer-tools {{ display: flex; gap: 8px; margin-bottom: 10px;
                     align-items: center; flex-wrap: wrap; }}
    button {{ border: 1px solid #b8c2cc; background: #fff; border-radius: 7px;
              padding: 7px 12px; cursor: pointer; }}
    #point-info {{ position: absolute; display: none; pointer-events: none;
                   min-width: 205px; padding: 9px 11px; border-radius: 7px;
                   background: #fffffff2; color: #17202a; font-size: 13px;
                   line-height: 1.45; box-shadow: 0 3px 14px #0005; }}
    .scenario ol {{ margin: 0; padding-left: 24px; line-height: 1.8; }}
    .coast-summary {{ background: #f7f8fa; border: 1px solid #dfe3e8;
                      padding: 12px 14px; margin-top: 14px; }}
    .coast-summary ul {{ margin: 7px 0; padding-left: 22px; }}
    .coast-timeline {{ position: relative; height: 10px; margin-top: 10px;
                       border-radius: 5px; background: #dfe6e9; overflow: hidden; }}
    .coast-bar {{ position: absolute; top: 0; bottom: 0;
                  background: #64748b; min-width: 4px; }}
    .timeline-labels {{ display: flex; justify-content: space-between;
                        color: #566573; font-size: 12px; margin-top: 3px; }}
    .report-page {{ break-before: page; page-break-before: always; }}
    .report-page.first {{ break-before: auto; page-break-before: auto; }}
    @media print {{
      .screen-only {{ display: none !important; }}
      .print-only {{ display: block; }}
    }}
  </style>
</head>
<body><main>
  <section class="card">
    <h1>비행 기록 보고서</h1>
    <table>{summary_html}</table>
    {coast_html}
  </section>
  {scenario_html}
  <section class="card">
    <h2>3D 전체 비행 궤적</h2>
    <div class="viewer-tools">
      <button type="button" id="reset-view">기본 시점</button>
      <button type="button" id="top-view">위에서 보기</button>
      <button type="button" id="z-scale">Z축: 3배</button>
      <button type="button" id="path-mode">보기: 전체 경로</button>
      <span class="note">드래그: 회전 · 휠: 확대/축소 · 점 위에 마우스: 좌표 확인</span>
    </div>
    <div class="viewer">
      <canvas id="flight3d"></canvas>
      <div id="point-info"></div>
    </div>
  </section>
  <section class="card report-page first">
    <h2>2D 전체 비행 궤적</h2>
    <div class="viewer-tools screen-only">
      <button type="button" id="play2d">재생</button>
      <button type="button" id="reset2d">처음으로</button>
      <button type="button" id="speed2d">속도 ×10</button>
      <strong id="time2d">0.0 / {duration:.1f} s</strong>
      <span class="note">그림 클릭: 재생·일시정지 · 파랑: 정상 궤적 · 주황: 코스팅 · 빨강: ROI 밖</span>
    </div>
    <div class="viewer2d screen-only">
      <canvas id="flight2d"></canvas>
      <div id="play-hint">클릭하여 궤적 재생</div>
    </div>
    <img class="print-only" src="{_image_data(trajectory_path)}"
         alt="2D 전체 비행 궤적">
  </section>
  <section class="card report-page">
    <h2>고도&amp;수평 속도</h2>
    <img src="{_image_data(profiles_path)}" alt="고도와 수평 속도">
  </section>
</main>
<script>
(() => {{
  'use strict';
  const d = {trajectory_json};
  window.__flightData = d;
  const canvas = document.getElementById('flight3d');
  const info = document.getElementById('point-info');
  const ctx = canvas.getContext('2d');
  let yaw = -0.72;
  let pitch = 0.58;
  let zoom = 1;
  let zExaggeration = 3;
  let insideOnly = false;
  let dragging = false;
  let lastX = 0;
  let lastY = 0;
  let projected = [];

  const dataMin = {{
    x: Math.min(...d.x), y: Math.min(...d.y), z: Math.min(...d.z)
  }};
  const dataMax = {{
    x: Math.max(...d.x), y: Math.max(...d.y), z: Math.max(...d.z)
  }};
  const field = d.field;
  const roi = d.roi || field;
  const min = {{
    x: Math.min(dataMin.x, field.x0, roi.x0),
    y: Math.min(dataMin.y, field.y0, roi.y0),
    z: 0
  }};
  const max = {{
    x: Math.max(dataMax.x, field.x1, roi.x1),
    y: Math.max(dataMax.y, field.y1, roi.y1),
    z: Math.max(dataMax.z, roi.z1 || dataMax.z)
  }};
  const center = {{
    x: (min.x + max.x) / 2,
    y: (min.y + max.y) / 2,
    z: (min.z + max.z) / 2
  }};

  function resize() {{
    const ratio = window.devicePixelRatio || 1;
    const box = canvas.getBoundingClientRect();
    canvas.width = Math.max(1, Math.round(box.width * ratio));
    canvas.height = Math.max(1, Math.round(box.height * ratio));
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    draw();
  }}

  function project(x, y, z) {{
    const dx = x - center.x;
    const dy = y - center.y;
    const dz = (z - center.z) * zExaggeration;
    const cy = Math.cos(yaw), sy = Math.sin(yaw);
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const rx = cy * dx - sy * dy;
    const ry = sy * dx + cy * dy;
    const py = cp * ry - sp * dz;
    const depth = sp * ry + cp * dz;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    const span = Math.max(
      max.x - min.x, max.y - min.y,
      (max.z - min.z) * zExaggeration, 1);
    const scale = Math.min(w, h) * 0.76 * zoom / span;
    return {{x: w / 2 + rx * scale, y: h / 2 - py * scale, depth}};
  }}

  function line3(a, b, color, width = 1, dash = []) {{
    const p = project(a[0], a[1], a[2]);
    const q = project(b[0], b[1], b[2]);
    ctx.beginPath();
    ctx.moveTo(p.x, p.y);
    ctx.lineTo(q.x, q.y);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dash);
    ctx.stroke();
    ctx.setLineDash([]);
  }}

  function polygon3(points, fill) {{
    const p = points.map(point => project(...point));
    ctx.beginPath();
    ctx.moveTo(p[0].x, p[0].y);
    for (let i = 1; i < p.length; i += 1) ctx.lineTo(p[i].x, p[i].y);
    ctx.closePath();
    ctx.fillStyle = fill;
    ctx.fill();
  }}

  function rectangle3(bounds, color, width, dash = []) {{
    const corners = [
      [bounds.x0, bounds.y0, 0], [bounds.x1, bounds.y0, 0],
      [bounds.x1, bounds.y1, 0], [bounds.x0, bounds.y1, 0]
    ];
    for (let i = 0; i < 4; i += 1) {{
      line3(corners[i], corners[(i + 1) % 4], color, width, dash);
    }}
  }}

  function drawAxes() {{
    const o = [0, 0, 0];
    line3(o, [field.x1, 0, 0], '#111827', 1.8);
    line3(o, [0, field.y1, 0], '#111827', 1.8);
    line3(o, [0, 0, max.z], '#111827', 1.8);
    const labels = [
      ['X (m)', [field.x1, 0, 0]],
      ['Y (m)', [0, field.y1, 0]],
      ['Z (m)', [0, 0, max.z]]
    ];
    ctx.font = 'bold 13px system-ui';
    for (const [label, point] of labels) {{
      const p = project(...point);
      ctx.fillStyle = '#111827';
      ctx.fillText(label, p.x + 5, p.y - 5);
    }}
    ctx.font = '11px system-ui';
    for (let z = 1; z <= Math.floor(max.z + 1e-9); z += 1) {{
      const p = project(0, 0, z);
      ctx.fillStyle = '#374151';
      ctx.fillText(z.toFixed(0), p.x + 5, p.y + 3);
    }}
  }}

  function drawGrid() {{
    const step = 3;
    polygon3([
      [roi.x0, roi.y0, 0], [roi.x1, roi.y0, 0],
      [roi.x1, roi.y1, 0], [roi.x0, roi.y1, 0]
    ], '#fff3f3');
    polygon3([
      [field.x0, field.y0, 0], [field.x1, field.y0, 0],
      [field.x1, field.y1, 0], [field.x0, field.y1, 0]
    ], '#eef6ff');
    for (let x = field.x0; x <= field.x1 + 1e-9; x += step) {{
      line3([x, field.y0, 0], [x, field.y1, 0], '#c7cdd4', 1);
      const p = project(x, field.y0, 0);
      ctx.fillStyle = '#374151';
      ctx.font = '10px system-ui';
      if (Math.abs(x) > 1e-9) {{
        ctx.fillText(x.toFixed(0), p.x + 3, p.y + 12);
      }}
    }}
    for (let y = field.y0; y <= field.y1 + 1e-9; y += step) {{
      line3([field.x0, y, 0], [field.x1, y, 0], '#c7cdd4', 1);
      const p = project(field.x0, y, 0);
      ctx.fillStyle = '#374151';
      ctx.font = '10px system-ui';
      if (Math.abs(y) > 1e-9) {{
        ctx.fillText(y.toFixed(0), p.x - 17, p.y - 3);
      }}
    }}
    rectangle3(field, '#111827', 2.2);
    rectangle3(roi, '#d32f2f', 2, [7, 5]);
  }}

  function drawArucoMarkers() {{
    const patterns = {{
      1: ['1001', '0110', '1100', '0011'],
      2: ['1010', '0011', '1101', '0100'],
      3: ['1100', '0101', '1011', '0010'],
      4: ['1000', '1110', '0011', '0101']
    }};
    const size = 0.78;
    const cell = size / 6;
    for (const marker of d.markers) {{
      const bits = patterns[marker.id] || patterns[1];
      const x0 = marker.x - size / 2;
      const y0 = marker.y - size / 2;
      for (let row = 0; row < 6; row += 1) {{
        for (let col = 0; col < 6; col += 1) {{
          const black = row === 0 || row === 5 || col === 0 || col === 5 ||
                        bits[row - 1][col - 1] === '1';
          const xa = x0 + col * cell;
          const ya = y0 + (5 - row) * cell;
          polygon3([
            [xa, ya, 0.015], [xa + cell, ya, 0.015],
            [xa + cell, ya + cell, 0.015], [xa, ya + cell, 0.015]
          ], black ? '#111111' : '#ffffff');
        }}
      }}
      const label = project(marker.x + size * 0.62, marker.y + size * 0.62, 0.02);
      ctx.fillStyle = '#111827';
      ctx.font = 'bold 11px system-ui';
      ctx.fillText(`M${{marker.id}}`, label.x, label.y);
    }}
  }}

  function inside(bounds, x, y) {{
    return x >= bounds.x0 && x <= bounds.x1 &&
           y >= bounds.y0 && y <= bounds.y1;
  }}

  function pathColor(i) {{
    if (!inside(roi, d.x[i], d.y[i])) return '#d32f2f';
    return '#1565c0';
  }}

  function draw() {{
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    ctx.clearRect(0, 0, w, h);
    drawGrid();
    drawArucoMarkers();
    drawAxes();
    projected = new Array(d.x.length);
    for (let i = 0; i < d.x.length; i += 1) {{
      projected[i] = project(d.x[i], d.y[i], d.z[i]);
      if (i === 0) continue;
      if (insideOnly &&
          (!inside(field, d.x[i - 1], d.y[i - 1]) ||
           !inside(field, d.x[i], d.y[i]))) continue;
      const a = projected[i - 1];
      const b = projected[i];
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.strokeStyle = pathColor(i);
      ctx.lineWidth = 2.6;
      ctx.setLineDash([]);
      ctx.stroke();
      if (d.coast[i]) {{
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.strokeStyle = '#f97316';
        ctx.lineWidth = 1.15;
        ctx.stroke();
      }}
    }}
    if (projected.length) {{
      const visible = d.x.map((_, i) => i).filter(
        i => !insideOnly || inside(field, d.x[i], d.y[i]));
      if (!visible.length) return;
      const startIndex = visible[0];
      const endIndex = visible[visible.length - 1];
      const points = [
        [projected[startIndex], '#2fdf75', 6],
        [projected[endIndex], '#ff4d4f', 6]
      ];
      for (const [p, color, radius] of points) {{
        ctx.beginPath();
        ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 1.5;
        ctx.stroke();
      }}
      ctx.font = 'bold 11px system-ui';
      ctx.fillStyle = '#087a36';
      ctx.fillText(insideOnly ? 'FIELD IN' : 'START',
                   projected[startIndex].x + 9, projected[startIndex].y - 7);
      ctx.fillStyle = '#c62828';
      const end = projected[endIndex];
      ctx.fillText(insideOnly ? 'FIELD OUT' : 'END', end.x + 9, end.y - 7);
    }}
    ctx.fillStyle = '#111827';
    ctx.font = '12px system-ui';
    ctx.fillText('경로: 비행 궤적 파랑 · ROI 밖 빨강 · 코스팅 주황', 14, 22);
    ctx.fillText(`Z 축 ×${{zExaggeration}} (표시만 확대, 좌표값은 실제값)`, 14, 40);
    ctx.fillText(
      `FIELD ${{field.x1 - field.x0}}×${{field.y1 - field.y0}} m · ` +
      `ROI ${{roi.x1 - roi.x0}}×${{roi.y1 - roi.y0}} m · 격자 3×3 m`,
      14, 58);
  }}

  function nearestPoint(x, y) {{
    let best = -1;
    let bestDist = 11 * 11;
    for (let i = 0; i < projected.length; i += 1) {{
      const dx = projected[i].x - x;
      const dy = projected[i].y - y;
      const dist = dx * dx + dy * dy;
      if (dist < bestDist) {{ bestDist = dist; best = i; }}
    }}
    return best;
  }}

  canvas.addEventListener('pointerdown', event => {{
    dragging = true;
    lastX = event.clientX;
    lastY = event.clientY;
    canvas.classList.add('dragging');
    canvas.setPointerCapture(event.pointerId);
    info.style.display = 'none';
  }});
  canvas.addEventListener('pointerup', event => {{
    dragging = false;
    canvas.classList.remove('dragging');
    canvas.releasePointerCapture(event.pointerId);
  }});
  canvas.addEventListener('pointermove', event => {{
    const rect = canvas.getBoundingClientRect();
    if (dragging) {{
      yaw += (event.clientX - lastX) * 0.009;
      pitch = Math.max(-1.5, Math.min(1.5,
        pitch + (event.clientY - lastY) * 0.009));
      lastX = event.clientX;
      lastY = event.clientY;
      draw();
      return;
    }}
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    const i = nearestPoint(x, y);
    if (i < 0) {{ info.style.display = 'none'; return; }}
    info.innerHTML =
      `<b>샘플 ${{i}}</b><br>` +
      `시간: ${{d.t[i].toFixed(3)}} s<br>` +
      `XYZ: (${{d.x[i].toFixed(3)}}, ${{d.y[i].toFixed(3)}}, ` +
      `${{d.z[i].toFixed(3)}}) m<br>` +
      `속도: ${{d.speed[i].toFixed(3)}} m/s<br>` +
      `Cov: (${{d.covX[i].toFixed(5)}}, ${{d.covY[i].toFixed(5)}}, ` +
      `${{d.covZ[i].toFixed(5)}})<br>` +
      `직전 간격: ${{(d.gap[i] * 1000).toFixed(1)}} ms`;
    info.style.display = 'block';
    info.style.left = `${{Math.min(x + 14, rect.width - 230)}}px`;
    info.style.top = `${{Math.min(y + 14, rect.height - 155)}}px`;
  }});
  canvas.addEventListener('pointerleave', () => {{
    if (!dragging) info.style.display = 'none';
  }});
  canvas.addEventListener('wheel', event => {{
    event.preventDefault();
    zoom = Math.max(0.3, Math.min(8, zoom * Math.exp(-event.deltaY * 0.001)));
    draw();
  }}, {{passive: false}});
  document.getElementById('reset-view').addEventListener('click', () => {{
    yaw = -0.72; pitch = 0.58; zoom = 1; draw();
  }});
  document.getElementById('top-view').addEventListener('click', () => {{
    yaw = 0; pitch = 0; zoom = 1; draw();
  }});
  document.getElementById('z-scale').addEventListener('click', event => {{
    zExaggeration = zExaggeration === 1 ? 3 : 1;
    event.currentTarget.textContent = `Z축: ${{zExaggeration}}배`;
    draw();
  }});
  document.getElementById('path-mode').addEventListener('click', event => {{
    insideOnly = !insideOnly;
    event.currentTarget.textContent = insideOnly ?
      '보기: 필드 내부만' : '보기: 전체 경로';
    info.style.display = 'none';
    draw();
  }});
  window.addEventListener('resize', resize);
  resize();
}})();
</script>
<script>
(() => {{
  'use strict';
  const d = window.__flightData;
  const canvas = document.getElementById('flight2d');
  const ctx = canvas.getContext('2d');
  const playButton = document.getElementById('play2d');
  const resetButton = document.getElementById('reset2d');
  const speedButton = document.getElementById('speed2d');
  const timeLabel = document.getElementById('time2d');
  const hint = document.getElementById('play-hint');
  const field = d.field;
  const roi = d.roi || field;
  const duration = d.t[d.t.length - 1] || 0;
  const patterns = {{
    1: ['1001', '0110', '1100', '0011'],
    2: ['1010', '0011', '1101', '0100'],
    3: ['1100', '0101', '1011', '0010'],
    4: ['1000', '1110', '0011', '0101']
  }};
  let playing = false;
  let started = false;
  let shownTime = 0;
  let shownIndex = 0;
  let playbackSpeed = 10;
  let previousFrame = null;
  let animationId = null;
  let transform = null;

  function inside(bounds, x, y) {{
    return x >= bounds.x0 && x <= bounds.x1 &&
           y >= bounds.y0 && y <= bounds.y1;
  }}

  function resize2d() {{
    const ratio = window.devicePixelRatio || 1;
    const box = canvas.getBoundingClientRect();
    canvas.width = Math.max(1, Math.round(box.width * ratio));
    canvas.height = Math.max(1, Math.round(box.height * ratio));
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    const margin = {{left: 58, right: 30, top: 28, bottom: 45}};
    const minX = Math.min(Math.min(...d.x), roi.x0, field.x0) - 0.6;
    const maxX = Math.max(Math.max(...d.x), roi.x1, field.x1) + 0.6;
    const minY = Math.min(Math.min(...d.y), roi.y0, field.y0) - 0.6;
    const maxY = Math.max(Math.max(...d.y), roi.y1, field.y1) + 0.6;
    const usableW = Math.max(1, box.width - margin.left - margin.right);
    const usableH = Math.max(1, box.height - margin.top - margin.bottom);
    const scale = Math.min(usableW / (maxX - minX), usableH / (maxY - minY));
    const drawW = (maxX - minX) * scale;
    const drawH = (maxY - minY) * scale;
    transform = {{
      minX, minY, maxY, scale,
      ox: margin.left + (usableW - drawW) / 2,
      oy: margin.top + (usableH - drawH) / 2
    }};
    draw2d();
  }}

  function point(x, y) {{
    return {{
      x: transform.ox + (x - transform.minX) * transform.scale,
      y: transform.oy + (transform.maxY - y) * transform.scale
    }};
  }}

  function fillWorldRect(bounds, fill, stroke, dash = []) {{
    const a = point(bounds.x0, bounds.y1);
    const b = point(bounds.x1, bounds.y0);
    ctx.fillStyle = fill;
    ctx.fillRect(a.x, a.y, b.x - a.x, b.y - a.y);
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 2;
    ctx.setLineDash(dash);
    ctx.strokeRect(a.x, a.y, b.x - a.x, b.y - a.y);
    ctx.setLineDash([]);
  }}

  function drawBase() {{
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    ctx.clearRect(0, 0, w, h);
    fillWorldRect(roi, '#fff3f3', '#d32f2f', [7, 5]);
    fillWorldRect(field, '#eef6ff', '#111827');
    ctx.strokeStyle = '#c7cdd4';
    ctx.lineWidth = 1;
    ctx.font = '11px system-ui';
    ctx.fillStyle = '#374151';
    for (let x = field.x0; x <= field.x1 + 1e-9; x += 3) {{
      const a = point(x, field.y0);
      const b = point(x, field.y1);
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      if (Math.abs(x) > 1e-9) ctx.fillText(x.toFixed(0), a.x - 4, a.y + 17);
    }}
    for (let y = field.y0; y <= field.y1 + 1e-9; y += 3) {{
      const a = point(field.x0, y);
      const b = point(field.x1, y);
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      if (Math.abs(y) > 1e-9) ctx.fillText(y.toFixed(0), a.x - 24, a.y + 4);
    }}
    const xLabel = point(field.x1, field.y0);
    const yLabel = point(field.x0, field.y1);
    ctx.fillStyle = '#111827';
    ctx.font = 'bold 12px system-ui';
    ctx.fillText('X (m)', xLabel.x - 10, xLabel.y + 34);
    ctx.fillText('Y (m)', yLabel.x - 45, yLabel.y + 4);
  }}

  function drawPath() {{
    for (let i = 1; i <= shownIndex; i += 1) {{
      const a = point(d.x[i - 1], d.y[i - 1]);
      const b = point(d.x[i], d.y[i]);
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.strokeStyle = inside(roi, d.x[i], d.y[i]) ? '#1565c0' : '#d32f2f';
      ctx.lineWidth = 2.5;
      ctx.stroke();
      if (d.coast[i]) {{
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.strokeStyle = '#f97316';
        ctx.lineWidth = 1.15;
        ctx.stroke();
      }}
    }}
  }}

  function drawAruco() {{
    const size = Math.max(24, Math.min(38, transform.scale * 0.78));
    const cell = size / 6;
    for (const marker of d.markers) {{
      const center = point(marker.x, marker.y);
      const bits = patterns[marker.id] || patterns[1];
      const left = center.x - size / 2;
      const top = center.y - size / 2;
      for (let row = 0; row < 6; row += 1) {{
        for (let col = 0; col < 6; col += 1) {{
          const black = row === 0 || row === 5 || col === 0 || col === 5 ||
                        bits[row - 1][col - 1] === '1';
          ctx.fillStyle = black ? '#111111' : '#ffffff';
          ctx.fillRect(left + col * cell, top + row * cell, cell + 0.3, cell + 0.3);
        }}
      }}
      ctx.strokeStyle = '#111111';
      ctx.lineWidth = 1;
      ctx.strokeRect(left, top, size, size);
      ctx.fillStyle = '#111827';
      ctx.font = 'bold 11px system-ui';
      ctx.textAlign = 'center';
      ctx.fillText('M' + marker.id, center.x, top - 6);
      ctx.textAlign = 'start';
    }}
  }}

  function drawLidars() {{
    for (const lidar of d.lidars) {{
      const p = point(lidar.x, lidar.y);
      ctx.beginPath();
      ctx.moveTo(p.x, p.y - 7);
      ctx.lineTo(p.x - 7, p.y + 6);
      ctx.lineTo(p.x + 7, p.y + 6);
      ctx.closePath();
      ctx.fillStyle = '#111827';
      ctx.fill();
      ctx.font = '11px system-ui';
      ctx.fillText('lidar' + lidar.id, p.x + 9, p.y + 4);
    }}
  }}

  function drawPosition() {{
    const start = point(d.x[0], d.y[0]);
    ctx.beginPath(); ctx.arc(start.x, start.y, 6, 0, Math.PI * 2);
    ctx.fillStyle = '#2fdf75'; ctx.fill();
    ctx.strokeStyle = '#087a36'; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.fillStyle = '#087a36'; ctx.font = 'bold 11px system-ui';
    ctx.fillText('START', start.x + 9, start.y - 7);

    if (shownIndex > 0 && shownIndex < d.x.length - 1) {{
      const current = point(d.x[shownIndex], d.y[shownIndex]);
      ctx.beginPath(); ctx.arc(current.x, current.y, 5, 0, Math.PI * 2);
      ctx.fillStyle = '#06b6d4'; ctx.fill();
      ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1.5; ctx.stroke();
    }}
    if (shownIndex >= d.x.length - 1) {{
      const end = point(d.x[d.x.length - 1], d.y[d.y.length - 1]);
      ctx.beginPath(); ctx.arc(end.x, end.y, 6, 0, Math.PI * 2);
      ctx.fillStyle = '#ff4d4f'; ctx.fill();
      ctx.strokeStyle = '#c62828'; ctx.lineWidth = 1.5; ctx.stroke();
      ctx.fillStyle = '#c62828'; ctx.font = 'bold 11px system-ui';
      ctx.fillText('END', end.x + 9, end.y - 7);
    }}
  }}

  function draw2d() {{
    if (!transform) return;
    drawBase();
    drawPath();
    drawAruco();
    drawLidars();
    drawPosition();
    timeLabel.textContent = shownTime.toFixed(1) + ' / ' + duration.toFixed(1) + ' s';
  }}

  function setPlaying(value) {{
    playing = value;
    playButton.textContent = playing ? '일시정지' : '재생';
    hint.style.display = started ? 'none' : 'block';
    previousFrame = null;
    if (playing) animationId = requestAnimationFrame(animate);
    else if (animationId !== null) cancelAnimationFrame(animationId);
  }}

  function reset2d() {{
    setPlaying(false);
    started = false;
    shownTime = 0;
    shownIndex = 0;
    hint.style.display = 'block';
    draw2d();
  }}

  function toggle2d() {{
    if (shownIndex >= d.x.length - 1) reset2d();
    started = true;
    hint.style.display = 'none';
    setPlaying(!playing);
  }}

  function animate(timestamp) {{
    if (!playing) return;
    if (previousFrame === null) previousFrame = timestamp;
    const elapsed = Math.min(0.1, (timestamp - previousFrame) / 1000);
    previousFrame = timestamp;
    shownTime = Math.min(duration, shownTime + elapsed * playbackSpeed);
    while (shownIndex + 1 < d.t.length && d.t[shownIndex + 1] <= shownTime) {{
      shownIndex += 1;
    }}
    draw2d();
    if (shownIndex >= d.x.length - 1 || shownTime >= duration) {{
      shownTime = duration;
      shownIndex = d.x.length - 1;
      setPlaying(false);
      draw2d();
      return;
    }}
    animationId = requestAnimationFrame(animate);
  }}

  canvas.addEventListener('click', toggle2d);
  playButton.addEventListener('click', toggle2d);
  resetButton.addEventListener('click', reset2d);
  speedButton.addEventListener('click', () => {{
    playbackSpeed = playbackSpeed === 5 ? 10 : playbackSpeed === 10 ? 20 : 5;
    speedButton.textContent = '속도 ×' + playbackSpeed;
  }});
  window.addEventListener('resize', resize2d);
  resize2d();
}})();
</script>
</body></html>'''

    output_path = os.path.join(run_dir, 'flight_report.html')
    with open(output_path, 'w', encoding='utf-8') as stream:
        stream.write(page)
    return output_path


def main():
    parser = argparse.ArgumentParser(
        description='점수와 독립적인 비행 궤적 HTML 보고서를 만든다.')
    parser.add_argument('--run', help='런 폴더(기본: 가장 최근 런)')
    parser.add_argument('--results-dir', default=DEFAULT_RESULTS,
                        help=f'런 상위 폴더(기본: {DEFAULT_RESULTS})')
    parser.add_argument('--params', help='ROI/LiDAR params YAML')
    parser.add_argument('--scenario', help='비행 시나리오 YAML')
    parser.add_argument('--no-scenario', action='store_true',
                        help='시나리오 영역을 표시하지 않음')
    args, _ = parser.parse_known_args(
        [arg for arg in sys.argv[1:] if arg != '--ros-args'])

    if args.run:
        run_dir = os.path.abspath(os.path.expanduser(args.run))
    else:
        runs = find_runs(os.path.expanduser(args.results_dir))
        if not runs:
            print(f'{args.results_dir} 아래에 {TRAJ}를 가진 런이 없습니다.')
            return 1
        run_dir = runs[0]

    if not os.path.isfile(os.path.join(run_dir, TRAJ)):
        print(f'{TRAJ}가 없습니다: {run_dir}')
        return 1

    params_path = args.params or default_params_path()
    params = read_params(params_path) if params_path else None
    scenario = None
    if not args.no_scenario:
        run_scenario = os.path.join(run_dir, 'run_scenario.yaml')
        scenario_path = args.scenario or (
            run_scenario if os.path.isfile(run_scenario)
            else default_scenario_path())
        scenario = read_scenario(scenario_path)
    try:
        output = write_html_report(run_dir, params, scenario)
    except (OSError, ValueError, KeyError) as error:
        print(f'보고서 생성 실패: {error}')
        return 1

    print(f'HTML 보고서: {output}')
    print(f'열기: xdg-open {output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
