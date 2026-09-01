#!/usr/bin/env python3
"""Replay pointcloud_target_node's pipeline on one scan and explain every cluster.

Shows, for each candidate, whether it passed the point count, the extent bound
and the cylinder fit -- so the parameter that is rejecting the tube is visible
instead of guessed.

    python3 ~/drone_ws/cluster_report.py /lidar2/points \
        --zmin -0.76 --zmax 0.0 --rmin 1.0 --rmax 6.0 \
        --azmin 90 --azmax -90 --radius 0.0475 --near -1.26 -1.49
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from sklearn.cluster import DBSCAN

from grid_sensor_calibration.pointcloud_target_node import fit_circle_fixed_radius


class Rep(Node):
    def __init__(self, a):
        super().__init__("cluster_report")
        self.a = a
        self.done = False
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PointCloud2, a.topic, self.cb, qos)
        print(f"waiting for one scan on {a.topic} ...")

    def cb(self, msg):
        if self.done:
            return
        self.done = True
        a = self.a
        pts = pc2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = pts[np.isfinite(pts).all(axis=1)]

        keep = (pts[:, 2] >= a.zmin) & (pts[:, 2] <= a.zmax)
        r_all = np.hypot(pts[:, 0], pts[:, 1])
        keep &= (r_all >= a.rmin) & (r_all <= a.rmax)
        az_all = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
        if a.azmin <= a.azmax:
            keep &= (az_all >= a.azmin) & (az_all <= a.azmax)
        else:
            keep &= (az_all >= a.azmin) | (az_all <= a.azmax)
        pts = pts[keep]
        print(f"\nframe_id = {msg.header.frame_id}")
        print(f"crop 통과 점 수 : {len(pts)}")
        if len(pts) < a.minpts:
            print("  -> 크롭 단계에서 이미 비었습니다.")
            return

        t0 = time.time()
        labels = DBSCAN(eps=a.eps, min_samples=a.minpts).fit(pts).labels_
        dt = (time.time() - t0) * 1000
        n_cl = len(set(labels)) - (1 if -1 in labels else 0)
        print(f"DBSCAN          : {n_cl}개 클러스터, {dt:.0f} ms "
              f"({'10 Hz 예산 100 ms 초과!' if dt > 100 else '예산 이내'})")

        rows = []
        for label in np.unique(labels):
            if label < 0:
                continue
            c = pts[labels == label]
            n = len(c)
            extent = float((c.max(0) - c.min(0)).max())
            cx, cy = float(np.median(c[:, 0])), float(np.median(c[:, 1]))
            fit_xy, resid = fit_circle_fixed_radius(c[:, :2], a.radius)
            if extent > a.extent:
                verdict = f"버림: extent {extent:.2f} > {a.extent}"
            elif resid > a.maxresid:
                verdict = f"버림: residual {resid * 1000:.0f} mm > {a.maxresid * 1000:.0f}"
            else:
                verdict = "통과"
            d = (np.hypot(cx - a.near[0], cy - a.near[1])
                 if a.near else float("inf"))
            rows.append((d, n, extent, cx, cy, resid, verdict))

        if a.near:
            rows.sort(key=lambda r: r[0])
            print(f"\n지관 위치 ({a.near[0]}, {a.near[1]})에 가까운 순:")
        else:
            rows.sort(key=lambda r: -r[1])
            print("\n점 개수 많은 순:")

        print(f"{'거리':>6} {'점수':>6} {'extent':>7} {'중심 x':>8} {'중심 y':>8} "
              f"{'resid':>7}  판정")
        for d, n, extent, cx, cy, resid, verdict in rows[:12]:
            dd = "  -  " if d == float("inf") else f"{d:5.2f}"
            print(f"{dd:>6} {n:6d} {extent:7.2f} {cx:8.2f} {cy:8.2f} "
                  f"{resid * 1000:6.0f}mm  {verdict}")

        ok = [r for r in rows if r[6] == "통과"]
        print(f"\n통과한 클러스터 {len(ok)}개")
        if ok:
            big = max(ok, key=lambda r: r[1])
            print(f"  selection=largest 가 고를 것: "
                  f"({big[3]:+.2f}, {big[4]:+.2f})  {big[1]}점")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("topic")
    p.add_argument("--zmin", type=float, default=-0.75)
    p.add_argument("--zmax", type=float, default=0.0)
    p.add_argument("--rmin", type=float, default=1.0)
    p.add_argument("--rmax", type=float, default=6.0)
    p.add_argument("--azmin", type=float, default=90.0)
    p.add_argument("--azmax", type=float, default=-90.0)
    p.add_argument("--eps", type=float, default=0.15)
    p.add_argument("--minpts", type=int, default=6)
    p.add_argument("--extent", type=float, default=1.5)
    p.add_argument("--radius", type=float, default=0.0475)
    p.add_argument("--maxresid", type=float, default=0.04)
    p.add_argument("--near", type=float, nargs=2, default=None)
    a = p.parse_args()

    rclpy.init()
    node = Rep(a)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
