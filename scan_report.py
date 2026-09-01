#!/usr/bin/env python3
"""One-shot crop diagnosis for pointcloud_target_node.

Reads a single scan and reports how many points survive each stage of the
crop, then where the survivors actually are -- so range/azimuth/z can be set
from data instead of guessed.

    python3 ~/drone_ws/scan_report.py /lidar2/points --zmin -0.75 --zmax 0.0
"""
import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def hist(name, values, edges, unit):
    print(f"\n  {name}")
    for lo, hi in zip(edges[:-1], edges[1:]):
        n = int(((values >= lo) & (values < hi)).sum())
        bar = "#" * min(50, n // 20)
        print(f"    {lo:+7.1f} .. {hi:+7.1f} {unit}  {n:6d}  {bar}")


class Report(Node):
    def __init__(self, topic, zmin, zmax):
        super().__init__("scan_report")
        self.zmin, self.zmax = zmin, zmax
        self.done = False
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PointCloud2, topic, self.cb, qos)
        print(f"waiting for one scan on {topic} ...")

    def cb(self, msg):
        if self.done:
            return
        self.done = True
        pts = pc2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = pts[np.isfinite(pts).all(axis=1)]
        pts = pts[np.linalg.norm(pts, axis=1) > 0.1]

        print(f"\nframe_id = {msg.header.frame_id}")
        print(f"total points          : {len(pts)}")

        band = pts[(pts[:, 2] >= self.zmin) & (pts[:, 2] <= self.zmax)]
        print(f"in z band [{self.zmin}, {self.zmax}] : {len(band)}")
        if len(band) == 0:
            print("\n  -> z band is empty. Ground level is about "
                  f"{np.percentile(pts[:, 2], 5):+.2f} m here; "
                  "set zmin ~0.15 m above it.")
            return

        r = np.hypot(band[:, 0], band[:, 1])
        az = np.degrees(np.arctan2(band[:, 1], band[:, 0]))
        hist("range [m]", r, np.arange(0, 11, 1.0), "m")
        hist("azimuth [deg]", az, np.arange(-180, 181, 30), "d")

        print("\n  densest spots in the band (range x azimuth cells):")
        cells = {}
        for ri, ai in zip((r // 1.0).astype(int), ((az + 180) // 30).astype(int)):
            cells[(ri, ai)] = cells.get((ri, ai), 0) + 1
        for (ri, ai), n in sorted(cells.items(), key=lambda kv: -kv[1])[:8]:
            print(f"    range {ri}.0-{ri + 1}.0 m   "
                  f"azimuth {ai * 30 - 180:+4.0f}..{ai * 30 - 150:+4.0f} deg   "
                  f"{n} points")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("topic")
    ap.add_argument("--zmin", type=float, default=-0.75)
    ap.add_argument("--zmax", type=float, default=0.0)
    a = ap.parse_args()

    rclpy.init()
    node = Report(a.topic, a.zmin, a.zmax)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
