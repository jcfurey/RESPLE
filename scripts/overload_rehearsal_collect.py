#!/usr/bin/env python3
"""Diagnostics collector for scripts/overload_rehearsal.sh.

Subscribes to /resple_diagnostics and /odom for --duration wall seconds,
streams every diagnostics sample to --csv, and writes a per-leg summary of
the overload metrics to --summary:

    odom msgs / rate, max pose step (the "jump" metric the rehearsal exists
    to compare), backlog_ms max/mean, rt_factor min/mean, shed_scans,
    dropped_scans, cycle_overruns, gap_extrap_knots, DIVERGED windows.

Run inside a sourced workspace (rclpy + nav_msgs + estimate_msgs).
"""

import argparse
import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from estimate_msgs.msg import Diagnostics

FIELDS = ['t_wall', 'rt_factor', 'backlog_ms', 'latency_wall_ms', 'shed_scans',
          'dropped_scans', 'cycle_overruns', 'gap_extrap_knots', 'filter_state',
          'nis', 'lidar_buffer_depth', 'tf_conflict_active']


class Collector(Node):
    def __init__(self, writer):
        super().__init__('overload_rehearsal_collect')
        self.writer = writer
        self.rows = 0
        self.last = None            # last Diagnostics msg (cumulative counters)
        self.diverged = 0
        self.backlogs = []
        self.rtfs = []
        self.odom_count = 0
        self.max_step = 0.0
        self.prev_pos = None
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        # resple_diagnostics publishes BEST_EFFORT (QoS must match to connect).
        qos_be = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Diagnostics, 'resple_diagnostics', self.on_diag, qos_be)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos)

    def on_diag(self, m: Diagnostics):
        self.last = m
        self.backlogs.append(m.backlog_ms)
        if m.rt_factor > 0.0:
            self.rtfs.append(m.rt_factor)
        if m.filter_state == 2:
            self.diverged += 1
        self.writer.writerow([time.monotonic(), m.rt_factor, m.backlog_ms,
                              m.latency_wall_ms, m.shed_scans, m.dropped_scans,
                              m.cycle_overruns, m.gap_extrap_knots,
                              m.filter_state, m.nis, m.lidar_buffer_depth,
                              int(m.tf_conflict_active)])
        self.rows += 1

    def on_odom(self, m: Odometry):
        self.odom_count += 1
        p = m.pose.pose.position
        pos = (p.x, p.y, p.z)
        if self.prev_pos is not None:
            step = math.dist(pos, self.prev_pos)
            self.max_step = max(self.max_step, step)
        self.prev_pos = pos


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, required=True)
    ap.add_argument('--csv', required=True)
    ap.add_argument('--summary', required=True)
    args = ap.parse_args()

    rclpy.init()
    with open(args.csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(FIELDS)
        node = Collector(w)
        t0 = time.monotonic()
        deadline = t0 + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        elapsed = time.monotonic() - t0
        node.destroy_node()
    rclpy.shutdown()

    lines = []
    lines.append(f'  odom msgs           {node.odom_count}  ({node.odom_count / elapsed:.1f} Hz wall)')
    lines.append(f'  max pose step (m)   {node.max_step:.4f}   <-- the "jump" metric')
    if node.backlogs:
        lines.append(f'  backlog_ms          max {max(node.backlogs):.0f}  mean {sum(node.backlogs) / len(node.backlogs):.0f}')
    if node.rtfs:
        lines.append(f'  rt_factor           min {min(node.rtfs):.2f}  mean {sum(node.rtfs) / len(node.rtfs):.2f}'
                     '  (~= playback rate is healthy; with shedding read WITH shed_scans)')
    if node.last is not None:
        m = node.last
        lines.append(f'  shed_scans          {m.shed_scans}')
        lines.append(f'  dropped_scans       {m.dropped_scans}')
        lines.append(f'  cycle_overruns      {m.cycle_overruns}')
        lines.append(f'  gap_extrap_knots    {m.gap_extrap_knots}')
    lines.append(f'  DIVERGED windows    {node.diverged}')
    lines.append(f'  diag samples        {node.rows}')
    text = '\n'.join(lines)
    with open(args.summary, 'w') as f:
        f.write(text + '\n')
    print(text)


if __name__ == '__main__':
    main()
