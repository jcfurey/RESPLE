#!/usr/bin/env python3
"""End-to-end smoke checker for the RESPLE node (see scripts/e2e_smoke.sh).

Collects /odom and /resple_diagnostics while the data_injector drives the
node with a stationary synthetic scene (ground truth: the pose stays at the
origin), then asserts the pass criteria and prints a verdict table.

Run inside a sourced workspace (needs rclpy + nav_msgs + estimate_msgs).
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from estimate_msgs.msg import Diagnostics


class SmokeCollector(Node):
    def __init__(self):
        super().__init__('e2e_smoke_check')
        self.positions = []          # (t_wall, x, y, z)
        self.speeds = []             # |v| body
        self.diverged_count = 0
        self.tf_conflict_count = 0
        self.diag_count = 0
        # LIVENESS OF THE MEASUREMENT UPDATE. Everything above is satisfied by
        # a node that fuses nothing at all: on a stationary scene, dead
        # reckoning sits exactly on the ground truth this gate checks.
        # Demonstrated, not theorised — running the node with
        # `-p nn_max_sq_dist:=1e-9` (k-NN gate rejects every neighbour, so
        # zero point-to-plane rows ever reach the IEKF) produced all seven
        # original PASSes with max|pos| = final|pos| = final|v| = 0.0000 and
        # corresp_used = 0. These fields are what make that impossible.
        self.corresp_used_last = 0
        self.corresp_used_seen = False
        self.numerical_failures = 0
        self.cov_sanitized = 0
        self.frames_with_corresp = 0
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        # resple_diagnostics publishes BEST_EFFORT — a reliable subscription
        # would silently receive nothing (QoS incompatibility).
        qos_be = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos)
        self.create_subscription(Diagnostics, 'resple_diagnostics', self.on_diag, qos_be)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.positions.append((time.monotonic(), p.x, p.y, p.z))
        self.speeds.append(math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z))

    def on_diag(self, msg: Diagnostics):
        self.diag_count += 1
        if msg.filter_state == 2:  # resple::health::FilterState::DIVERGED
            self.diverged_count += 1
        if msg.tf_conflict_active:
            self.tf_conflict_count += 1
        # corresp_used is PER-FRAME (the worker copies the last IEKF update's
        # CorrespStats into each message), not cumulative — so count frames
        # that fused something rather than looking for monotonic growth.
        self.corresp_used_seen = True
        self.corresp_used_last = msg.corresp_used
        if msg.corresp_used > 0:
            self.frames_with_corresp += 1
        # Both of these are cumulative and documented as MUST-stay-0.
        self.numerical_failures = max(self.numerical_failures,
                                      msg.iekf_numerical_failures)
        self.cov_sanitized = max(self.cov_sanitized, msg.cov_sanitized)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=30.0,
                    help='collection window (wall seconds)')
    ap.add_argument('--min-odom-msgs', type=int, default=200,
                    help='minimum /odom messages (steady state is ~knot rate, '
                         '~100 Hz, after a few seconds of init)')
    ap.add_argument('--max-drift-m', type=float, default=0.30,
                    help='pose must never leave this ball around the origin')
    ap.add_argument('--final-drift-m', type=float, default=0.20,
                    help='mean position norm over the last 50 msgs')
    ap.add_argument('--final-speed-mps', type=float, default=0.05,
                    help='mean |v| over the last 50 msgs (scene is static)')
    ap.add_argument('--min-corresp-frames', type=int, default=100,
                    help='minimum diagnostics frames reporting corresp_used > 0. '
                         'This is the criterion a no-op estimator cannot meet — '
                         'without it the whole gate passes with the LiDAR '
                         'measurement update dead (see SmokeCollector).')
    args = ap.parse_args()

    rclpy.init()
    node = SmokeCollector()
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    norms = [math.sqrt(x * x + y * y + z * z) for _, x, y, z in node.positions]
    tail = norms[-50:] if norms else []
    tail_v = node.speeds[-50:] if node.speeds else []
    checks = [
        ('odom messages >= %d' % args.min_odom_msgs,
         len(norms), len(norms) >= args.min_odom_msgs),
        ('max |pos| <= %.2f m' % args.max_drift_m,
         max(norms) if norms else float('nan'),
         bool(norms) and max(norms) <= args.max_drift_m),
        ('final mean |pos| <= %.2f m' % args.final_drift_m,
         sum(tail) / len(tail) if tail else float('nan'),
         bool(tail) and sum(tail) / len(tail) <= args.final_drift_m),
        ('final mean |v| <= %.2f m/s' % args.final_speed_mps,
         sum(tail_v) / len(tail_v) if tail_v else float('nan'),
         bool(tail_v) and sum(tail_v) / len(tail_v) <= args.final_speed_mps),
        ('diagnostics received', node.diag_count, node.diag_count > 0),
        ('never DIVERGED', node.diverged_count, node.diverged_count == 0),
        ('TF guard quiet', node.tf_conflict_count, node.tf_conflict_count == 0),
        # The measurement update must actually be running. Everything above
        # this line is satisfied by a node that fuses nothing.
        ('frames fusing LiDAR >= %d' % args.min_corresp_frames,
         node.frames_with_corresp,
         node.corresp_used_seen and node.frames_with_corresp >= args.min_corresp_frames),
        ('IEKF numerical failures', node.numerical_failures,
         node.numerical_failures == 0),
        ('covariance sanitized', node.cov_sanitized, node.cov_sanitized == 0),
    ]
    ok = True
    print('%-32s %12s  %s' % ('check', 'value', 'verdict'))
    for name, value, passed in checks:
        ok &= passed
        vstr = ('%.4f' % value) if isinstance(value, float) else str(value)
        print('%-32s %12s  %s' % (name, vstr, 'PASS' if passed else 'FAIL'))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
