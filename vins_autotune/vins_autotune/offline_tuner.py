"""Offline parameter tuner: runs VINS on a rosbag with different configs,
collects ATE against ground truth, and finds the best parameter set.

Usage:
  ros2 run vins_autotune vins_autotune_offline \
    --config config/fisheye_bag/fisheye_bag_config.yaml \
    --bag /path/to/rosbag \
    --gt-topic /ground_truth/odom \
    --vins-topic /vins_estimator/odometry \
    --eval-window 20 \
    --passes 3
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import logging
from typing import Dict, List, Optional, Tuple

import numpy as np

# Add the package to path when run standalone
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vins_autotune.config_editor import ConfigEditor, ParamSpec
from vins_autotune.metrics import PoseSample, MetricsResult, compute_metrics
from vins_autotune.tuner import CoordinateDescentTuner, TrialResult, default_cost

_logger = logging.getLogger("vins_autotune.offline")


def play_bag_and_collect(
    bag_path: str,
    vins_topic: str,
    gt_topic: str,
    duration: float,
    use_sim_time: bool = True,
) -> Tuple[List[PoseSample], List[PoseSample]]:
    """Play a rosbag for `duration` seconds and collect odometry messages.

    Uses `ros2 bag play` and subscribes via a temporary rclpy node.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry

    vins_samples: List[PoseSample] = []
    gt_samples: List[PoseSample] = []

    class Collector(Node):
        def __init__(self):
            super().__init__("offline_collector")
            qos = QoSProfile(depth=500, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Odometry, vins_topic, self._vins, qos)
            self.create_subscription(Odometry, gt_topic, self._gt, qos)

        def _vins(self, msg):
            vins_samples.append(_odom_to_sample(msg))

        def _gt(self, msg):
            gt_samples.append(_odom_to_sample(msg))

    def _odom_to_sample(msg: Odometry) -> PoseSample:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return PoseSample(
            timestamp=t,
            pos=np.array([p.x, p.y, p.z]),
            quat=np.array([q.x, q.y, q.z, q.w]),
        )

    rclpy.init()
    node = Collector()

    # Start bag playback in a subprocess
    cmd = ["ros2", "bag", "play", bag_path]
    if use_sim_time:
        cmd.extend(["--clock", "100"])
    _logger.info("Starting bag playback: %s", " ".join(cmd))
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

    start = time.time()
    while time.time() - start < duration and proc.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    proc.terminate()
    proc.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()

    _logger.info("Collected %d VINS samples, %d GT samples",
                 len(vins_samples), len(gt_samples))
    return vins_samples, gt_samples


def sync_samples(
    vins: List[PoseSample],
    gt: List[PoseSample],
    max_dt: float = 0.05,
) -> Tuple[List[PoseSample], List[PoseSample]]:
    """Time-synchronize two pose lists by nearest timestamp."""
    if not vins or not gt:
        return [], []
    gt_times = np.array([s.timestamp for s in gt])
    mv, mg = [], []
    for v in vins:
        idx = int(np.argmin(np.abs(gt_times - v.timestamp)))
        if abs(gt_times[idx] - v.timestamp) <= max_dt:
            mv.append(v)
            mg.append(gt[idx])
    return mv, mg


def run_offline_tuning(args: argparse.Namespace) -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")

    editor = ConfigEditor(args.config)
    editor.backup()

    # Build param specs from defaults, override with current config values
    from vins_autotune.autotune_node import DEFAULT_PARAM_SPECS
    specs = {}
    for key, spec in DEFAULT_PARAM_SPECS.items():
        val = editor.read_value(key)
        if val is not None:
            spec.value = val
        specs[key] = spec

    # Optionally restrict to a subset of parameters
    if args.params:
        specs = {k: specs[k] for k in args.params if k in specs}

    def eval_fn(trial_values: Dict[str, float]) -> MetricsResult:
        _logger.info("Evaluating with params: %s", trial_values)
        vins, gt = play_bag_and_collect(
            bag_path=args.bag,
            vins_topic=args.vins_topic,
            gt_topic=args.gt_topic,
            duration=args.eval_window,
        )
        vins_sync, gt_sync = sync_samples(vins, gt, max_dt=args.sync_dt)
        if len(vins_sync) < 10:
            _logger.warn("Insufficient synchronized samples (%d)", len(vins_sync))
            return MetricsResult()
        return compute_metrics(vins_sync, gt_sync, align=True)

    tuner = CoordinateDescentTuner(
        config_editor=editor,
        params=specs,
        eval_fn=eval_fn,
        eval_window=args.eval_window,
        max_passes=args.passes,
        cost_fn=default_cost,
    )

    history: List[dict] = []

    def on_trial(tr: TrialResult) -> None:
        history.append({
            "param": tr.param_key,
            "value": tr.tried_value,
            "cost": tr.cost,
            "ate_trans": tr.metrics.ate_trans,
            "ate_rot": tr.metrics.ate_rot,
            "scale_drift": tr.metrics.scale_drift,
            "accepted": tr.accepted,
        })

    best = tuner.run(on_trial=on_trial)

    result = {"best_params": best, "history": history}
    print("\n=== Tuning Results ===")
    print(json.dumps(result, indent=2))

    if args.output:
        with open(args.output, "w") as f:
            json.dump(result, f, indent=2)
        _logger.info("Results saved to %s", args.output)


def main():
    parser = argparse.ArgumentParser(description="Offline VINS parameter auto-tuner")
    parser.add_argument("--config", required=True, help="Path to VINS YAML config")
    parser.add_argument("--bag", required=True, help="Path to ROS2 bag")
    parser.add_argument("--gt-topic", default="/ground_truth/odom",
                        help="Ground truth odometry topic")
    parser.add_argument("--vins-topic", default="/vins_estimator/odometry",
                        help="VINS odometry topic")
    parser.add_argument("--eval-window", type=float, default=20.0,
                        help="Seconds of bag to evaluate per trial")
    parser.add_argument("--passes", type=int, default=3,
                        help="Max coordinate-descent passes")
    parser.add_argument("--sync-dt", type=float, default=0.05,
                        help="Max sync time difference (s)")
    parser.add_argument("--params", nargs="*", default=None,
                        help="Subset of parameters to tune (default: all)")
    parser.add_argument("--output", default=None,
                        help="Path to save JSON results")
    args = parser.parse_args()
    run_offline_tuning(args)


if __name__ == "__main__":
    main()