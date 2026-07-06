"""ROS2 node for online VINS parameter auto-tuning using ground truth odometry.

Subscribes to:
  - /vins_estimator/odometry  (nav_msgs/Odometry) — VINS output
  - <ground_truth_topic>      (nav_msgs/Odometry) — reference trajectory

Publishes:
  - /vins_autotune/metrics         (std_msgs/Float64MultiArray) — ATE/RPE/scale
  - /vins_autotune/recommendations (std_msgs/String) — JSON recommendations
  - /vins_autotune/status          (std_msgs/String) — human-readable status

The node can operate in two modes:
  1. MONITOR mode (default): only computes and publishes metrics, no config changes.
  2. TUNE mode: runs coordinate-descent optimization, modifies the YAML config
     file in-place, and signals the user to restart VINS for each trial.
"""
from __future__ import annotations

import json
import math
import threading
import time
import logging
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import TransformStamped
import tf2_ros

from .metrics import PoseSample, MetricsResult, compute_metrics
from .trajectory_buffer import TrajectoryBuffer
from .config_editor import ConfigEditor, ParamSpec
from .tuner import CoordinateDescentTuner, TrialResult, default_cost

_logger = logging.getLogger("vins_autotune")


def _odom_to_sample(msg: Odometry) -> PoseSample:
    t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return PoseSample(
        timestamp=t,
        pos=np.array([p.x, p.y, p.z]),
        quat=np.array([q.x, q.y, q.z, q.w]),
    )


# Default parameter search space — tuned for fisheye_bag config
DEFAULT_PARAM_SPECS: Dict[str, ParamSpec] = {
    "max_cnt": ParamSpec("max_cnt", 450, 250, 600, 50, "feature count target", "int"),
    "min_dist": ParamSpec("min_dist", 15, 10, 25, 3, "feature min distance px", "int"),
    "F_threshold": ParamSpec("F_threshold", 2.0, 1.0, 3.5, 0.5, "RANSAC F threshold", "float"),
    "fast_threshold": ParamSpec("fast_threshold", 15, 7, 45, 4, "AGAST threshold", "int"),
    "keyframe_parallax": ParamSpec("keyframe_parallax", 10.0, 5.0, 20.0, 2.5, "keyframe parallax px", "float"),
    "acc_n": ParamSpec("acc_n", 0.35, 0.1, 1.0, 0.1, "accel noise std", "float"),
    "gyr_n": ParamSpec("gyr_n", 0.015, 0.005, 0.05, 0.005, "gyro noise std", "float"),
    "acc_w": ParamSpec("acc_w", 0.003, 0.001, 0.01, 0.001, "accel bias random walk", "float"),
    "gyr_w": ParamSpec("gyr_w", 0.001, 0.0003, 0.003, 0.0003, "gyro bias random walk", "float"),
}


class VinsAutotuneNode(Node):
    def __init__(self):
        super().__init__("vins_autotune")

        # --- Parameters ---
        self.declare_parameter("vins_odom_topic", "/vins_estimator/odometry")
        self.declare_parameter("ground_truth_topic", "/ground_truth/odom")
        self.declare_parameter("config_file", "")
        self.declare_parameter("mode", "monitor")  # "monitor" or "tune"
        self.declare_parameter("eval_window", 15.0)
        self.declare_parameter("max_passes", 3)
        self.declare_parameter("buffer_window", 30.0)
        self.declare_parameter("sync_dt", 0.05)
        self.declare_parameter("metrics_rate", 1.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("align_trajectories", True)

        self.vins_topic = self.get_parameter("vins_odom_topic").value
        self.gt_topic = self.get_parameter("ground_truth_topic").value
        self.config_file = self.get_parameter("config_file").value
        self.mode = self.get_parameter("mode").value
        self.eval_window = float(self.get_parameter("eval_window").value)
        self.max_passes = int(self.get_parameter("max_passes").value)
        self.align = bool(self.get_parameter("align_trajectories").value)

        # --- Trajectory buffer ---
        self.buffer = TrajectoryBuffer(
            window_sec=float(self.get_parameter("buffer_window").value),
            max_sync_dt=float(self.get_parameter("sync_dt").value),
        )

        # --- Latest metrics ---
        self._latest_metrics: Optional[MetricsResult] = None
        self._metrics_lock = threading.Lock()

        # --- Publishers ---
        self.pub_metrics = self.create_publisher(Float64MultiArray, "/vins_autotune/metrics", 10)
        self.pub_recommendations = self.create_publisher(String, "/vins_autotune/recommendations", 10)
        self.pub_status = self.create_publisher(String, "/vins_autotune/status", 10)
        self.pub_gt_path = self.create_publisher(String, "/vins_autotune/status", 10)

        if bool(self.get_parameter("publish_tf").value):
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None

        # --- Subscribers ---
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, self.vins_topic, self._vins_cb, qos)
        self.create_subscription(Odometry, self.gt_topic, self._gt_cb, qos)

        # --- Metrics timer ---
        rate = float(self.get_parameter("metrics_rate").value)
        self.create_timer(1.0 / max(rate, 0.1), self._publish_metrics)

        # --- Tuner (optional) ---
        self._tuner: Optional[CoordinateDescentTuner] = None
        self._tune_thread: Optional[threading.Thread] = None
        if self.mode == "tune":
            self._start_tuner()

        self.get_logger().info(
            f"vins_autotune started: mode={self.mode}, vins={self.vins_topic}, "
            f"gt={self.gt_topic}, config={self.config_file or '(none)'}"
        )
        self._publish_status(f"Node started in {self.mode} mode")

    # ---------- Callbacks ----------

    def _vins_cb(self, msg: Odometry) -> None:
        self.buffer.add_vins(_odom_to_sample(msg))

    def _gt_cb(self, msg: Odometry) -> None:
        self.buffer.add_gt(_odom_to_sample(msg))

    # ---------- Metrics ----------

    def _compute_current_metrics(self) -> Optional[MetricsResult]:
        vins, gt = self.buffer.get_synchronized_pairs()
        if len(vins) < 10:
            return None
        return compute_metrics(vins, gt, align=self.align)

    def _publish_metrics(self) -> None:
        metrics = self._compute_current_metrics()
        if metrics is None:
            return
        with self._metrics_lock:
            self._latest_metrics = metrics

        msg = Float64MultiArray()
        msg.data = [
            metrics.ate_trans,
            metrics.ate_rot,
            metrics.rpe_trans,
            metrics.rpe_rot,
            metrics.scale_drift,
            float(metrics.n_samples),
            metrics.vins_path_len,
            metrics.gt_path_len,
        ]
        self.pub_metrics.publish(msg)

        # Broadcast TF: vins_aligned → ground_truth frame for RViz visualization
        if self.tf_broadcaster and metrics.n_samples > 0:
            self._broadcast_tf(metrics)

    def _broadcast_tf(self, metrics: MetricsResult) -> None:
        """Publish a TF from 'map' to 'vins_aligned' showing current ATE offset."""
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "vins_aligned"
        # Use the last VINS position as a simple visualization
        vins, _ = self.buffer.get_all()
        if vins:
            tf.transform.translation.x = float(vins[-1].pos[0])
            tf.transform.translation.y = float(vins[-1].pos[1])
            tf.transform.translation.z = float(vins[-1].pos[2])
            tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)
        self.get_logger().info(text)

    def _publish_recommendation(self, rec: dict) -> None:
        msg = String()
        msg.data = json.dumps(rec)
        self.pub_recommendations.publish(msg)

    # ---------- Tuner ----------

    def _start_tuner(self) -> None:
        if not self.config_file:
            self.get_logger().error("TUNE mode requires 'config_file' parameter")
            return

        editor = ConfigEditor(self.config_file)
        # Load current values from config into specs
        specs = {}
        for key, spec in DEFAULT_PARAM_SPECS.items():
            val = editor.read_value(key)
            if val is not None:
                spec.value = val
            specs[key] = spec

        def eval_fn(trial_values: Dict[str, float]) -> MetricsResult:
            """Evaluate the current running VINS over eval_window seconds."""
            self._publish_status(
                f"Evaluating trial: {trial_values} — waiting {self.eval_window}s"
            )
            # Clear buffer to get fresh data for this trial
            self.buffer.clear()
            # Wait for VINS to produce data with the new config
            # (User must restart VINS manually, or we wait for new samples)
            start = time.time()
            while time.time() - start < self.eval_window:
                time.sleep(0.5)
                if self.buffer.vins_count > 0 and self.buffer.gt_count > 0:
                    # Wait a bit more to fill the window
                    pass
            metrics = self._compute_current_metrics()
            if metrics is None:
                self.get_logger().warn("No synchronized data during eval window")
                return MetricsResult()
            return metrics

        self._tuner = CoordinateDescentTuner(
            config_editor=editor,
            params=specs,
            eval_fn=eval_fn,
            eval_window=self.eval_window,
            max_passes=self.max_passes,
            cost_fn=default_cost,
        )

        def on_trial(tr: TrialResult) -> None:
            rec = {
                "param": tr.param_key,
                "value": tr.tried_value,
                "cost": tr.cost,
                "ate_trans": tr.metrics.ate_trans,
                "ate_rot": tr.metrics.ate_rot,
                "scale_drift": tr.metrics.scale_drift,
                "accepted": tr.accepted,
                "timestamp": time.time(),
            }
            self._publish_recommendation(rec)

        def run_thread():
            try:
                self._publish_status("Starting coordinate-descent tuning")
                best = self._tuner.run(on_trial=on_trial)
                self._publish_status(f"Tuning complete. Best params: {best}")
                self._publish_recommendation({"final_params": best, "done": True})
            except Exception as e:
                self.get_logger().error(f"Tuner failed: {e}")
                self._publish_status(f"Tuner error: {e}")

        self._tune_thread = threading.Thread(target=run_thread, daemon=True)
        self._tune_thread.start()

    # ---------- API for external access ----------

    def get_latest_metrics(self) -> Optional[MetricsResult]:
        with self._metrics_lock:
            return self._latest_metrics


def main(args=None):
    rclpy.init(args=args)
    node = VinsAutotuneNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()