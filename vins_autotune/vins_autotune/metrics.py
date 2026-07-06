"""Quality metrics for VINS odometry evaluation against ground truth.

Computes Absolute Trajectory Error (ATE), Relative Pose Error (RPE),
scale drift and rotation drift on time-synchronized pose pairs.
"""
from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass
class PoseSample:
    timestamp: float
    pos: np.ndarray          # (3,) translation
    quat: np.ndarray         # (4,) [x, y, z, w] quaternion


@dataclass
class MetricsResult:
    ate_trans: float = 0.0       # RMSE of translation ATE (meters)
    ate_rot: float = 0.0         # RMSE of rotation ATE (degrees)
    rpe_trans: float = 0.0       # RMSE relative translation error (meters)
    rpe_rot: float = 0.0         # RMSE relative rotation error (degrees)
    scale_drift: float = 1.0     # ratio gt_dist / vins_dist
    n_samples: int = 0
    vins_path_len: float = 0.0
    gt_path_len: float = 0.0
    extra: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {
            "ate_trans": self.ate_trans,
            "ate_rot": self.ate_rot,
            "rpe_trans": self.rpe_trans,
            "rpe_rot": self.rpe_rot,
            "scale_drift": self.scale_drift,
            "n_samples": self.n_samples,
            "vins_path_len": self.vins_path_len,
            "gt_path_len": self.gt_path_len,
        }


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """Convert [x, y, z, w] quaternion to 3x3 rotation matrix."""
    x, y, z, w = q
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotmat_to_angle(R: np.ndarray) -> float:
    """Rotation angle (radians) from a rotation matrix."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return np.arccos(c)


def umeyama_alignment(model: np.ndarray, target: np.ndarray,
                      with_scale: bool = True) -> Tuple[float, np.ndarray, np.ndarray]:
    """Compute Sim3 (s, R, t) that aligns `model` points to `target` points.

    Returns (scale, rotation 3x3, translation 3).
    Uses the standard Umeyama method.
    """
    assert model.shape == target.shape
    n = model.shape[0]
    mu_m = model.mean(axis=0)
    mu_t = target.mean(axis=0)
    model_c = model - mu_m
    target_c = target - mu_t

    # Cross-covariance
    H = model_c.T @ target_c / n
    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T

    var_m = np.sum(model_c ** 2) / n
    if with_scale:
        scale = np.trace(np.diag(S) @ D) / var_m
    else:
        scale = 1.0

    t = mu_t - scale * R @ mu_m
    return scale, R, t


def path_length(positions: np.ndarray) -> float:
    """Total path length from an (N, 3) array of positions."""
    if positions.shape[0] < 2:
        return 0.0
    diffs = np.diff(positions, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def compute_metrics(vins: List[PoseSample], gt: List[PoseSample],
                    align: bool = True) -> MetricsResult:
    """Compute ATE/RPE/scale between two synchronized pose trajectories.

    The lists must already be time-synchronized (same length, same order).
    If `align` is True, a Sim3 Umeyama alignment is computed on the full
    trajectory and applied to VINS before error computation.
    """
    res = MetricsResult()
    n = min(len(vins), len(gt))
    if n < 2:
        return res

    vins_pos = np.array([s.pos for s in vins[:n]])
    gt_pos = np.array([s.pos for s in gt[:n]])
    vins_rot = [quat_to_rotmat(s.quat) for s in vins[:n]]
    gt_rot = [quat_to_rotmat(s.quat) for s in gt[:n]]

    res.vins_path_len = path_length(vins_pos)
    res.gt_path_len = path_length(gt_pos)

    # Scale + alignment
    if align:
        scale, R_align, t_align = umeyama_alignment(vins_pos, gt_pos, with_scale=True)
        res.scale_drift = scale
        vins_pos_aligned = (scale * (R_align @ vins_pos.T)).T + t_align
        vins_rot_aligned = [R_align @ R for R in vins_rot]
    else:
        vins_pos_aligned = vins_pos
        vins_rot_aligned = vins_rot

    # ATE translation
    trans_err = np.linalg.norm(vins_pos_aligned - gt_pos, axis=1)
    res.ate_trans = float(np.sqrt(np.mean(trans_err ** 2)))

    # ATE rotation (degrees)
    rot_errs = []
    for R_v, R_g in zip(vins_rot_aligned, gt_rot):
        R_err = R_g @ R_v.T
        rot_errs.append(np.degrees(rotmat_to_angle(R_err)))
    res.ate_rot = float(np.sqrt(np.mean(np.array(rot_errs) ** 2)))

    # RPE — consecutive relative motion
    rpe_t = []
    rpe_r = []
    for i in range(1, n):
        d_gt_t = gt_pos[i] - gt_pos[i - 1]
        d_vins_t = vins_pos_aligned[i] - vins_pos_aligned[i - 1]
        rpe_t.append(np.linalg.norm(d_gt_t - d_vins_t))
        d_gt_r = gt_rot[i] @ gt_rot[i - 1].T
        d_vins_r = vins_rot_aligned[i] @ vins_rot_aligned[i - 1].T
        rpe_r.append(np.degrees(rotmat_to_angle(d_gt_r @ d_vins_r.T)))
    res.rpe_trans = float(np.sqrt(np.mean(np.array(rpe_t) ** 2)))
    res.rpe_rot = float(np.sqrt(np.mean(np.array(rpe_r) ** 2)))

    res.n_samples = n
    return res