"""Time-synchronized trajectory buffer for VINS vs ground truth comparison."""
from __future__ import annotations

import threading
from collections import deque
from typing import Deque, List, Tuple, Optional

import numpy as np

from .metrics import PoseSample


class TrajectoryBuffer:
    """Thread-safe buffer that stores VINS and ground-truth poses and
    provides time-synchronized pairs within a sliding window.

    The window is defined in seconds. Older samples are evicted automatically.
    """

    def __init__(self, window_sec: float = 30.0, max_sync_dt: float = 0.05):
        self._lock = threading.Lock()
        self._vins: Deque[PoseSample] = deque()
        self._gt: Deque[PoseSample] = deque()
        self._window_sec = window_sec
        self._max_sync_dt = max_sync_dt

    def add_vins(self, sample: PoseSample) -> None:
        with self._lock:
            self._vins.append(sample)
            self._evict(self._vins, sample.timestamp)

    def add_gt(self, sample: PoseSample) -> None:
        with self._lock:
            self._gt.append(sample)
            self._evict(self._gt, sample.timestamp)

    def _evict(self, buf: Deque[PoseSample], cur_time: float) -> None:
        threshold = cur_time - self._window_sec
        while buf and buf[0].timestamp < threshold:
            buf.popleft()

    def get_synchronized_pairs(self) -> Tuple[List[PoseSample], List[PoseSample]]:
        """Return time-matched (vins, gt) pose lists.

        For each VINS sample, finds the closest GT sample within max_sync_dt.
        """
        with self._lock:
            vins_list = list(self._vins)
            gt_list = list(self._gt)

        if not vins_list or not gt_list:
            return [], []

        gt_times = np.array([s.timestamp for s in gt_list])
        matched_vins: List[PoseSample] = []
        matched_gt: List[PoseSample] = []

        for v in vins_list:
            idx = int(np.argmin(np.abs(gt_times - v.timestamp)))
            dt = abs(gt_times[idx] - v.timestamp)
            if dt <= self._max_sync_dt:
                matched_vins.append(v)
                matched_gt.append(gt_list[idx])

        return matched_vins, matched_gt

    def get_all(self) -> Tuple[List[PoseSample], List[PoseSample]]:
        with self._lock:
            return list(self._vins), list(self._gt)

    def clear(self) -> None:
        with self._lock:
            self._vins.clear()
            self._gt.clear()

    @property
    def vins_count(self) -> int:
        with self._lock:
            return len(self._vins)

    @property
    def gt_count(self) -> int:
        with self._lock:
            return len(self._gt)