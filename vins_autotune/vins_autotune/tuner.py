"""Coordinate-descent parameter tuner.

Evaluates one parameter at a time: tries a step up and a step down,
keeps the direction that improves the cost (ATE). Falls back gracefully
when no improvement is found.
"""
from __future__ import annotations

import math
import time
import logging
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from .config_editor import ConfigEditor, ParamSpec
from .metrics import MetricsResult

_logger = logging.getLogger("vins_autotune.tuner")


@dataclass
class TrialResult:
    param_key: str
    tried_value: float
    cost: float
    metrics: MetricsResult
    accepted: bool


@dataclass
class TunerState:
    # Current best cost (lower is better)
    best_cost: float = float("inf")
    # History of all trials
    history: List[TrialResult] = field(default_factory=list)
    # Per-parameter convergence flag
    converged: Dict[str, bool] = field(default_factory=dict)
    # Full pass counter
    pass_count: int = 0
    # Whether the tuner has a valid baseline
    initialized: bool = False


def default_cost(metrics: MetricsResult) -> float:
    """Weighted combination of ATE translation and rotation."""
    return metrics.ate_trans + 0.02 * metrics.ate_rot


class CoordinateDescentTuner:
    """One-at-a-time parameter optimizer.

    Workflow per pass:
      1. For each parameter in order:
         a. Try value + step → measure cost over eval_window seconds.
         b. Try value - step → measure cost.
         c. Keep the better value; revert if neither improves.
      2. Repeat until no parameter improves or max_passes reached.

    The `eval_fn` callback is called with a dict of param→value to apply,
    and must return a MetricsResult after running VINS for `eval_window` sec.
    """

    def __init__(
        self,
        config_editor: ConfigEditor,
        params: Dict[str, ParamSpec],
        eval_fn: Callable[[Dict[str, float]], MetricsResult],
        eval_window: float = 15.0,
        max_passes: int = 5,
        cost_fn: Callable[[MetricsResult], float] = default_cost,
        min_improvement: float = 0.01,
    ):
        self.editor = config_editor
        self.params = dict(params)
        self.eval_fn = eval_fn
        self.eval_window = eval_window
        self.max_passes = max_passes
        self.cost_fn = cost_fn
        self.min_improvement = min_improvement
        self.state = TunerState()
        for k in params:
            self.state.converged[k] = False

    def _current_values(self) -> Dict[str, float]:
        return {k: float(s.value) for k, s in self.params.items()}

    def _apply_and_eval(self, trial_values: Dict[str, float]) -> Tuple[float, MetricsResult]:
        self.editor.apply_param_specs(
            {k: ParamSpec(key=k, value=v, min=self.params[k].min,
                          max=self.params[k].max, step=self.params[k].step,
                          dtype=self.params[k].dtype)
             for k, v in trial_values.items()}
        )
        # Give the system time to restart / settle with new params
        time.sleep(2.0)
        metrics = self.eval_fn(trial_values)
        cost = self.cost_fn(metrics)
        return cost, metrics

    def run(self, on_trial: Optional[Callable[[TrialResult], None]] = None) -> Dict[str, float]:
        """Run the full tuning loop. Returns the best parameter values found."""
        # Baseline
        if not self.state.initialized:
            _logger.info("Tuner: establishing baseline with current config")
            base_vals = self._current_values()
            cost, metrics = self._apply_and_eval(base_vals)
            self.state.best_cost = cost
            self.state.initialized = True
            _logger.info("Baseline cost=%.4f (ATE_t=%.3f, ATE_r=%.2f, scale=%.3f)",
                         cost, metrics.ate_trans, metrics.ate_rot, metrics.scale_drift)

        for p in range(self.max_passes):
            self.state.pass_count = p
            _logger.info("=== Tuning pass %d/%d ===", p + 1, self.max_passes)
            improved_this_pass = False

            for key, spec in self.params.items():
                if self.state.converged.get(key, False):
                    continue

                current_val = float(spec.value)
                results: List[TrialResult] = []

                for direction in (+1, -1):
                    trial_val = current_val + direction * spec.step
                    trial_val = spec.clamp(trial_val)
                    if trial_val == current_val:
                        continue  # hit boundary

                    trial_vals = self._current_values()
                    trial_vals[key] = trial_val
                    _logger.info("Trying %s = %s (dir %+d)", key, trial_val, direction)
                    cost, metrics = self._apply_and_eval(trial_vals)
                    tr = TrialResult(key, trial_val, cost, metrics,
                                     accepted=cost < self.state.best_cost - self.min_improvement)
                    results.append(tr)
                    if on_trial:
                        on_trial(tr)

                    if tr.accepted:
                        # Accept this improvement
                        self.params[key].value = trial_val
                        self.state.best_cost = cost
                        improved_this_pass = True
                        _logger.info("ACCEPTED %s = %s → cost=%.4f", key, trial_val, cost)
                        break  # don't try the other direction
                    else:
                        _logger.info("Rejected %s = %s → cost=%.4f (best=%.4f)",
                                     key, trial_val, cost, self.state.best_cost)

                if not any(r.accepted for r in results):
                    # No improvement in either direction → reduce step / converge
                    self.params[key].step = max(spec.step * 0.5, spec.step * 0.1)
                    if self.params[key].step < abs(spec.step) * 0.1:
                        self.state.converged[key] = True
                        _logger.info("Parameter %s converged", key)

            if not improved_this_pass:
                _logger.info("No improvement in pass %d — stopping", p + 1)
                break

        best = self._current_values()
        _logger.info("Tuning complete. Best cost=%.4f. Final params: %s",
                     self.state.best_cost, best)
        # Ensure final config reflects best values
        self.editor.apply_param_specs(
            {k: ParamSpec(key=k, value=v, min=self.params[k].min,
                          max=self.params[k].max, step=self.params[k].step,
                          dtype=self.params[k].dtype)
             for k, v in best.items()}
        )
        return best