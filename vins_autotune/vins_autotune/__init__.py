"""vins_autotune: Online/offline parameter auto-tuning for VINS-Mono.

Compares VINS odometry against ground truth and optimizes tracker/estimator
parameters using coordinate descent on ATE/RPE metrics.
"""
__version__ = "0.1.0"