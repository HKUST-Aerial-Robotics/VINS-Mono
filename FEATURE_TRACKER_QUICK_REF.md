# Feature Tracker Auto-Calibration - Quick Reference

## Enable/Disable

```yaml
# config/thermal_cam_config.yaml
auto_calibrate_tracker: 1        # Auto-calibrate (1) or manual (0)
calibrate_print_stats: 1         # Print stats to terminal
```

## What Gets Calibrated

| Parameter | Manual | Auto (Thermal 384×288) | Auto (VGA 640×480) |
|-----------|--------|------------------------|-------------------|
| `max_cnt` | 180 | 96 | 205 |
| `min_dist` | 20 | 22 | 35 |
| `fast_threshold` | 30 | 26 | 27 |

## Terminal Output

### At Startup
```
[INFO] === Feature Tracker Auto-Calibration ===
[INFO] Image resolution: 384x288 (110592 pixels)
[INFO] Manual config: max_cnt=180, min_dist=20, fast_threshold=30
[INFO] Optimal config: max_cnt=96, min_dist=22, fast_threshold=26
[WARN] Applied auto-calibrated feature tracker parameters!
```

### Every Frame
```
[INFO] [TRACKER STATS] Tracked: 82 | New: 18 | Total: 100/96 (104.2%) | Time: 12.3ms
       └─ Tracked features  └─ New features  └─ Current/Max  └─ Fill%  └─ ms
```

## Target Statistics

| Metric | Target | Range |
|--------|--------|-------|
| Fill Ratio | 80-100% | 70-120% |
| Tracked Ratio | 70-85% | 50-90% |
| Processing Time | < 15ms | < 20ms |

## Auto-Calibration Formulas

```
max_cnt = max(80, image_pixels / 1500)
min_dist = max(15, sqrt(image_pixels / max_cnt * 0.8))
fast_threshold = 25 + (image_pixels / 200000)
```

## Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| Too few features (< 50%) | Detector too strict | Lower `fast_threshold` |
| Too many features (> 120%) | Detector too loose | Increase `fast_threshold` |
| Processing slow (> 20ms) | Too many features | Increase `min_dist` |
| Dark image, no features | Low contrast | Enable `equalize: 1` |
| Features jumping | Outliers in tracking | Enable `use_bidirectional_flow: 1` |

## Files Modified

- `feature_tracker/src/parameters.h/cpp` - Auto-calibration logic
- `feature_tracker/src/feature_tracker_node.cpp` - Statistics output
- `config/thermal_cam_config.yaml` - Configuration
- `config/usb_cam_config.yaml` - Configuration
- `CHANGELOG.md` - Documentation

## Quick Test

```bash
# 1. Enable auto-calibration in config file
auto_calibrate_tracker: 1
calibrate_print_stats: 1

# 2. Build
cd ~/catkin_ws && catkin build

# 3. Run and check output
roslaunch vins_estimator vins_rviz.launch

# 4. Look for [TRACKER STATS] lines in terminal
# Should show: Tracked: X | New: Y | Total: Z/MAX (%)
```

---

For details, see: `FEATURE_TRACKER_AUTO_CALIBRATION.md`
