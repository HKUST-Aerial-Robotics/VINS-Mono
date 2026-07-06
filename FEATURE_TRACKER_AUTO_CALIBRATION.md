# Feature Tracker Auto-Calibration Guide

## Overview

The feature tracker auto-calibration system automatically optimizes detector parameters based on camera resolution and field-of-view (FOV). This ensures robust feature detection and tracking across different camera types without manual tuning.

## How It Works

### Calibration Logic

The system calculates three key parameters at startup:

#### 1. Maximum Feature Count (`max_cnt`)
```
Formula: max_cnt = max(80, image_area / 1500)
Rationale: Target ~1500 pixels per feature for good tracking stability
```

**Examples:**
- Thermal (384×288 = 110,592 px): max_cnt = max(80, 74) = **80 features**
- VGA (640×480 = 307,200 px): max_cnt = max(80, 205) = **205 features**

#### 2. Minimum Distance Between Features (`min_dist`)
```
Formula: min_dist = max(15, sqrt(image_area / max_cnt * 0.8))
Rationale: Ensure uniform grid distribution with 80% fill factor
```

**Examples:**
- Thermal: min_dist = max(15, sqrt(1380)) = max(15, **37**) = 37 px
- VGA: min_dist = max(15, sqrt(1199)) = max(15, **35**) = 35 px

#### 3. AGAST Corner Detection Threshold (`fast_threshold`)
```
Formula: fast_threshold = 25 + (image_area / 200000)
Rationale: Adapt sensitivity to resolution (lower = more sensitive)
```

**Examples:**
- Thermal: fast_threshold = 25 + 0.55 = **25.6 ≈ 26**
- VGA: fast_threshold = 25 + 1.54 = **26.5 ≈ 27**

## Configuration

### Enable/Disable Auto-Calibration

```yaml
# In config file (thermal_cam_config.yaml or usb_cam_config.yaml)

# Enable auto-calibration (overrides manual max_cnt, min_dist, fast_threshold)
auto_calibrate_tracker: 1    # 1 = enabled, 0 = disabled

# Print statistics to terminal
calibrate_print_stats: 1     # 1 = print, 0 = silent
```

### Manual Configuration (When `auto_calibrate_tracker: 0`)

```yaml
max_cnt: 150                 # Keep manual values
min_dist: 25
fast_threshold: 30
```

## Console Output

### Initialization Output

When the feature tracker initializes with auto-calibration enabled:

```
[INFO] === Feature Tracker Auto-Calibration ===
[INFO] Image resolution: 384x288 (110592 pixels)
[INFO] Manual config: max_cnt=180, min_dist=20, fast_threshold=30
[INFO] Optimal config: max_cnt=96, min_dist=22, fast_threshold=26
[WARN] Applied auto-calibrated feature tracker parameters!
```

### Real-time Statistics

Every frame (if `calibrate_print_stats: 1`):

```
[INFO] [TRACKER STATS] Tracked: 82 | New: 18 | Total: 100/96 (104.2%) | Time: 12.3ms
```

**Fields:**
- `Tracked`: Features with age > 1 frame (good for tracking)
- `New`: Features detected in current frame (age = 1)
- `Total/Max`: Current features vs. maximum capacity
- `(%)`: Fill ratio - should stay 70-100% for optimal performance
- `Time`: Feature tracking processing time

## Use Cases

### Case 1: Thermal Camera (384×288)
```
Application: UAV with thermal imaging
Config: auto_calibrate_tracker: 1

Results:
- Auto-calibrated: max_cnt=96, min_dist=22, fast_threshold=26
- Benefit: Adapted for low-contrast thermal imagery
- Tracking quality: 80-100 stable features per frame
```

### Case 2: Standard USB Camera (640×480)
```
Application: Desktop vision system
Config: auto_calibrate_tracker: 1

Results:
- Auto-calibrated: max_cnt=205, min_dist=35, fast_threshold=27
- Benefit: More features for higher resolution
- Tracking quality: 150-200 stable features per frame
```

### Case 3: Override with Manual Tuning
```yaml
# Want specific parameters for your use case
auto_calibrate_tracker: 0    # Disable auto-calibration
max_cnt: 120                 # Custom values
min_dist: 28
fast_threshold: 35
```

## Performance Monitoring

### Healthy Feature Distribution

```
[TRACKER STATS] Tracked: 85 | New: 15 | Total: 100/100 (100.0%) | Time: 10.2ms
```
✅ Good - 85% of features are tracked, 15% are new

### Under-Featured

```
[TRACKER STATS] Tracked: 45 | New: 5 | Total: 50/100 (50.0%) | Time: 5.1ms
```
⚠️ Warning - Less than 70% feature capacity being used

**Solutions:**
- Increase `max_cnt` or decrease `min_dist`
- Lower `fast_threshold` to detect weaker corners
- Check if image is too dark (enable CLAHE with `equalize: 1`)

### Over-Featured

```
[TRACKER STATS] Tracked: 120 | New: 30 | Total: 150/100 (150.0%) | Time: 25.3ms
```
⚠️ Warning - More features than capacity (tracking will skip features)

**Solutions:**
- Decrease `max_cnt` or increase `min_dist`
- Increase `fast_threshold` to select only strong corners
- Reduce `AGAST` iterations if processing time exceeds 20ms

## Technical Details

### Grid-Based Feature Selection

Features are selected using a grid-based approach:
1. Image divided into `min_dist × min_dist` cells
2. Strongest corner in each cell is selected
3. Result: Uniform feature distribution (no clustering)

### Inverse Depth Parameterization

Features use inverse depth for triangulation:
- Closer features = larger inverse depth values
- Scale correction adjusts all inverse depths proportionally
- Prevents scale ambiguity in monocular VIO

## Troubleshooting

### Issue: "Feature detection too slow"

```
[TRACKER STATS] ... | Time: 45.2ms
```

**Solution:**
- Reduce `AGAST` iterations (in parameters.cpp)
- Disable `use_bidirectional_flow: 0`
- Lower `max_cnt` further

### Issue: "Not enough features for tracking"

```
[TRACKER STATS] Tracked: 25 | New: 10 | Total: 35/100 (35.0%)
```

**Solution:**
- Enable histogram equalization: `equalize: 1`
- Lower `fast_threshold` by 5-10
- Increase `max_cnt`

### Issue: "Features jumping around"

**Solution:**
- Enable `use_bidirectional_flow: 1` for consistency check
- Increase `F_threshold` for stricter outlier rejection
- Check camera calibration accuracy

## References

- **AGAST** - Accelerated Segment Test detector (Mair et al.)
- **Lucas-Kanade** Optical Flow - Feature tracking algorithm
- **Grid-based selection** - Uniform feature distribution
- **Inverse depth** - Monocular VIO parameterization

## Related Parameters

- **IMU Filtering**: `enable_imu_acc_filter`, `imu_acc_filter_alpha`
- **Scale Correction**: `enable_altitude_scale_correction`
- **ZUPT**: `enable_zupt`, `zupt_vel_threshold`
- **CLAHE**: `equalize` (histogram equalization)
