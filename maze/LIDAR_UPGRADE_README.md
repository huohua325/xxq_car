# LIDAR Data Format Upgrade Guide

## 📊 Problem Summary

### Current Status
- **LIDAR raw data**: 360 points (one per degree)
- **Transmitted data**: 8 sector statistics (min/avg only)
- **Information loss**: **97.8%** ❌
- **Not suitable for**: Precise mapping, path planning, detail detection

### New Solution
- **Transmission format**: CSV point cloud
- **Transmitted data**: 360 raw points (angle, distance)
- **Information retention**: **100%** ✅
- **Data volume**: ~3.5KB per scan
- **Transmission time**: ~250ms @115200 baud

## 📦 Modified Files

1. **`maze/simple_lidar.py`** - ✅ Upgraded to support both old (8-sector) and new (360-point) formats
2. **`xxq/Core/Src/hardware/radar.h`** - ✅ Added `RadarPointCloud_t` structure and new function declarations
3. **`xxq/Core/Src/hardware/radar.c`** - ✅ Added `Radar_BuildPointCloud` and `Radar_SendPointCloudCSV` functions

## 🚀 Quick Start

### Option A: Python-Side Upgrade Only (Recommended for Quick Testing)

**Step 1**: Use the new wrapper in your test code

```python
# Replace
from maze.simple_lidar import SimpleLidarWrapper

# With
from maze.simple_lidar_v2 import SimpleLidarWrapperV2

# Rest of code remains the same
lidar = SimpleLidarWrapperV2(robot)
scan = lidar.request_scan(timeout=6.0)

# New feature: Get complete point cloud
if lidar.format_type == 'pointcloud':
    cloud = lidar.get_point_cloud()  # [(angle, dist), ...]
```

**Pros**:
- ✅ No firmware modification needed
- ✅ Backward compatible (auto-supports old JSON format)
- ✅ Ready to use immediately
- ❌ Still only 8 sector data (need firmware upgrade for 360 points)

### Option B: Complete Upgrade (Firmware + Python)

**Step 1**: Modify Python side

Reference `maze/lidar_upgrade_patch.py`, add to `ble_robot_control.py`:

1. Add `LidarPointCloud` class
2. Add `_parse_lidar_csv` method  
3. Modify `_parse_csv` method, add `LIDAR` check

**Step 2**: Modify firmware side

Reference `xxq/Core/Src/hardware/lidar_csv_format.c`:

1. Add point cloud data structures to `radar.h`
2. Add `Radar_SendPointCloudCSV` function to `radar.c`
3. Implement `Radar_BuildPointCloud` (adapt to your LIDAR model)
4. Modify 'A' command handler in `main.c`, call new function

**Step 3**: Compile and flash firmware

```bash
# Using STM32CubeIDE
# Project → Build Project
# Run → Debug or Flash
```

**Step 4**: Test

```bash
python maze/simple_lidar_v2.py
```

**Expected output**:
```
✅ Scan successful, received 360 points
   Data format: pointcloud
First 5 points:
  Point 1: Angle=  0°, Distance=1.20m
  Point 2: Angle=  1°, Distance=1.35m
  Point 3: Angle=  2°, Distance=1.15m
  ...
```

## 📐 Data Format Specification

### CSV Format (New)
```
LIDAR,timestamp,num_points,angle1,dist1,angle2,dist2,...\n
```

**Example**:
```
LIDAR,12345,360,0,1.20,1,1.35,2,1.15,3,1.30,...,359,2.50\n
```

**Fields**:
- `LIDAR`: Data type identifier
- `timestamp`: Milliseconds since startup
- `num_points`: Number of points (typically 360)
- `angle,dist`: Repeated pairs (angle in degrees, distance in meters)

### JSON Format (Old, Compatible)
```json
{
  "type": "LIDAR",
  "timestamp": 12345,
  "data": {
    "total_points": 300,
    "angle_coverage": 360.0,
    "sectors": [
      {"sector_id": 0, "angle_center": 0, "count": 45, "min_dist": 0.15, "avg_dist": 0.87},
      ...
    ]
  }
}
```

## 🎯 Performance Comparison

| Item | Old Format (JSON) | New Format (CSV) |
|------|-------------------|------------------|
| Data points | 8 | 360 |
| Angular resolution | 45° | 1° |
| Transmission size | ~500 bytes | ~3500 bytes |
| Transmission time | ~50ms | ~250ms |
| Information retention | 2.2% | 100% |
| Use case | Simple obstacle avoidance | SLAM mapping |

## 💡 Usage Recommendations

### Scenario 1: Stage 2 Exam Delivery
**Recommendation**: Continue using old format (8 sectors)
- Already sufficient for basic functionality
- No firmware modification needed
- Stable and reliable

### Scenario 2: SLAM Exploration Development
**Recommendation**: Upgrade to new format (360 points)
- Precise mapping requires high-resolution data
- Can detect small obstacles
- Supports more complex navigation algorithms

### Scenario 3: Rapid Prototyping
**Recommendation**: Use `simple_lidar_v2.py`
- Automatically compatible with both formats
- Works before and after firmware upgrade
- Progressive migration

## 📝 Important Notes

1. **LIDAR Model Dependency**: `Radar_BuildPointCloud` function needs adaptation to actual LIDAR data format
2. **Increased Data Volume**: CSV format data ~3.5KB, ensure UART buffer is large enough
3. **Transmission Time**: Scan + transmission takes ~2-3 seconds, plan accordingly
4. **Backward Compatibility**: Python code supports both new and old formats for safe upgrade

## 🔧 Troubleshooting

### Issue 1: Python Cannot Parse New Format
**Symptoms**: Scan timeout or parse failure
**Solution**:
1. Check if `_parse_lidar_csv` method added to `ble_robot_control.py`
2. Confirm firmware sends CSV format starting with `LIDAR,`
3. Use serial debugging tool to view raw data

### Issue 2: Firmware Sends Incomplete Data
**Symptoms**: Python receives < 360 points
**Solution**:
1. Increase UART send timeout (default 500ms may not be enough)
2. Check `csv_buffer` size (recommended 4096 bytes)
3. Use DMA transmission (refer to optimized version)

### Issue 3: Firmware Compilation Error
**Symptoms**: `RadarPointCloud_t` undefined
**Solution**:
1. Ensure data structure definitions added to `radar.h`
2. Check if `#include "radar.h"` in `main.c`
3. Regenerate project (Project → Clean)

## 📊 Data Flow Diagram

```
┌─────────────┐   360°      ┌──────────────┐   CSV        ┌──────────────┐
│   LIDAR     │  ────────>  │  STM32       │  ────────>   │  Python      │
│   Hardware  │  Raw Data   │  Firmware    │  3D Points   │  simple_     │
│             │             │  (radar.c)   │  SLAM Map    │  lidar_v2.py │
└─────────────┘             └──────────────┘              └──────────────┘
     360 points          RadarPointCloud_t              360 dict objects
     @ 1° resolution     → CSV format                   {'angle': 0, 
                         "LIDAR,ts,360,                  'distance': 1.2}
                          0,1.2,1,1.3,..."
```

## ✅ Acceptance Criteria

**New format working correctly indicators**:
1. ✅ `lidar.format_type == 'pointcloud'`
2. ✅ `len(scan) >= 300` (at least 300 valid points)
3. ✅ Angle range: 0-359 degrees
4. ✅ Distance reasonable: 0.05-8.0 meters
5. ✅ Visualization shows detailed information

## 📚 File Structure

```
maze/
├── simple_lidar.py              # Original wrapper (8 sectors)
├── simple_lidar_v2.py           # NEW: Upgraded wrapper (360 points)
├── lidar_upgrade_patch.py       # NEW: Python patch code
└── LIDAR_UPGRADE_README.md      # This file

xxq/Core/Src/hardware/
├── radar.h                       # Add: RadarPointCloud_t
├── radar.c                       # Add: Radar_SendPointCloudCSV
└── lidar_csv_format.c            # NEW: Reference implementation
```

## 🔄 Migration Steps

### Phase 1: Preparation (No code changes)
1. Read this README
2. Review `simple_lidar_v2.py`
3. Understand CSV format

### Phase 2: Python Upgrade (Safe to do now)
1. Use `simple_lidar_v2.py` in test code
2. Test with current firmware (will use old format)
3. Verify backward compatibility

### Phase 3: Firmware Upgrade (Optional, for 360 points)
1. Add data structures to `radar.h`
2. Implement `Radar_SendPointCloudCSV` in `radar.c`
3. Adapt `Radar_BuildPointCloud` to your LIDAR
4. Test with `python maze/simple_lidar_v2.py`

### Phase 4: Verification
1. Check `format_type == 'pointcloud'`
2. Verify 360 points received
3. Test SLAM integration

## 📞 Support

**If you encounter issues**:
1. Check file names are all in English (no Chinese characters)
2. Review code comments in `lidar_csv_format.c`
3. Test with serial monitor to see raw data
4. Compare with working `simple_lidar.py` (old format)

---

**Created**: 2025-01-14  
**Version**: 1.0  
**Status**: Ready for implementation  
**Files use English names**: ✅

