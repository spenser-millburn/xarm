# xArm Spline Path Following Examples

This document provides practical examples for using the 2D spline path following feature.

## Overview

The implementation uses **xArm Mode 1 (Servoj mode)** to stream trajectory points at 100-200 Hz, enabling smooth path following for arbitrary 2D curves. This is based on the xArm User Manual section 4.1 recommendations.

## Quick Start

### 1. Start the Server

```bash
cd /home/mclovin/xarm
./start.sh
```

### 2. Run the Test Script

```bash
python3 test_spline.py
```

## Path Types

### Cubic Spline (Recommended for Smooth Curves)

Best for smooth, flowing paths with natural curvature.

```python
import requests

data = {
    "control_points": [
        {"x": 300, "y": -50},
        {"x": 350, "y": 50},
        {"x": 400, "y": -50},
        {"x": 450, "y": 50},
        {"x": 500, "y": -50}
    ],
    "z_height": 250,
    "num_samples": 200,
    "spline_type": "cubic",
    "roll": 180,
    "pitch": 0,
    "yaw": 0
}

# Preview first
preview = requests.post("http://localhost:8000/api/path/spline/preview", json=data)
print(f"Path has {preview.json()['num_points']} points")

# Execute
requests.post("http://localhost:8000/api/path/spline/execute", json=data)
```

### Quadratic Spline

For simpler curves with less computational overhead.

```python
data = {
    "control_points": [
        {"x": 300, "y": 0},
        {"x": 400, "y": 100},
        {"x": 500, "y": 0}
    ],
    "z_height": 250,
    "num_samples": 150,
    "spline_type": "quadratic",
    "roll": 180,
    "pitch": 0,
    "yaw": 0
}

requests.post("http://localhost:8000/api/path/spline/execute", json=data)
```

### Linear Path

For straight line segments between points.

```python
data = {
    "control_points": [
        {"x": 300, "y": 0},
        {"x": 400, "y": 100},
        {"x": 500, "y": 0},
        {"x": 600, "y": 100}
    ],
    "z_height": 250,
    "num_samples": 100,
    "spline_type": "linear",
    "roll": 180,
    "pitch": 0,
    "yaw": 0
}

requests.post("http://localhost:8000/api/path/spline/execute", json=data)
```

### Circle Path

Perfect circles using parametric equations.

```python
circle_data = {
    "center_x": 400,
    "center_y": 0,
    "radius": 75,
    "z_height": 250,
    "num_points": 150,
    "roll": 180,
    "pitch": 0,
    "yaw": 0
}

requests.post("http://localhost:8000/api/path/circle/execute", json=circle_data)
```

## Advanced Usage

### Drawing Letters or Shapes

Create custom patterns by defining appropriate control points:

```python
# Draw an "S" shape
s_shape = {
    "control_points": [
        {"x": 300, "y": 100},
        {"x": 350, "y": 50},
        {"x": 400, "y": 0},
        {"x": 450, "y": -50},
        {"x": 500, "y": -100}
    ],
    "z_height": 250,
    "num_samples": 200,
    "spline_type": "cubic",
    "roll": 180,
    "pitch": 0,
    "yaw": 0
}

requests.post("http://localhost:8000/api/path/spline/execute", json=s_shape)
```

### Monitoring Execution

```python
import time

# Start path
response = requests.post("http://localhost:8000/api/path/spline/execute", json=data)
result = response.json()

print(f"Started path with {result['num_points']} points")
print(f"Estimated duration: {result['estimated_duration_sec']:.2f} seconds")

# Monitor progress
while True:
    status = requests.get("http://localhost:8000/api/path/status").json()
    if not status['is_executing']:
        print("Path completed!")
        break
    time.sleep(0.5)
```

### Emergency Stop

```python
# Stop path execution immediately
requests.post("http://localhost:8000/api/path/stop")
```

## Parameter Guidelines

Based on xArm manual recommendations:

- **num_samples**: 100-200 recommended for smooth motion
  - Higher = smoother but longer execution time
  - Lower = faster but more jerky

- **z_height**: Keep constant for 2D paths
  - Recommended: 200-400mm depending on workspace

- **roll/pitch/yaw**: End effector orientation
  - Default `roll=180, pitch=0, yaw=0` = pointing straight down

- **Frequency**: Automatically calculated (100-200 Hz)
  - Manual recommendation: 30-250 Hz range
  - Below 30 Hz = jerky motion
  - Above 250 Hz = commands may be dropped

## Technical Details

### How Mode 1 Works

1. Robot switches to **Mode 1 (Servoj mode)**
2. Streams position commands at high frequency (100-200 Hz)
3. Each command: `set_position_aa([x, y, z, roll, pitch, yaw])`
4. **No buffering** - only latest command is executed
5. Returns to **Mode 0** when complete

### Inverse Kinematics

- **Automatically handled** by xArm controller
- Each Cartesian point (x,y,z,r,p,y) converted to joint angles
- **Possible issues**:
  - Unreachable positions (no IK solution)
  - Singularities (very high joint speeds)
  - Joint limit violations

### Safety Considerations

- Keep points close together to avoid sudden movements
- Stay away from workspace boundaries
- Avoid base center area (singularity zone)
- Test with preview first
- Have emergency stop ready

## Troubleshooting

### Path execution returns error codes

- Check if points are reachable
- Verify Z height is safe
- Ensure robot is in correct mode

### Jerky motion

- Increase `num_samples` for more points
- Check network latency
- Verify control box connection

### Robot stops mid-path

- Check for singularities
- Verify no collision detection triggered
- Ensure payload is set correctly

## API Reference

### POST /api/path/spline/preview
Preview spline without moving robot.

**Request:**
```json
{
  "control_points": [{"x": 300, "y": 0}, ...],
  "z_height": 250,
  "num_samples": 200,
  "spline_type": "cubic",
  "roll": 180,
  "pitch": 0,
  "yaw": 0
}
```

**Response:**
```json
{
  "status": "preview_generated",
  "num_points": 200,
  "path_points": [...],
  "control_points": [...]
}
```

### POST /api/path/spline/execute
Execute spline path (robot moves!).

Returns immediately, path runs in background.

### POST /api/path/circle/execute
Execute circular path.

**Request:**
```json
{
  "center_x": 400,
  "center_y": 0,
  "radius": 50,
  "z_height": 250,
  "num_points": 100,
  "roll": 180,
  "pitch": 0,
  "yaw": 0
}
```

### GET /api/path/status
Check if path is currently executing.

**Response:**
```json
{
  "is_executing": true
}
```

### POST /api/path/stop
Stop current path execution immediately.

## References

- xArm User Manual v2.0.0 - Section 4.1 (Motion Modes)
- xArm Python SDK Documentation
- Implementation: `/home/mclovin/xarm/main.py` (lines 102-202)
