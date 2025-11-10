# xArm Spline Path Following - Quick Start Guide

Complete implementation of 2D spline path following for xArm using Mode 1 (Servoj mode).

## Installation

```bash
cd /home/mclovin/xarm
pip3 install -r requirements.txt
```

## 1. Visualize Spline Paths (Offline)

**Plot all example patterns:**
```bash
python3 plot_spline.py
```

**Plot specific pattern:**
```bash
python3 plot_spline.py --pattern wave
python3 plot_spline.py --pattern heart
python3 plot_spline.py --pattern circle
```

**Save to file:**
```bash
python3 plot_spline.py --pattern all --save my_paths.png
```

## 2. Start the Control Server

```bash
./start.sh
```

Server will start at `http://localhost:8000`

## 3. Test the API (No Robot Movement)

```bash
python3 test_spline.py
```

This tests:
- ✓ API connectivity
- ✓ Spline path preview generation
- ✗ Path execution (commented out for safety)

## 4. Execute Paths on Robot

### Python Example

```python
import requests

# Define your path
data = {
    "control_points": [
        {"x": 300, "y": 0},
        {"x": 350, "y": 50},
        {"x": 400, "y": 0},
        {"x": 450, "y": -50},
        {"x": 500, "y": 0}
    ],
    "z_height": 300,
    "num_samples": 200,
    "spline_type": "cubic",
    "roll": 180,
    "pitch": 0,
    "yaw": 0
}

# SAFE: Preview first (no movement)
preview = requests.post("http://localhost:8000/api/path/spline/preview", json=data)
print(f"Path has {preview.json()['num_points']} points")

# ROBOT WILL MOVE: Execute the path
response = requests.post("http://localhost:8000/api/path/spline/execute", json=data)
print(response.json())
```

### cURL Example

```bash
# Preview
curl -X POST http://localhost:8000/api/path/spline/preview \
  -H "Content-Type: application/json" \
  -d '{
    "control_points": [
      {"x": 300, "y": 0},
      {"x": 400, "y": 50},
      {"x": 500, "y": 0}
    ],
    "z_height": 300,
    "num_samples": 200,
    "spline_type": "cubic",
    "roll": 180,
    "pitch": 0,
    "yaw": 0
  }'

# Execute (robot moves!)
curl -X POST http://localhost:8000/api/path/spline/execute \
  -H "Content-Type: application/json" \
  -d '{...same data...}'
```

## 5. Available Patterns

All patterns are pre-defined in `plot_spline.py`:

| Pattern | Description | Control Points |
|---------|-------------|----------------|
| `wave` | Sine wave | 7 points |
| `s` | S-shape curve | 5 points |
| `heart` | Heart shape | 9 points |
| `spiral` | Outward spiral | 8 points |
| `square` | Rounded square | 5 points |
| `circle` | Perfect circle | Parametric |

## 6. API Endpoints

### Path Execution
- `POST /api/path/spline/preview` - Preview path (no movement)
- `POST /api/path/spline/execute` - Execute spline path
- `POST /api/path/circle/execute` - Execute circular path
- `POST /api/path/stop` - Stop current path
- `GET /api/path/status` - Check execution status

### Robot Control
- `GET /api/status` - Robot status
- `POST /api/motion/enable` - Enable motors
- `POST /api/motion/disable` - Disable motors
- `POST /api/stop` - Emergency stop

## 7. Workflow

```
1. Design path → plot_spline.py
2. Visualize   → Check plot looks good
3. Preview     → POST /api/path/spline/preview
4. Enable      → POST /api/motion/enable
5. Execute     → POST /api/path/spline/execute
6. Monitor     → GET /api/path/status
```

## 8. Safety Checklist

Before executing any path on the robot:

- [ ] Workspace is clear
- [ ] Robot is properly mounted
- [ ] Emergency stop is accessible
- [ ] Path previewed and looks reasonable
- [ ] Z height is safe (200-400mm recommended)
- [ ] X/Y coordinates are within workspace
- [ ] Path avoids singularities (base center)
- [ ] Control box is connected and powered
- [ ] Robot is enabled

## 9. Parameters Explained

```python
{
    "control_points": [{"x": 300, "y": 0}, ...],  # Path waypoints (mm)
    "z_height": 300,              # Constant Z for 2D path (mm)
    "num_samples": 200,           # Points along path (100-200 recommended)
    "spline_type": "cubic",       # "cubic", "quadratic", or "linear"
    "roll": 180,                  # End effector pointing down (degrees)
    "pitch": 0,                   # No tilt (degrees)
    "yaw": 0                      # No rotation (degrees)
}
```

## 10. Troubleshooting

### Server won't start
- Check xArm is powered and connected
- Verify IP address in `main.py` line 14
- Ping the xArm: `ping 192.168.1.213`

### Path execution fails
- Robot not enabled: `POST /api/motion/enable`
- Points unreachable: Check workspace limits
- Singularity: Avoid base center area

### Plot doesn't show
```bash
export MPLBACKEND=TkAgg
python3 plot_spline.py
```

### Jerky motion
- Increase `num_samples` (200+)
- Use cubic spline instead of linear
- Check network latency

## 11. Files Reference

| File | Purpose |
|------|---------|
| `main.py` | FastAPI server with spline API |
| `plot_spline.py` | Visualization tool |
| `test_spline.py` | API test script |
| `start.sh` | Server startup script |
| `requirements.txt` | Python dependencies |
| `README.md` | Full documentation |
| `SPLINE_EXAMPLES.md` | Usage examples |
| `PLOTTING_GUIDE.md` | Plotting tutorial |
| `QUICKSTART.md` | This file |

## 12. Example Session

```bash
# Terminal 1: Start server
cd /home/mclovin/xarm
./start.sh

# Terminal 2: Visualize paths
python3 plot_spline.py --pattern wave

# Terminal 3: Test API
python3 test_spline.py

# Terminal 3: Execute custom path
python3 << EOF
import requests

data = {
    "control_points": [
        {"x": 300, "y": 0},
        {"x": 400, "y": 50},
        {"x": 500, "y": 0}
    ],
    "z_height": 300,
    "num_samples": 200,
    "spline_type": "cubic",
    "roll": 180,
    "pitch": 0,
    "yaw": 0
}

# Preview
preview = requests.post("http://localhost:8000/api/path/spline/preview", json=data)
print(preview.json())

# Enable robot
requests.post("http://localhost:8000/api/motion/enable")

# Execute (robot moves!)
response = requests.post("http://localhost:8000/api/path/spline/execute", json=data)
print(response.json())
EOF
```

## 13. Technical Details

**Based on xArm User Manual Section 4.1:**
- Uses **Mode 1 (Servoj mode)** for smooth path following
- Streams at **100-200 Hz** (recommended frequency)
- **Inverse kinematics** solved automatically by xArm
- **No buffering** - only latest command executed
- Returns to **Mode 0** when complete

**Spline Generation:**
- Uses scipy `CubicSpline` and `interp1d`
- Natural boundary conditions for smooth endpoints
- Parametric interpolation for even point distribution

## 14. Next Steps

1. Read full docs: `README.md`, `SPLINE_EXAMPLES.md`
2. Experiment with plotting tool
3. Test with preview API (safe)
4. Execute simple paths on robot
5. Design custom patterns for your application

## Support

- GitHub Issues: Report bugs/features
- Manual: xArm User Manual v2.0.0 section 4.1
- SDK Docs: xArm Python SDK documentation

---

**Created:** 2025-11-06
**xArm Python SDK:** v1.17.0
**Implementation:** `/home/mclovin/xarm/`
