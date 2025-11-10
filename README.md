# xArm6 Web Control Interface

A web-based control interface for the UFactory xArm6 robotic arm using FastAPI and vanilla JavaScript.

## Features

- **Real-time Status Monitoring**: View robot state, position, and joint angles
- **Motion Control**: Enable/disable motion, move to home position
- **Position Control**: Move to specific Cartesian coordinates
- **Joint Control**: Control individual joint angles
- **Gripper Control**: Open and close the gripper
- **Emergency Stop**: Immediate safety stop
- **Error Management**: Clear errors and warnings
- **2D Path Following**: Execute smooth spline paths using Mode 1 (Servoj mode)
  - Cubic, quadratic, and linear spline interpolation
  - Circular path following
  - 100-200 Hz streaming for smooth motion
  - Real-time path preview

## Installation

1. Install dependencies:
```bash
pip install -r requirements.txt
```

## Running the Application

1. Start the FastAPI server:
```bash
cd /home/mclovin/xarm
python main.py
```

2. Open your browser and navigate to:
```
http://localhost:8000
```

## Configuration

The xArm IP address is set in `main.py`:
```python
XARM_IP = "192.168.1.213"
```

Change this to match your xArm's IP address if different.

## API Endpoints

### Status
- `GET /api/status` - Get current robot status
- `POST /api/connect` - Connect to the robot
- `POST /api/disconnect` - Disconnect from the robot

### Motion
- `POST /api/motion/enable` - Enable robot motion
- `POST /api/motion/disable` - Disable robot motion
- `POST /api/move/home` - Move to home position
- `POST /api/stop` - Emergency stop

### Control
- `POST /api/move/position` - Move to Cartesian position
- `POST /api/move/joints` - Move to joint angles
- `POST /api/clear/errors` - Clear errors and warnings

### Gripper
- `POST /api/gripper/open` - Open gripper
- `POST /api/gripper/close` - Close gripper

### Path Following (Spline & Curves)
- `POST /api/path/spline/preview` - Preview spline path without moving robot
- `POST /api/path/spline/execute` - Execute spline path using Mode 1 (Servoj)
- `POST /api/path/circle/execute` - Execute circular path
- `POST /api/path/stop` - Stop current path execution
- `GET /api/path/status` - Get path execution status

## Path Following Usage

The xArm supports smooth 2D path following using **Mode 1 (Servoj mode)** which streams trajectory points at 100-200 Hz for continuous motion.

### Spline Path Example

```python
import requests

# Define control points for your path
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
    "spline_type": "cubic",  # "cubic", "quadratic", or "linear"
    "roll": 180,
    "pitch": 0,
    "yaw": 0
}

# Preview the path first (no robot movement)
preview = requests.post("http://localhost:8000/api/path/spline/preview", json=data)
print(preview.json())

# Execute the path (robot will move!)
response = requests.post("http://localhost:8000/api/path/spline/execute", json=data)
print(response.json())
```

### Circular Path Example

```python
# Execute a circular path
circle_data = {
    "center_x": 400,
    "center_y": 0,
    "radius": 50,
    "z_height": 300,
    "num_points": 100,
    "roll": 180,
    "pitch": 0,
    "yaw": 0
}

response = requests.post("http://localhost:8000/api/path/circle/execute", json=circle_data)
```

### Test Script

Run the included test script to verify the implementation:

```bash
cd /home/mclovin/xarm
python3 test_spline.py
```

## How It Works

The path following implementation uses:

1. **Spline Generation**: Uses scipy to generate smooth cubic/quadratic splines from control points
2. **Mode 1 (Servoj)**: Switches robot to servo mode for high-frequency position streaming
3. **100-200 Hz Streaming**: Sends position commands at recommended frequency for smooth motion
4. **Inverse Kinematics**: Automatically handled by xArm controller for each point
5. **Background Execution**: Runs in separate thread to avoid blocking the API

Based on xArm User Manual section 4.1 - Mode 1 (Servoj mode) recommendations.

## Safety

⚠️ **IMPORTANT**: Always ensure the robot workspace is clear before enabling motion or sending movement commands. Use the Emergency Stop button if needed.

## License

MIT
