# Circle Path Configuration Guide

## Quick Start

```bash
python3 run_real_path.py
# Choose option 2: Circle path
```

## Parameters

### Radius (mm)
- **Default**: 40mm
- **Recommended range**: 20-100mm
- **Small radius** (20-40mm): Tight circle, good for testing
- **Medium radius** (40-80mm): Standard circular motion
- **Large radius** (80-150mm): Wide sweeping circle

### Center Position (X, Y)
- **Default**: X=400, Y=0
- **Safe workspace**:
  - X: 250-550mm
  - Y: -150 to +150mm
- **Center should be reachable** from all points on circle

### Z Height (mm)
- **Default**: 300mm
- **Recommended**: 250-400mm
- **Lower** (200-250mm): Closer to table (careful!)
- **Higher** (350-450mm): Safer for testing

### Number of Points
- **Default**: 100 points
- **Low** (50-80): Faster, less smooth, angular
- **Medium** (100-150): Good balance
- **High** (150-300): Very smooth, slower

## Point Spacing Calculator

The script automatically calculates:
- **Circumference** = 2π × radius
- **Point spacing** = circumference / num_points

Example:
- Radius: 50mm
- Circumference: 314mm
- Points: 100
- Spacing: 3.1mm between points

## Example Configurations

### 1. Small Test Circle
```
Radius: 30mm
Center: X=400, Y=0
Z height: 350mm
Points: 80
```
Perfect for first test - small, safe, quick.

### 2. Medium Circle
```
Radius: 60mm
Center: X=400, Y=0
Z height: 300mm
Points: 120
```
Good general purpose circle.

### 3. Large Smooth Circle
```
Radius: 100mm
Center: X=400, Y=0
Z height: 300mm
Points: 200
```
Large, very smooth motion.

### 4. Off-Center Circle
```
Radius: 50mm
Center: X=450, Y=50
Z height: 280mm
Points: 100
```
Circle not centered on workspace.

## Execution Time

Execution time ≈ num_points / frequency_hz

- 100 points @ 100 Hz = 1.0 second
- 150 points @ 100 Hz = 1.5 seconds
- 200 points @ 100 Hz = 2.0 seconds

## Workspace Safety

Make sure entire circle fits in workspace:

```
Min X = center_x - radius  (should be > 200mm)
Max X = center_x + radius  (should be < 650mm)
Min Y = center_y - radius  (should be > -250mm)
Max Y = center_y + radius  (should be < 250mm)
```

Example check for radius=50, center=(400,0):
- Min X = 400 - 50 = 350mm ✓
- Max X = 400 + 50 = 450mm ✓
- Min Y = 0 - 50 = -50mm ✓
- Max Y = 0 + 50 = 50mm ✓

## Python API Example

```python
import requests

circle_data = {
    "center_x": 400,
    "center_y": 0,
    "radius": 75,
    "z_height": 300,
    "num_points": 150,
    "roll": 180,
    "pitch": 0,
    "yaw": 0
}

# Execute
response = requests.post("http://localhost:8000/api/path/circle/execute",
                        json=circle_data)
print(response.json())
```

## Visualization

Before executing, visualize with plotting tool:

```bash
python3 plot_spline.py --pattern circle
```

Or create custom circle plot:

```python
from plot_spline import plot_circle_path
import matplotlib.pyplot as plt

plot_circle_path(center_x=400, center_y=0, radius=75,
                 num_points=150, title="My Circle")
plt.show()
```

## Tips

1. **Start small**: Test with radius=30mm first
2. **Check clearance**: Ensure nothing in path
3. **Monitor closely**: Watch robot during execution
4. **Increase gradually**: Go from small to large radius
5. **Check logs**: Server terminal shows progress

## Common Uses

- **Calibration**: Test robot accuracy
- **Inspection**: Circular scanning pattern
- **Mixing/stirring**: Circular motion applications
- **Testing**: Verify smooth servo mode operation
- **Demo**: Show path following capabilities

## Troubleshooting

### Circle too jerky
- Increase `num_points` (try 150-200)
- Check network latency

### Robot stops mid-circle
- Radius too large (unreachable points)
- Too close to singularity (center near base)
- Check workspace limits

### Slow execution
- Decrease `num_points`
- Should execute at ~100 Hz

### Position errors
- Circle center unreachable
- Points outside workspace
- Check final position after execution

## Advanced: Multiple Circles

Execute concentric circles:

```python
for radius in [30, 50, 70, 90]:
    circle_data = {
        "center_x": 400,
        "center_y": 0,
        "radius": radius,
        "z_height": 300,
        "num_points": 100,
        "roll": 180, "pitch": 0, "yaw": 0
    }
    requests.post("http://localhost:8000/api/path/circle/execute", json=circle_data)
    time.sleep(2)  # Wait for completion
```

## Next Steps

- Try different radii
- Change center position
- Combine with gripper control
- Create spiral patterns (decreasing/increasing radius)
- Use for pick-and-place applications
