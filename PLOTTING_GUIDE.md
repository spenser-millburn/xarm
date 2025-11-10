# Spline Path Plotting Guide

This guide explains how to visualize and design 2D spline paths for the xArm before executing them on the robot.

## Quick Start

### View All Example Patterns

```bash
cd /home/mclovin/xarm
python3 plot_spline.py
```

This displays 6 example patterns in a grid:
- Wave Pattern
- S-Shape
- Heart
- Spiral
- Rounded Square
- Circle

## Command Line Usage

### Basic Patterns

```bash
# Plot a wave pattern
python3 plot_spline.py --pattern wave

# Plot an S-shape
python3 plot_spline.py --pattern s

# Plot a heart shape
python3 plot_spline.py --pattern heart

# Plot a spiral
python3 plot_spline.py --pattern spiral

# Plot a square with rounded corners
python3 plot_spline.py --pattern square

# Plot a circle
python3 plot_spline.py --pattern circle

# Show all patterns in comparison view
python3 plot_spline.py --pattern all
```

### Spline Type Options

```bash
# Cubic spline (smoothest, default)
python3 plot_spline.py --pattern wave --type cubic

# Quadratic spline
python3 plot_spline.py --pattern wave --type quadratic

# Linear interpolation (sharp corners)
python3 plot_spline.py --pattern wave --type linear
```

### Sample Resolution

```bash
# Low resolution (faster)
python3 plot_spline.py --pattern heart --samples 50

# High resolution (smoother)
python3 plot_spline.py --pattern heart --samples 500

# Default is 200 samples
```

### Save Plots

```bash
# Save to file without displaying
python3 plot_spline.py --pattern all --save paths.png --no-show

# Save and display
python3 plot_spline.py --pattern heart --save heart.png

# Save with high DPI
python3 plot_spline.py --pattern wave --save wave_hires.png
```

## Python API Usage

### Import and Generate

```python
from plot_spline import generate_spline_path, plot_spline_path

# Define control points
control_points = [
    (300, 0),
    (350, 50),
    (400, 0),
    (450, -50),
    (500, 0)
]

# Generate path arrays
x_path, y_path = generate_spline_path(control_points, num_samples=200, spline_type="cubic")

# Plot it
plot_spline_path(control_points, num_samples=200, spline_type="cubic", title="My Custom Path")
plt.show()
```

### Circle Path

```python
from plot_spline import generate_circle_path, plot_circle_path

# Generate circle
x_path, y_path = generate_circle_path(center_x=400, center_y=0, radius=75, num_points=100)

# Plot it
plot_circle_path(400, 0, 75, 100, "My Circle")
plt.show()
```

### Compare Multiple Paths

```python
from plot_spline import plot_multiple_splines

paths = [
    {
        'control_points': [(300, 0), (400, 50), (500, 0)],
        'spline_type': 'cubic',
        'label': 'Cubic',
        'color': 'blue',
        'num_samples': 200
    },
    {
        'control_points': [(300, 0), (400, 50), (500, 0)],
        'spline_type': 'quadratic',
        'label': 'Quadratic',
        'color': 'red',
        'num_samples': 200
    },
    {
        'control_points': [(300, 0), (400, 50), (500, 0)],
        'spline_type': 'linear',
        'label': 'Linear',
        'color': 'green',
        'num_samples': 200
    }
]

plot_multiple_splines(paths, title="Spline Type Comparison")
plt.show()
```

## Creating Custom Patterns

### Step 1: Design Control Points

Think about your desired path shape and place control points:

```python
# Example: Letter "M" shape
control_points = [
    (300, 0),    # Bottom left
    (325, 100),  # Top left peak
    (350, 50),   # Middle valley
    (375, 100),  # Top right peak
    (400, 0)     # Bottom right
]
```

### Step 2: Visualize

```python
from plot_spline import plot_spline_path
import matplotlib.pyplot as plt

plot_spline_path(control_points, num_samples=200, spline_type="cubic", title="Letter M")
plt.show()
```

### Step 3: Iterate

Adjust control points until satisfied, then use the same points in your xArm API call.

## Understanding the Plot

### Elements Shown

- **Blue Line**: The actual spline path the robot will follow
- **Red Dots**: Your control points
- **Red Dashed Line**: Control polygon connecting control points
- **Green Square**: Start position
- **Red Square**: End position
- **Green Dots**: Sample points along the path (where robot will be commanded)
- **Blue Arrows**: Direction of motion
- **Info Box**: Path statistics (length, number of points, etc.)

### Path Length Calculation

The plot calculates approximate path length in mm:

```
Path Length = sum of distances between consecutive sample points
```

This helps estimate execution time:
- At 100 Hz: time = num_samples / 100 seconds
- At 200 Hz: time = num_samples / 200 seconds

## Predefined Pattern Library

The script includes these built-in patterns:

### 1. Wave Pattern
```python
from plot_spline import get_wave_pattern
points = get_wave_pattern()
# Returns: [(300, 0), (350, 50), (400, 0), (450, -50), (500, 0), (550, 50), (600, 0)]
```

### 2. S-Shape
```python
from plot_spline import get_s_shape
points = get_s_shape()
# Returns: [(300, 100), (350, 50), (400, 0), (450, -50), (500, -100)]
```

### 3. Heart
```python
from plot_spline import get_heart_shape
points = get_heart_shape()
# Returns 9 points forming a heart
```

### 4. Spiral
```python
from plot_spline import get_spiral_pattern
points = get_spiral_pattern()
# Returns points forming an outward spiral
```

### 5. Square
```python
from plot_spline import get_square_pattern
points = get_square_pattern()
# Returns square corners that will be rounded by spline
```

## Integration with xArm API

Once you're happy with a plotted path, use the same control points with the API:

```python
import requests

# Design and visualize first
from plot_spline import get_wave_pattern, plot_spline_path
import matplotlib.pyplot as plt

control_points = get_wave_pattern()
plot_spline_path(control_points)
plt.show()

# Then execute on robot
data = {
    "control_points": [{"x": x, "y": y} for x, y in control_points],
    "z_height": 300,
    "num_samples": 200,
    "spline_type": "cubic",
    "roll": 180,
    "pitch": 0,
    "yaw": 0
}

# Preview via API
preview = requests.post("http://localhost:8000/api/path/spline/preview", json=data)
print(preview.json())

# Execute (robot moves!)
response = requests.post("http://localhost:8000/api/path/spline/execute", json=data)
```

## Tips for Good Paths

### 1. Smooth Curves
- Use **cubic splines** for smooth curves
- Place control points where you want the curve to change direction
- More control points = more complex curves but may be harder to control

### 2. Sharp Corners
- Use **linear interpolation** for sharp corners
- Place control points exactly at corner locations

### 3. Workspace Limits
- Typical xArm6 reach: ~700mm
- Keep X: 200-600mm, Y: -300 to +300mm for safety
- Z height: 200-400mm recommended for 2D paths

### 4. Avoiding Singularities
- Stay away from base center (X=0, Y=0)
- Don't make extremely tight curves
- Test with preview before executing

### 5. Path Density
- 100-200 samples is optimal for most paths
- Longer paths may need more samples
- Very short paths can use fewer samples

## Troubleshooting

### Plot doesn't show
```bash
# Check matplotlib backend
export MPLBACKEND=TkAgg
python3 plot_spline.py
```

### "Not enough control points" error
- Cubic needs at least 4 points
- Quadratic needs at least 3 points
- Linear needs at least 2 points

### Path looks jagged
- Increase `--samples` value
- Use cubic instead of linear spline type

### Plot axes not equal
- This is handled automatically with `ax.axis('equal')`
- If still an issue, the workspace might be unusual

## Examples Gallery

### Example 1: Simple Arc
```python
points = [(300, 0), (400, 100), (500, 0)]
plot_spline_path(points, spline_type="cubic")
```

### Example 2: Figure-8
```python
points = [
    (350, 0),
    (400, 50),
    (450, 0),
    (400, -50),
    (350, 0)
]
plot_spline_path(points, spline_type="cubic")
```

### Example 3: Zigzag
```python
points = [
    (300, -50),
    (350, 50),
    (400, -50),
    (450, 50),
    (500, -50)
]
plot_spline_path(points, spline_type="linear")  # Sharp corners
```

## Command Reference

```bash
# Full command syntax
python3 plot_spline.py [OPTIONS]

Options:
  --pattern {wave,s,heart,spiral,square,circle,all}
                        Predefined pattern to plot (default: wave)
  --type {cubic,quadratic,linear}
                        Spline type (default: cubic)
  --samples N           Number of samples along path (default: 200)
  --save FILENAME       Save figure to file
  --no-show            Do not display plot (useful with --save)
  --help               Show help message
```

## Next Steps

1. **Experiment** with predefined patterns
2. **Design** your own paths by modifying control points
3. **Visualize** and verify the path looks correct
4. **Preview** via API (`/api/path/spline/preview`)
5. **Execute** on the robot (`/api/path/spline/execute`)

Always test paths with preview and visualization before running on the actual robot!
