#!/usr/bin/env python3
"""
Spline path visualization tool for xArm.
Generates and plots 2D spline paths before execution.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from matplotlib.patches import Circle
import argparse


def generate_spline_path(control_points, num_samples=200, spline_type="cubic"):
    """
    Generate a smooth 2D spline path from control points.
    Returns arrays of x and y coordinates.
    """
    if len(control_points) < 2:
        raise ValueError("Need at least 2 control points")

    # Extract x and y coordinates
    x_points = np.array([p[0] for p in control_points])
    y_points = np.array([p[1] for p in control_points])

    # Parametric representation
    t = np.linspace(0, 1, len(control_points))
    t_new = np.linspace(0, 1, num_samples)

    if spline_type == "cubic" and len(control_points) >= 4:
        # Cubic spline interpolation
        cs_x = interpolate.CubicSpline(t, x_points, bc_type='natural')
        cs_y = interpolate.CubicSpline(t, y_points, bc_type='natural')
        x_new = cs_x(t_new)
        y_new = cs_y(t_new)
    elif spline_type == "quadratic" and len(control_points) >= 3:
        # Quadratic spline
        cs_x = interpolate.interp1d(t, x_points, kind='quadratic')
        cs_y = interpolate.interp1d(t, y_points, kind='quadratic')
        x_new = cs_x(t_new)
        y_new = cs_y(t_new)
    else:
        # Linear interpolation fallback
        cs_x = interpolate.interp1d(t, x_points, kind='linear')
        cs_y = interpolate.interp1d(t, y_points, kind='linear')
        x_new = cs_x(t_new)
        y_new = cs_y(t_new)

    return x_new, y_new


def generate_circle_path(center_x, center_y, radius, num_points=100):
    """
    Generate a circular path.
    Returns arrays of x and y coordinates.
    """
    theta = np.linspace(0, 2*np.pi, num_points)
    x = center_x + radius * np.cos(theta)
    y = center_y + radius * np.sin(theta)
    return x, y


def plot_spline_path(control_points, num_samples=200, spline_type="cubic",
                     title="2D Spline Path", show_points=True, show_grid=True):
    """
    Plot a spline path with control points and trajectory.
    """
    # Generate spline
    x_path, y_path = generate_spline_path(control_points, num_samples, spline_type)

    # Extract control points
    x_ctrl = [p[0] for p in control_points]
    y_ctrl = [p[1] for p in control_points]

    # Create figure
    fig, ax = plt.subplots(figsize=(12, 8))

    # Plot the spline path
    ax.plot(x_path, y_path, 'b-', linewidth=2, label='Spline Path')

    # Plot control points
    ax.plot(x_ctrl, y_ctrl, 'ro', markersize=10, label='Control Points', zorder=5)

    # Connect control points with dashed lines
    ax.plot(x_ctrl, y_ctrl, 'r--', linewidth=1, alpha=0.5, label='Control Polygon')

    # Show sample points along path
    if show_points:
        sample_interval = max(1, num_samples // 20)  # Show ~20 points
        ax.plot(x_path[::sample_interval], y_path[::sample_interval],
                'g.', markersize=4, alpha=0.6, label='Sample Points')

    # Mark start and end
    ax.plot(x_path[0], y_path[0], 'gs', markersize=12, label='Start', zorder=6)
    ax.plot(x_path[-1], y_path[-1], 'rs', markersize=12, label='End', zorder=6)

    # Add direction arrows
    arrow_interval = max(1, num_samples // 10)
    for i in range(0, len(x_path) - arrow_interval, arrow_interval):
        dx = x_path[i + arrow_interval] - x_path[i]
        dy = y_path[i + arrow_interval] - y_path[i]
        ax.arrow(x_path[i], y_path[i], dx*0.3, dy*0.3,
                head_width=5, head_length=5, fc='blue', ec='blue', alpha=0.4)

    # Annotate control points
    for i, (x, y) in enumerate(control_points):
        ax.annotate(f'P{i}', (x, y), xytext=(5, 5), textcoords='offset points',
                   fontsize=10, fontweight='bold')

    # Formatting
    ax.set_xlabel('X (mm)', fontsize=12)
    ax.set_ylabel('Y (mm)', fontsize=12)
    ax.set_title(f'{title} ({spline_type.capitalize()} Spline, {num_samples} points)',
                 fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(show_grid, alpha=0.3)
    ax.axis('equal')

    # Add info text
    path_length = np.sum(np.sqrt(np.diff(x_path)**2 + np.diff(y_path)**2))
    info_text = f'Path Length: {path_length:.1f} mm\nControl Points: {len(control_points)}\nSamples: {num_samples}'
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
            fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout()
    return fig, ax


def plot_circle_path(center_x, center_y, radius, num_points=100,
                     title="Circular Path"):
    """
    Plot a circular path.
    """
    # Generate circle
    x_path, y_path = generate_circle_path(center_x, center_y, radius, num_points)

    # Create figure
    fig, ax = plt.subplots(figsize=(10, 10))

    # Plot the circle path
    ax.plot(x_path, y_path, 'b-', linewidth=2, label='Circle Path')

    # Plot center
    ax.plot(center_x, center_y, 'ro', markersize=10, label='Center', zorder=5)

    # Mark start and end
    ax.plot(x_path[0], y_path[0], 'gs', markersize=12, label='Start', zorder=6)

    # Show sample points
    sample_interval = max(1, num_points // 20)
    ax.plot(x_path[::sample_interval], y_path[::sample_interval],
            'g.', markersize=4, alpha=0.6, label='Sample Points')

    # Add direction arrows
    arrow_interval = max(1, num_points // 12)
    for i in range(0, len(x_path) - arrow_interval, arrow_interval):
        dx = x_path[i + arrow_interval] - x_path[i]
        dy = y_path[i + arrow_interval] - y_path[i]
        ax.arrow(x_path[i], y_path[i], dx*0.3, dy*0.3,
                head_width=radius*0.1, head_length=radius*0.1,
                fc='blue', ec='blue', alpha=0.4)

    # Draw radius line
    ax.plot([center_x, x_path[0]], [center_y, y_path[0]], 'r--',
            linewidth=1, alpha=0.5, label=f'Radius = {radius} mm')

    # Formatting
    ax.set_xlabel('X (mm)', fontsize=12)
    ax.set_ylabel('Y (mm)', fontsize=12)
    ax.set_title(f'{title} (r={radius}mm, {num_points} points)',
                 fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')

    # Add info text
    circumference = 2 * np.pi * radius
    info_text = f'Circumference: {circumference:.1f} mm\nRadius: {radius} mm\nSamples: {num_points}'
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
            fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout()
    return fig, ax


def plot_multiple_splines(paths_config, title="Multiple Paths Comparison"):
    """
    Plot multiple spline paths for comparison.

    paths_config: list of dicts with keys: 'control_points', 'spline_type', 'label', 'color'
    """
    fig, ax = plt.subplots(figsize=(14, 10))

    for config in paths_config:
        control_points = config['control_points']
        spline_type = config.get('spline_type', 'cubic')
        label = config.get('label', 'Path')
        color = config.get('color', 'blue')
        num_samples = config.get('num_samples', 200)

        # Generate spline
        x_path, y_path = generate_spline_path(control_points, num_samples, spline_type)

        # Plot
        ax.plot(x_path, y_path, '-', color=color, linewidth=2, label=label)

        # Plot control points
        x_ctrl = [p[0] for p in control_points]
        y_ctrl = [p[1] for p in control_points]
        ax.plot(x_ctrl, y_ctrl, 'o', color=color, markersize=8, alpha=0.6)

        # Mark start
        ax.plot(x_path[0], y_path[0], 's', color=color, markersize=10, zorder=6)

    ax.set_xlabel('X (mm)', fontsize=12)
    ax.set_ylabel('Y (mm)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')

    plt.tight_layout()
    return fig, ax


# Predefined example paths
def get_wave_pattern():
    """Generate a wave pattern."""
    return [
        (300, 0),
        (350, 50),
        (400, 0),
        (450, -50),
        (500, 0),
        (550, 50),
        (600, 0)
    ]


def get_s_shape():
    """Generate an S shape."""
    return [
        (300, 100),
        (350, 50),
        (400, 0),
        (450, -50),
        (500, -100)
    ]


def get_heart_shape():
    """Generate a heart shape."""
    return [
        (400, 0),
        (450, 50),
        (475, 100),
        (450, 150),
        (400, 175),
        (350, 150),
        (325, 100),
        (350, 50),
        (400, 0)
    ]


def get_spiral_pattern():
    """Generate a spiral pattern."""
    points = []
    for i in range(8):
        angle = i * np.pi / 4
        radius = 50 + i * 10
        x = 400 + radius * np.cos(angle)
        y = radius * np.sin(angle)
        points.append((x, y))
    return points


def get_square_pattern():
    """Generate a square with rounded corners."""
    return [
        (300, -50),
        (300, 50),
        (500, 50),
        (500, -50),
        (300, -50)
    ]


def main():
    parser = argparse.ArgumentParser(description='Visualize xArm spline paths')
    parser.add_argument('--pattern', choices=['wave', 's', 'heart', 'spiral', 'square', 'circle', 'all'],
                       default='wave', help='Predefined pattern to plot')
    parser.add_argument('--type', choices=['cubic', 'quadratic', 'linear'],
                       default='cubic', help='Spline type')
    parser.add_argument('--samples', type=int, default=200,
                       help='Number of samples along path')
    parser.add_argument('--save', type=str, help='Save figure to file')
    parser.add_argument('--no-show', action='store_true', help='Do not display plot')

    args = parser.parse_args()

    # Generate selected pattern
    if args.pattern == 'wave':
        control_points = get_wave_pattern()
        plot_spline_path(control_points, args.samples, args.type, "Wave Pattern")

    elif args.pattern == 's':
        control_points = get_s_shape()
        plot_spline_path(control_points, args.samples, args.type, "S-Shape Pattern")

    elif args.pattern == 'heart':
        control_points = get_heart_shape()
        plot_spline_path(control_points, args.samples, args.type, "Heart Pattern")

    elif args.pattern == 'spiral':
        control_points = get_spiral_pattern()
        plot_spline_path(control_points, args.samples, args.type, "Spiral Pattern")

    elif args.pattern == 'square':
        control_points = get_square_pattern()
        plot_spline_path(control_points, args.samples, args.type, "Square Pattern")

    elif args.pattern == 'circle':
        plot_circle_path(400, 0, 75, args.samples, "Circle Pattern")

    elif args.pattern == 'all':
        # Show comparison of all patterns
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle('xArm Spline Path Examples', fontsize=16, fontweight='bold')

        patterns = [
            (get_wave_pattern(), 'Wave'),
            (get_s_shape(), 'S-Shape'),
            (get_heart_shape(), 'Heart'),
            (get_spiral_pattern(), 'Spiral'),
            (get_square_pattern(), 'Square'),
        ]

        for idx, (pattern, name) in enumerate(patterns):
            row = idx // 3
            col = idx % 3
            ax = axes[row, col]

            x_path, y_path = generate_spline_path(pattern, args.samples, args.type)
            x_ctrl = [p[0] for p in pattern]
            y_ctrl = [p[1] for p in pattern]

            ax.plot(x_path, y_path, 'b-', linewidth=2)
            ax.plot(x_ctrl, y_ctrl, 'ro', markersize=6)
            ax.plot(x_ctrl, y_ctrl, 'r--', linewidth=1, alpha=0.5)
            ax.plot(x_path[0], y_path[0], 'gs', markersize=8)

            ax.set_title(name, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.axis('equal')
            ax.set_xlabel('X (mm)')
            ax.set_ylabel('Y (mm)')

        # Circle in last subplot
        ax = axes[1, 2]
        x_circ, y_circ = generate_circle_path(400, 0, 75, args.samples)
        ax.plot(x_circ, y_circ, 'b-', linewidth=2)
        ax.plot(400, 0, 'ro', markersize=6)
        ax.plot(x_circ[0], y_circ[0], 'gs', markersize=8)
        ax.set_title('Circle', fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')

        plt.tight_layout()

    # Save if requested
    if args.save:
        plt.savefig(args.save, dpi=300, bbox_inches='tight')
        print(f"Figure saved to {args.save}")

    # Show plot
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    # If run without arguments, show interactive examples
    import sys

    if len(sys.argv) == 1:
        print("xArm Spline Path Plotter")
        print("=" * 60)
        print("\nGenerating example patterns...\n")

        # Show all patterns
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle('xArm 2D Spline Path Examples', fontsize=16, fontweight='bold')

        patterns = [
            (get_wave_pattern(), 'Wave Pattern', 'cubic'),
            (get_s_shape(), 'S-Shape', 'cubic'),
            (get_heart_shape(), 'Heart', 'cubic'),
            (get_spiral_pattern(), 'Spiral', 'cubic'),
            (get_square_pattern(), 'Rounded Square', 'cubic'),
        ]

        for idx, (pattern, name, spline_type) in enumerate(patterns):
            row = idx // 3
            col = idx % 3
            ax = axes[row, col]

            x_path, y_path = generate_spline_path(pattern, 200, spline_type)
            x_ctrl = [p[0] for p in pattern]
            y_ctrl = [p[1] for p in pattern]

            # Plot spline
            ax.plot(x_path, y_path, 'b-', linewidth=2.5, label='Path')
            ax.plot(x_ctrl, y_ctrl, 'ro', markersize=8, label='Control Points', zorder=5)
            ax.plot(x_ctrl, y_ctrl, 'r--', linewidth=1, alpha=0.5)
            ax.plot(x_path[0], y_path[0], 'gs', markersize=10, label='Start', zorder=6)

            # Add arrows
            arrow_interval = max(1, len(x_path) // 8)
            for i in range(0, len(x_path) - arrow_interval, arrow_interval):
                dx = x_path[i + arrow_interval] - x_path[i]
                dy = y_path[i + arrow_interval] - y_path[i]
                ax.arrow(x_path[i], y_path[i], dx*0.3, dy*0.3,
                        head_width=5, head_length=5, fc='blue', ec='blue', alpha=0.4)

            ax.set_title(name, fontweight='bold', fontsize=12)
            ax.grid(True, alpha=0.3)
            ax.axis('equal')
            ax.set_xlabel('X (mm)', fontsize=10)
            ax.set_ylabel('Y (mm)', fontsize=10)
            if idx == 0:
                ax.legend(fontsize=8, loc='upper right')

        # Circle in last subplot
        ax = axes[1, 2]
        x_circ, y_circ = generate_circle_path(400, 0, 75, 100)
        ax.plot(x_circ, y_circ, 'b-', linewidth=2.5, label='Path')
        ax.plot(400, 0, 'ro', markersize=8, label='Center', zorder=5)
        ax.plot(x_circ[0], y_circ[0], 'gs', markersize=10, label='Start', zorder=6)

        # Add arrows for circle
        arrow_interval = len(x_circ) // 12
        for i in range(0, len(x_circ) - arrow_interval, arrow_interval):
            dx = x_circ[i + arrow_interval] - x_circ[i]
            dy = y_circ[i + arrow_interval] - y_circ[i]
            ax.arrow(x_circ[i], y_circ[i], dx*0.3, dy*0.3,
                    head_width=5, head_length=5, fc='blue', ec='blue', alpha=0.4)

        ax.set_title('Circle Pattern', fontweight='bold', fontsize=12)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        ax.set_xlabel('X (mm)', fontsize=10)
        ax.set_ylabel('Y (mm)', fontsize=10)
        ax.legend(fontsize=8, loc='upper right')

        plt.tight_layout()

        print("Usage examples:")
        print("  python3 plot_spline.py --pattern wave")
        print("  python3 plot_spline.py --pattern heart --type cubic")
        print("  python3 plot_spline.py --pattern all --save paths.png")
        print("\nShowing all example patterns...")

        plt.show()
    else:
        main()
