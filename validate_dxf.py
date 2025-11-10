#!/usr/bin/env python3
"""
DXF Path Validator for xArm6
Validates that all points in a DXF file are reachable by the robot arm
"""

import ezdxf
import numpy as np
from typing import List, Tuple, Dict
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

# xArm6 specifications
XARM6_REACH_MAX = 700.0  # mm, maximum reach
XARM6_REACH_MIN = 100.0  # mm, minimum reach (to avoid singularities)
Z_MIN = 0.0              # mm, minimum Z height (table level)
Z_MAX = 600.0            # mm, maximum Z height

class PathValidator:
    def __init__(self, dxf_path: str, scale: float = 1.0, offset_x: float = 0.0, offset_y: float = 0.0):
        self.dxf_path = dxf_path
        self.scale = scale
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.errors = []
        self.warnings = []
        self.points = []

    def load_dxf(self):
        """Load and parse DXF file, extracting all path points"""
        print(f"[LOAD] Loading DXF: {self.dxf_path}")
        print(f"[LOAD] Scale: {self.scale}, Offset: ({self.offset_x}, {self.offset_y})")

        try:
            doc = ezdxf.readfile(self.dxf_path)
            msp = doc.modelspace()

            points_per_unit = 4.0
            all_points = []

            for entity in msp:
                if entity.dxftype() == 'LINE':
                    start = entity.dxf.start
                    end = entity.dxf.end

                    start_2d = (start.x, start.y)
                    end_2d = (end.x, end.y)

                    length = np.sqrt((end_2d[0] - start_2d[0])**2 + (end_2d[1] - start_2d[1])**2)
                    num_points = max(2, int(length * points_per_unit))

                    x_line = np.linspace(start_2d[0], end_2d[0], num_points)
                    y_line = np.linspace(start_2d[1], end_2d[1], num_points)

                    for x, y in zip(x_line, y_line):
                        all_points.append((x, y))

                elif entity.dxftype() == 'LWPOLYLINE' or entity.dxftype() == 'POLYLINE':
                    if entity.dxftype() == 'LWPOLYLINE':
                        points = list(entity.get_points())
                    else:  # POLYLINE
                        points = [(v.dxf.location.x, v.dxf.location.y) for v in entity.vertices]

                    for i in range(len(points) - 1):
                        start = points[i]
                        end = points[i + 1]

                        length = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
                        num_points = max(2, int(length * points_per_unit))

                        x_line = np.linspace(start[0], end[0], num_points)
                        y_line = np.linspace(start[1], end[1], num_points)

                        for x, y in zip(x_line, y_line):
                            all_points.append((x, y))

            # Apply scale and offset
            self.points = [(x * self.scale + self.offset_x,
                           y * self.scale + self.offset_y,
                           300.0)  # Z height from your logs
                          for x, y in all_points]

            print(f"[LOAD] Extracted {len(self.points)} points from DXF")
            return True

        except Exception as e:
            self.errors.append(f"Failed to load DXF: {e}")
            return False

    def validate_workspace(self):
        """Check if all points are within robot workspace"""
        print("\n[VALIDATE] Checking workspace boundaries...")

        unreachable_points = []
        too_close_points = []
        z_violations = []

        for i, (x, y, z) in enumerate(self.points):
            # Calculate distance from robot base (assumed at origin)
            radius = np.sqrt(x**2 + y**2)

            # Check maximum reach
            if radius > XARM6_REACH_MAX:
                unreachable_points.append((i, x, y, z, radius))

            # Check minimum reach (singularity zone)
            if radius < XARM6_REACH_MIN:
                too_close_points.append((i, x, y, z, radius))

            # Check Z limits
            if z < Z_MIN or z > Z_MAX:
                z_violations.append((i, x, y, z))

        # Report findings
        if unreachable_points:
            self.errors.append(f"Found {len(unreachable_points)} points beyond maximum reach ({XARM6_REACH_MAX}mm)")
            print(f"[ERROR] {len(unreachable_points)} points beyond max reach!")
            print("[ERROR] First 5 unreachable points:")
            for i, x, y, z, r in unreachable_points[:5]:
                print(f"  Point {i}: pos=[{x:.1f}, {y:.1f}, {z:.1f}], radius={r:.1f}mm (exceeds {XARM6_REACH_MAX}mm)")

        if too_close_points:
            self.warnings.append(f"Found {len(too_close_points)} points too close to robot base")
            print(f"[WARN] {len(too_close_points)} points in singularity zone!")

        if z_violations:
            self.errors.append(f"Found {len(z_violations)} points with invalid Z height")
            print(f"[ERROR] {len(z_violations)} Z-height violations!")

        if not unreachable_points and not too_close_points and not z_violations:
            print("[OK] All points within workspace boundaries")

        return {
            'unreachable': unreachable_points,
            'too_close': too_close_points,
            'z_violations': z_violations
        }

    def analyze_problem_area(self):
        """Specifically analyze the area around the failure point"""
        print("\n[ANALYZE] Checking area around failure point [531.1, 229.1, 300.0]...")

        failure_x, failure_y, failure_z = 531.1, 229.1, 300.0
        failure_radius = np.sqrt(failure_x**2 + failure_y**2)

        print(f"[ANALYZE] Failure point radius: {failure_radius:.1f}mm")
        print(f"[ANALYZE] Max reach: {XARM6_REACH_MAX:.1f}mm")
        print(f"[ANALYZE] Clearance: {XARM6_REACH_MAX - failure_radius:.1f}mm")

        # Find points near the failure
        nearby_points = []
        search_radius = 50.0  # mm

        for i, (x, y, z) in enumerate(self.points):
            dist_to_failure = np.sqrt((x - failure_x)**2 + (y - failure_y)**2 + (z - failure_z)**2)
            if dist_to_failure < search_radius:
                radius = np.sqrt(x**2 + y**2)
                nearby_points.append((i, x, y, z, radius, dist_to_failure))

        if nearby_points:
            print(f"\n[ANALYZE] Found {len(nearby_points)} points within {search_radius}mm of failure:")
            nearby_points.sort(key=lambda p: p[5])  # Sort by distance to failure
            for i, x, y, z, r, d in nearby_points[:10]:
                status = "UNREACHABLE" if r > XARM6_REACH_MAX else "OK"
                print(f"  Point {i}: pos=[{x:.1f}, {y:.1f}, {z:.1f}], radius={r:.1f}mm, {status}")

    def get_statistics(self):
        """Calculate statistics about the path"""
        print("\n[STATS] Path Statistics:")

        if not self.points:
            print("[STATS] No points loaded")
            return

        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        zs = [p[2] for p in self.points]
        radii = [np.sqrt(x**2 + y**2) for x, y in zip(xs, ys)]

        print(f"  Total points: {len(self.points)}")
        print(f"  X range: [{min(xs):.1f}, {max(xs):.1f}] mm")
        print(f"  Y range: [{min(ys):.1f}, {max(ys):.1f}] mm")
        print(f"  Z range: [{min(zs):.1f}, {max(zs):.1f}] mm")
        print(f"  Radius range: [{min(radii):.1f}, {max(radii):.1f}] mm")
        print(f"  Max safe radius: {XARM6_REACH_MAX} mm")

        # Calculate center of mass
        center_x = np.mean(xs)
        center_y = np.mean(ys)
        center_radius = np.sqrt(center_x**2 + center_y**2)

        print(f"\n  Path center: [{center_x:.1f}, {center_y:.1f}]")
        print(f"  Center radius: {center_radius:.1f} mm")

    def suggest_fixes(self):
        """Suggest scale/offset adjustments to fit path within workspace"""
        print("\n[SUGGESTIONS] Recommended fixes:")

        if not self.points:
            return

        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]

        # Calculate current bounds
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        # Calculate current center
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2

        # Calculate maximum radius in current configuration
        max_radius = max(np.sqrt(x**2 + y**2) for x, y in zip(xs, ys))

        if max_radius > XARM6_REACH_MAX:
            # Need to scale down
            scale_factor = XARM6_REACH_MAX * 0.9 / max_radius  # 90% of max for safety
            suggested_scale = self.scale * scale_factor

            print(f"1. SCALE DOWN: Use scale={suggested_scale:.3f} (current: {self.scale})")
            print(f"   This will reduce max radius from {max_radius:.1f}mm to {max_radius * scale_factor:.1f}mm")

        # Suggest centering the path
        print(f"\n2. CENTER THE PATH:")
        print(f"   Current center: [{center_x:.1f}, {center_y:.1f}]")
        print(f"   Suggested offset: [{self.offset_x - center_x:.1f}, {self.offset_y - center_y:.1f}]")
        print(f"   This will center the drawing at the robot origin")

        # Suggest safe workspace
        safe_radius = XARM6_REACH_MAX * 0.8  # Use 80% of workspace for safety
        print(f"\n3. SAFE WORKSPACE:")
        print(f"   Keep all points within {safe_radius:.1f}mm radius")
        print(f"   Current max radius: {max_radius:.1f}mm")

    def visualize(self, output_file='path_validation.png'):
        """Create visualization of the path and workspace"""
        print(f"\n[VIZ] Creating visualization: {output_file}")

        if not self.points:
            print("[VIZ] No points to visualize")
            return

        fig = plt.figure(figsize=(15, 5))

        # 2D Top view
        ax1 = fig.add_subplot(131)
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        radii = [np.sqrt(x**2 + y**2) for x, y in zip(xs, ys)]

        # Color points by reachability
        colors = ['red' if r > XARM6_REACH_MAX else 'green' for r in radii]
        ax1.scatter(xs, ys, c=colors, s=1, alpha=0.5)

        # Draw workspace circles
        circle_max = plt.Circle((0, 0), XARM6_REACH_MAX, color='blue', fill=False, linestyle='--', label='Max reach')
        circle_safe = plt.Circle((0, 0), XARM6_REACH_MAX * 0.8, color='green', fill=False, linestyle=':', label='Safe zone')
        circle_min = plt.Circle((0, 0), XARM6_REACH_MIN, color='red', fill=False, linestyle='--', label='Singularity')
        ax1.add_patch(circle_max)
        ax1.add_patch(circle_safe)
        ax1.add_patch(circle_min)

        # Mark failure point
        ax1.plot(531.1, 229.1, 'rx', markersize=15, markeredgewidth=3, label='Failure point')

        ax1.set_xlabel('X (mm)')
        ax1.set_ylabel('Y (mm)')
        ax1.set_title('Top View - Workspace Check')
        ax1.axis('equal')
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        # Radius over time
        ax2 = fig.add_subplot(132)
        ax2.plot(radii, linewidth=0.5)
        ax2.axhline(y=XARM6_REACH_MAX, color='r', linestyle='--', label='Max reach')
        ax2.axhline(y=XARM6_REACH_MAX * 0.8, color='g', linestyle=':', label='Safe zone')
        ax2.axhline(y=XARM6_REACH_MIN, color='orange', linestyle='--', label='Min reach')

        # Mark where failure occurred (point 2988)
        if len(radii) > 2988:
            ax2.plot(2988, radii[2988], 'rx', markersize=10, markeredgewidth=2)

        ax2.set_xlabel('Point Index')
        ax2.set_ylabel('Radius from Origin (mm)')
        ax2.set_title('Radius Over Path')
        ax2.grid(True, alpha=0.3)
        ax2.legend()

        # 3D view
        ax3 = fig.add_subplot(133, projection='3d')
        zs = [p[2] for p in self.points]
        ax3.scatter(xs, ys, zs, c=colors, s=1, alpha=0.3)
        ax3.plot([531.1], [229.1], [300.0], 'rx', markersize=15, markeredgewidth=3)

        ax3.set_xlabel('X (mm)')
        ax3.set_ylabel('Y (mm)')
        ax3.set_zlabel('Z (mm)')
        ax3.set_title('3D Path View')

        plt.tight_layout()
        plt.savefig(output_file, dpi=150)
        print(f"[VIZ] Saved to: {output_file}")

    def run(self):
        """Run complete validation"""
        print("=" * 70)
        print("xArm6 DXF Path Validator")
        print("=" * 70)

        if not self.load_dxf():
            print("\n[FAILED] Could not load DXF file")
            return False

        self.get_statistics()
        violations = self.validate_workspace()
        self.analyze_problem_area()
        self.suggest_fixes()
        self.visualize()

        print("\n" + "=" * 70)
        print("VALIDATION SUMMARY")
        print("=" * 70)

        if self.errors:
            print(f"[ERROR] Found {len(self.errors)} error(s):")
            for error in self.errors:
                print(f"  - {error}")

        if self.warnings:
            print(f"[WARN] Found {len(self.warnings)} warning(s):")
            for warning in self.warnings:
                print(f"  - {warning}")

        if not self.errors and not self.warnings:
            print("[SUCCESS] Path is valid and within robot workspace!")
            return True
        else:
            print("\n[FAILED] Path validation failed. See suggestions above.")
            return False


def main():
    if len(sys.argv) < 2:
        print("Usage: python validate_dxf.py <dxf_file> [scale] [offset_x] [offset_y]")
        print("\nExample:")
        print("  python validate_dxf.py /home/mclovin/Downloads/fixed_top.dxf 1.0 400.0 -200.0")
        sys.exit(1)

    dxf_file = sys.argv[1]
    scale = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    offset_x = float(sys.argv[3]) if len(sys.argv) > 3 else 400.0
    offset_y = float(sys.argv[4]) if len(sys.argv) > 4 else -200.0

    validator = PathValidator(dxf_file, scale, offset_x, offset_y)
    success = validator.run()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
