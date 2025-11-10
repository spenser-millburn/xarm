#!/usr/bin/env python3
"""
Execute real spline paths on the xArm robot.
Safety-focused script with step-by-step execution.
"""

import requests
import time
import sys
import serial

API_BASE = "http://localhost:8000/api"

# Global start position storage
START_POSITION = {"x": 300, "y": 0, "z": 300, "roll": 180, "pitch": 0, "yaw": 0}

# Serial port configuration for laser gas control
SERIAL_PORT = "/dev/ttyUSB1"
SERIAL_BAUD = 115200


def send_gas_command(command):
    """Send command to gas control via serial port."""
    try:
        with serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1) as ser:
            # Send command
            ser.write(f"{command}\n".encode())
            time.sleep(0.1)  # Brief delay for command processing
            print(f"✓ Sent gas command: {command}")
            return True
    except serial.SerialException as e:
        print(f"✗ Serial error: {e}")
        return False
    except Exception as e:
        print(f"✗ Error sending gas command: {e}")
        return False


def gas_on():
    """Turn on the gas for laser cutting."""
    print("🔥 Turning gas ON...")
    return send_gas_command("gas on")


def gas_off():
    """Turn off the gas."""
    print("💨 Turning gas OFF...")
    return send_gas_command("gas off")


def laser_on():
    """Turn on the laser."""
    print("⚡ Turning LASER ON...")
    return send_gas_command("onkey")


def laser_off():
    """Turn off the laser."""
    print("🔌 Turning LASER OFF...")
    return send_gas_command("offkey")


def check_connection():
    """Verify server and robot are connected."""
    print("=" * 60)
    print("Checking connection to xArm...")
    print("=" * 60)

    try:
        response = requests.get(f"{API_BASE}/status", timeout=5)
        if response.status_code == 200:
            status = response.json()
            print(f"✓ Server connected")
            print(f"  Robot connected: {status['connected']}")
            print(f"  Mode: {status['mode']}")
            print(f"  State: {status['state_name']}")
            print(f"  Error code: {status['error_code']}")
            print(f"  Warn code: {status['warn_code']}")

            if status['position']:
                print(f"  Current position: X={status['position'][0]:.1f}, Y={status['position'][1]:.1f}, Z={status['position'][2]:.1f}")

            return status['connected']
        else:
            print(f"✗ Server error: {response.status_code}")
            return False
    except requests.exceptions.ConnectionError:
        print("✗ Cannot connect to server!")
        print("  Make sure the server is running:")
        print("    cd /home/mclovin/xarm")
        print("    ./start.sh")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def enable_robot():
    """Enable robot motion."""
    print("\n" + "=" * 60)
    print("Enabling robot motion...")
    print("=" * 60)

    try:
        response = requests.post(f"{API_BASE}/motion/enable")
        if response.status_code == 200:
            print("✓ Robot enabled")
            time.sleep(1)
            return True
        else:
            print(f"✗ Enable failed: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def move_to_start_position(x=300, y=0, z=300):
    """Move robot to safe starting position."""
    print("\n" + "=" * 60)
    print(f"Moving to start position: X={x}, Y={y}, Z={z}")
    print("=" * 60)

    response = input("Proceed with move to start position? [y/N]: ")
    if response.lower() != 'y':
        print("Cancelled by user")
        return False

    try:
        data = {
            "x": x,
            "y": y,
            "z": z,
            "roll": 180,
            "pitch": 0,
            "yaw": 0,
            "speed": 100,
            "wait": True
        }

        response = requests.post(f"{API_BASE}/move/position", json=data)
        if response.status_code == 200:
            print("✓ Moved to start position")
            time.sleep(2)
            return True
        else:
            print(f"✗ Move failed: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def preview_path(control_points, z_height=300, spline_type="cubic", num_samples=200):
    """Preview the path without moving robot."""
    print("\n" + "=" * 60)
    print("Generating path preview...")
    print("=" * 60)

    data = {
        "control_points": control_points,
        "z_height": z_height,
        "num_samples": num_samples,
        "spline_type": spline_type,
        "roll": 180,
        "pitch": 0,
        "yaw": 0
    }

    try:
        response = requests.post(f"{API_BASE}/path/spline/preview", json=data)
        if response.status_code == 200:
            result = response.json()
            print(f"✓ Path preview generated")
            print(f"  Control points: {len(control_points)}")
            print(f"  Total samples: {result['num_points']}")
            print(f"  Spline type: {spline_type}")
            print(f"  Z height: {z_height} mm")

            # Show first few points
            if 'path_points' in result and len(result['path_points']) > 0:
                print(f"\n  First 3 points:")
                for i, pt in enumerate(result['path_points'][:3]):
                    print(f"    Point {i}: X={pt['x']:.1f}, Y={pt['y']:.1f}, Z={pt['z']:.1f}")

            return True
        else:
            print(f"✗ Preview failed: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def execute_spline_path(control_points, z_height=300, spline_type="cubic", num_samples=200, speed_scale=1.0):
    """Execute the spline path on the robot."""
    print("\n" + "=" * 60)
    print("⚠️  EXECUTING PATH - ROBOT WILL MOVE!")
    print("=" * 60)
    print(f"Path configuration:")
    print(f"  Control points: {control_points}")
    print(f"  Z height: {z_height} mm")
    print(f"  Spline type: {spline_type}")
    print(f"  Samples: {num_samples}")

    response = input("\n⚠️  Robot will move! Continue? [y/N]: ")
    if response.lower() != 'y':
        print("Cancelled by user")
        return False

    data = {
        "control_points": control_points,
        "z_height": z_height,
        "num_samples": num_samples,
        "spline_type": spline_type,
        "roll": 180,
        "pitch": 0,
        "yaw": 0
    }

    try:
        response = requests.post(f"{API_BASE}/path/spline/execute", json=data)
        if response.status_code == 200:
            result = response.json()
            print(f"\n✓ Path execution started!")
            print(f"  Number of points: {result['num_points']}")
            print(f"  Frequency: {result['frequency_hz']} Hz")
            print(f"  Estimated duration: {result['estimated_duration_sec']:.2f} seconds")

            # Monitor execution
            print("\nMonitoring execution...")
            print("(Check server terminal for detailed logs)")
            start_time = time.time()

            # Give thread time to start
            time.sleep(0.1)

            max_wait = result['estimated_duration_sec'] + 5  # Add 5 second buffer

            while True:
                elapsed = time.time() - start_time

                if elapsed > max_wait:
                    print(f"\n⚠️  Timeout after {elapsed:.1f}s (expected {result['estimated_duration_sec']:.1f}s)")
                    break

                status_resp = requests.get(f"{API_BASE}/path/status")
                if status_resp.status_code == 200:
                    status = status_resp.json()

                    if status['is_executing']:
                        progress = (elapsed / result['estimated_duration_sec']) * 100
                        print(f"  {elapsed:.1f}s / {result['estimated_duration_sec']:.1f}s ({min(progress, 100):.0f}%) - Executing...", end='\r')
                        time.sleep(0.1)  # Check more frequently
                    else:
                        print(f"\n✓ Path completed in {elapsed:.1f} seconds!")
                        break
                else:
                    print(f"\n⚠️  Status check failed")
                    break

            # Final status check
            final_status = requests.get(f"{API_BASE}/status").json()
            if final_status.get('position'):
                pos = final_status['position']
                print(f"  Final position: X={pos[0]:.1f}, Y={pos[1]:.1f}, Z={pos[2]:.1f}")

            return True
        else:
            print(f"✗ Execution failed: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def execute_circle_path(center_x=400, center_y=0, radius=50, z_height=300, num_points=100, num_cycles=1):
    """Execute a circular path."""
    print("\n" + "=" * 60)
    print("⚠️  EXECUTING CIRCLE - ROBOT WILL MOVE!")
    print("=" * 60)
    print(f"Circle configuration:")
    print(f"  Center: X={center_x}, Y={center_y}")
    print(f"  Radius: {radius} mm")
    print(f"  Z height: {z_height} mm")
    print(f"  Points: {num_points}")
    print(f"  Cycles: {num_cycles}")

    circumference = 2 * 3.14159 * radius
    total_distance = circumference * num_cycles
    print(f"  Total distance: {total_distance:.0f} mm ({total_distance/1000:.2f} meters)")

    response = input("\n⚠️  Robot will move! Continue? [y/N]: ")
    if response.lower() != 'y':
        print("Cancelled by user")
        return False

    data = {
        "center_x": center_x,
        "center_y": center_y,
        "radius": radius,
        "z_height": z_height,
        "num_points": num_points,
        "num_cycles": num_cycles,
        "roll": 180,
        "pitch": 0,
        "yaw": 0
    }

    try:
        response = requests.post(f"{API_BASE}/path/circle/execute", json=data)
        if response.status_code == 200:
            result = response.json()
            print(f"\n✓ Circle execution started!")
            print(f"  Number of points: {result['num_points']}")
            print(f"  Frequency: {result['frequency_hz']} Hz")
            print(f"  Estimated duration: {result['estimated_duration_sec']:.2f} seconds")

            # Monitor execution
            print("\nMonitoring execution...")
            start_time = time.time()

            while True:
                status_resp = requests.get(f"{API_BASE}/path/status")
                if status_resp.status_code == 200:
                    status = status_resp.json()
                    elapsed = time.time() - start_time

                    if status['is_executing']:
                        print(f"  {elapsed:.1f}s - Circle executing...", end='\r')
                        time.sleep(0.5)
                    else:
                        print(f"\n✓ Circle completed in {elapsed:.1f} seconds!")
                        break
                else:
                    print(f"\n⚠️  Status check failed")
                    break

            return True
        else:
            print(f"✗ Execution failed: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def execute_laser_cut_circle(center_x=400, center_y=0, radius=50, z_height=300, num_points=100, num_cycles=1):
    """Execute a laser cut circle with gas control."""
    print("\n" + "=" * 60)
    print("🔥 LASER CUT CIRCLE - ROBOT WILL MOVE AND GAS WILL ACTIVATE!")
    print("=" * 60)
    print(f"Circle configuration:")
    print(f"  Center: X={center_x}, Y={center_y}")
    print(f"  Radius: {radius} mm")
    print(f"  Z height: {z_height} mm")
    print(f"  Points: {num_points}")
    print(f"  Cycles: {num_cycles}")

    circumference = 2 * 3.14159 * radius
    total_distance = circumference * num_cycles
    print(f"  Total distance: {total_distance:.0f} mm ({total_distance/1000:.2f} meters)")
    print(f"\n⚠️  LASER CUTTING SEQUENCE:")
    print(f"  1. Move to circle start position")
    print(f"  2. Turn ON gas")
    print(f"  3. Wait 5 seconds for gas stabilization")
    print(f"  4. Turn ON laser")
    print(f"  5. Execute circle path (cutting)")
    print(f"  6. Turn OFF laser")
    print(f"  7. Turn OFF gas")

    response = input("\n⚠️  Robot will move and GAS will activate! Continue? [y/N]: ")
    if response.lower() != 'y':
        print("Cancelled by user")
        return False

    try:
        # Step 1: Move robot to circle start position FIRST (without executing path)
        import math
        start_x = center_x + radius  # Start at rightmost point of circle
        start_y = center_y

        print("\n📍 Step 1: Moving to circle start position...")
        move_data = {
            "x": start_x,
            "y": start_y,
            "z": z_height,
            "roll": 180,
            "pitch": 0,
            "yaw": 0,
            "speed": 100,
            "wait": True
        }

        move_response = requests.post(f"{API_BASE}/move/position", json=move_data)
        if move_response.status_code != 200:
            print(f"✗ Failed to move to start position: {move_response.text}")
            return False

        print(f"✓ Robot at start position: X={start_x:.1f}, Y={start_y:.1f}, Z={z_height}")
        time.sleep(1)  # Brief pause

        # Step 2: Turn on gas
        print("\n🔥 Step 2: Turning on gas...")
        if not gas_on():
            print("⚠️  Gas control failed, aborting...")
            return False

        # Wait 5 seconds for gas stabilization
        print("⏳ Waiting 5 seconds for gas stabilization...")
        for i in range(5, 0, -1):
            print(f"  {i} seconds...", end='\r')
            time.sleep(1)
        print("  Gas stabilized!    ")

        # Step 3: Turn on laser
        print("\n⚡ Step 3: Turning on laser...")
        if not laser_on():
            print("⚠️  Laser control failed, shutting down...")
            gas_off()
            return False

        # Step 4: NOW start the circle path execution
        print("\n🚀 Step 4: Starting circle cutting path...")
        data = {
            "center_x": center_x,
            "center_y": center_y,
            "radius": radius,
            "z_height": z_height,
            "num_points": num_points,
            "num_cycles": num_cycles,
            "roll": 180,
            "pitch": 0,
            "yaw": 0
        }

        response = requests.post(f"{API_BASE}/path/circle/execute", json=data)
        if response.status_code != 200:
            print(f"✗ Execution failed: {response.text}")
            # Emergency shutdown
            laser_off()
            gas_off()
            return False

        result = response.json()
        print(f"✓ Circle cutting started!")
        print(f"  Number of points: {result['num_points']}")
        print(f"  Frequency: {result['frequency_hz']} Hz")
        print(f"  Estimated duration: {result['estimated_duration_sec']:.2f} seconds")

        # Give the path execution a moment to actually start
        time.sleep(1)

        # Monitor execution
        print("\n🔥 Laser cutting in progress...")
        start_time = time.time()

        while True:
            status_resp = requests.get(f"{API_BASE}/path/status")
            if status_resp.status_code == 200:
                status = status_resp.json()
                elapsed = time.time() - start_time

                if status['is_executing']:
                    progress = (elapsed / result['estimated_duration_sec']) * 100
                    print(f"  {elapsed:.1f}s / {result['estimated_duration_sec']:.1f}s ({min(progress, 100):.0f}%) - Cutting...", end='\r')
                    time.sleep(0.5)
                else:
                    print(f"\n✓ Circle cut completed in {elapsed:.1f} seconds!")
                    break
            else:
                print(f"\n⚠️  Status check failed")
                break

        # Turn off laser first
        laser_off()

        # Then turn off gas
        gas_off()

        print("\n✓ Laser cut circle complete!")
        return True

    except Exception as e:
        print(f"\n✗ Error during laser cut: {e}")
        # Emergency shutdown: laser off first, then gas
        print("⚠️  Attempting emergency laser shutoff...")
        laser_off()
        print("⚠️  Attempting emergency gas shutoff...")
        gas_off()
        return False


def execute_dxf_path(dxf_file_path, scale=1.0, offset_x=300, offset_y=0,
                     z_height=300, z_travel=350, points_per_unit=10):
    """Execute a path from a DXF file."""
    print("\n" + "=" * 60)
    print("⚠️  EXECUTING DXF PATH - ROBOT WILL MOVE!")
    print("=" * 60)
    print(f"DXF configuration:")
    print(f"  File: {dxf_file_path}")
    print(f"  Scale: {scale}x")
    print(f"  Offset: X={offset_x}, Y={offset_y}")
    print(f"  Z drawing height: {z_height}mm")
    print(f"  Z travel height: {z_travel}mm (pen up)")
    print(f"  Points per mm: {points_per_unit}")

    response = input("\n⚠️  Robot will trace DXF! Continue? [y/N]: ")
    if response.lower() != 'y':
        print("Cancelled by user")
        return False

    data = {
        "dxf_file_path": dxf_file_path,
        "scale": scale,
        "offset_x": offset_x,
        "offset_y": offset_y,
        "z_height": z_height,
        "z_travel": z_travel,
        "points_per_unit": points_per_unit,
        "roll": 180,
        "pitch": 0,
        "yaw": 0
    }

    try:
        response = requests.post(f"{API_BASE}/path/dxf/execute", json=data)
        if response.status_code == 200:
            result = response.json()
            print(f"\n✓ DXF path execution started!")
            print(f"  DXF file: {result['dxf_file']}")
            print(f"  Total points: {result['total_points']:,}")
            print(f"  Frequency: {result['frequency_hz']} Hz")
            print(f"  Estimated duration: {result['estimated_duration_sec']:.1f}s ({result['estimated_duration_sec']/60:.1f} min)")

            # Monitor execution
            print("\nMonitoring execution...")
            start_time = time.time()

            while True:
                status_resp = requests.get(f"{API_BASE}/path/status")
                if status_resp.status_code == 200:
                    status = status_resp.json()
                    elapsed = time.time() - start_time

                    if status['is_executing']:
                        print(f"  {elapsed:.1f}s - DXF tracing...", end='\r')
                        time.sleep(0.5)
                    else:
                        print(f"\n✓ DXF path completed in {elapsed:.1f} seconds!")
                        break
                else:
                    print(f"\n⚠️  Status check failed")
                    break

            return True
        else:
            print(f"✗ Execution failed: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def execute_laser_dxf_path(dxf_file_path, scale=1.0, offset_x=300, offset_y=0,
                           z_height=300, z_travel=350, points_per_unit=10,
                           gas_lead_time_ms=2000, laser_shutdown_delay_ms=500):
    """Execute a laser cutting path from a DXF file with gas control."""
    print("\n" + "=" * 60)
    print("🔥 LASER CUTTING DXF PATH - ROBOT AND LASER WILL ACTIVATE!")
    print("=" * 60)
    print(f"DXF configuration:")
    print(f"  File: {dxf_file_path}")
    print(f"  Scale: {scale}x")
    print(f"  Offset: X={offset_x}, Y={offset_y}")
    print(f"  Z cutting height: {z_height}mm")
    print(f"  Z travel height: {z_travel}mm")
    print(f"  Points per mm: {points_per_unit}")
    print(f"\nLaser safety sequence:")
    print(f"  1. Move to position with laser OFF")
    print(f"  2. Turn ON gas (wait {gas_lead_time_ms}ms for stabilization)")
    print(f"  3. Turn ON laser")
    print(f"  4. Execute cutting path")
    print(f"  5. Turn OFF laser")
    print(f"  6. Wait {laser_shutdown_delay_ms}ms")
    print(f"  7. Turn OFF gas")
    print(f"  8. Move to next position (repeat)")

    print("\n⚠️  WARNING: This will activate the laser and gas system!")
    print("⚠️  Ensure proper safety equipment and ventilation!")
    response = input("\n⚠️  Proceed with LASER CUTTING? [y/N]: ")
    if response.lower() != 'y':
        print("Cancelled by user")
        return False

    data = {
        "dxf_file_path": dxf_file_path,
        "scale": scale,
        "offset_x": offset_x,
        "offset_y": offset_y,
        "z_height": z_height,
        "z_travel": z_travel,
        "points_per_unit": points_per_unit,
        "roll": 180,
        "pitch": 0,
        "yaw": 0,
        "gas_lead_time_ms": gas_lead_time_ms,
        "laser_shutdown_delay_ms": laser_shutdown_delay_ms
    }

    try:
        response = requests.post(f"{API_BASE}/path/laser-dxf/execute", json=data)
        if response.status_code == 200:
            result = response.json()
            print(f"\n🔥 Laser DXF cutting started!")
            print(f"  DXF file: {result['dxf_file']}")
            print(f"  Total points: {result['total_points']:,}")
            print(f"  Laser control markers: {result['laser_markers']}")
            print(f"  Frequency: {result['frequency_hz']} Hz")
            print(f"  Estimated duration: {result['estimated_duration_sec']:.1f}s ({result['estimated_duration_sec']/60:.1f} min)")

            # Monitor execution
            print("\nMonitoring laser cutting...")
            print("⚠️  Do NOT interrupt during laser operation!")
            start_time = time.time()

            while True:
                status_resp = requests.get(f"{API_BASE}/path/status")
                if status_resp.status_code == 200:
                    status = status_resp.json()
                    elapsed = time.time() - start_time

                    if status['is_executing']:
                        print(f"  {elapsed:.1f}s - 🔥 Laser cutting in progress...", end='\r')
                        time.sleep(0.5)
                    else:
                        print(f"\n✓ Laser cutting completed in {elapsed:.1f} seconds!")
                        print("  Gas and laser have been shut down safely.")
                        break
                else:
                    print(f"\n⚠️  Status check failed")
                    print("⚠️  Laser and gas should be OFF - verify manually!")
                    break

            return True
        else:
            print(f"✗ Execution failed: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        print("⚠️  EMERGENCY: Verify laser and gas are OFF!")
        return False


def execute_grid_pattern(grid_rows=5, grid_cols=5, spacing=100, origin_x=200, origin_y=-200,
                         z_travel=300, z_pick=100, num_points_per_move=50):
    """Execute a grid pick-and-place pattern."""
    print("\n" + "=" * 60)
    print("⚠️  EXECUTING GRID PATTERN - ROBOT WILL MOVE!")
    print("=" * 60)
    print(f"Grid configuration:")
    print(f"  Grid size: {grid_rows}x{grid_cols} ({grid_rows * grid_cols} positions)")
    print(f"  Spacing: {spacing}mm")
    print(f"  Origin: X={origin_x}, Y={origin_y}")
    print(f"  Z travel height: {z_travel}mm")
    print(f"  Z pick/place height: {z_pick}mm")

    # Calculate workspace bounds
    max_x = origin_x + ((grid_cols - 1) * spacing)
    max_y = origin_y + ((grid_rows - 1) * spacing)
    print(f"\n  Workspace bounds:")
    print(f"    X: [{origin_x}, {max_x}]mm")
    print(f"    Y: [{origin_y}, {max_y}]mm")
    print(f"    Z: [{z_pick}, {z_travel}]mm")

    # Estimate duration
    total_positions = grid_rows * grid_cols
    points_per_position = num_points_per_move + 200 + 50 + 200  # travel + descend + pause + ascend
    total_points = total_positions * points_per_position
    estimated_duration = total_points / 100  # 100 Hz frequency
    print(f"\n  Estimated points: ~{total_points:,}")
    print(f"  Estimated duration: ~{estimated_duration:.1f}s ({estimated_duration/60:.1f} minutes)")
    print(f"  Note: Vertical movements are smooth (200 points = 2s per descend/ascend)")

    response = input("\n⚠️  Robot will move through grid! Continue? [y/N]: ")
    if response.lower() != 'y':
        print("Cancelled by user")
        return False

    data = {
        "grid_rows": grid_rows,
        "grid_cols": grid_cols,
        "spacing": spacing,
        "origin_x": origin_x,
        "origin_y": origin_y,
        "z_travel": z_travel,
        "z_pick": z_pick,
        "num_points_per_move": num_points_per_move,
        "roll": 180,
        "pitch": 0,
        "yaw": 0
    }

    try:
        response = requests.post(f"{API_BASE}/path/grid/execute", json=data)
        if response.status_code == 200:
            result = response.json()
            print(f"\n✓ Grid pattern execution started!")
            print(f"  Grid size: {result['grid_size']}")
            print(f"  Grid positions: {result['total_grid_positions']}")
            print(f"  Total points: {result['total_points']}")
            print(f"  Frequency: {result['frequency_hz']} Hz")
            print(f"  Estimated duration: {result['estimated_duration_sec']:.1f}s")

            # Monitor execution
            print("\nMonitoring execution...")
            start_time = time.time()

            while True:
                status_resp = requests.get(f"{API_BASE}/path/status")
                if status_resp.status_code == 200:
                    status = status_resp.json()
                    elapsed = time.time() - start_time

                    if status['is_executing']:
                        print(f"  {elapsed:.1f}s - Grid pattern executing...", end='\r')
                        time.sleep(0.5)
                    else:
                        print(f"\n✓ Grid pattern completed in {elapsed:.1f} seconds!")
                        break
                else:
                    print(f"\n⚠️  Status check failed")
                    break

            return True
        else:
            print(f"✗ Execution failed: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def stop_path():
    """Emergency stop current path execution."""
    print("\n🛑 Stopping path execution...")
    try:
        response = requests.post(f"{API_BASE}/path/stop")
        if response.status_code == 200:
            print("✓ Path stopped")
            return True
        else:
            print(f"✗ Stop failed: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def move_to_point():
    """Move robot to a specific XYZ point."""
    print("\n" + "=" * 60)
    print("Move to specific XYZ point")
    print("=" * 60)
    print("  SAFE WORKSPACE: X=[150-700mm], Y=[-500 to 500mm], Z=[100-700mm]")

    try:
        x_input = input("  X position (mm): ").strip()
        if not x_input:
            print("✗ X position required")
            return False
        x = float(x_input)

        y_input = input("  Y position (mm): ").strip()
        if not y_input:
            print("✗ Y position required")
            return False
        y = float(y_input)

        z_input = input("  Z position (mm): ").strip()
        if not z_input:
            print("✗ Z position required")
            return False
        z = float(z_input)

        speed_input = input("  Speed (mm/s, default 50): ").strip()
        speed = int(speed_input) if speed_input else 50

        print(f"\n  Target: X={x}, Y={y}, Z={z}")
        print(f"  Speed: {speed} mm/s")

        response = input("\n⚠️  Robot will move! Continue? [y/N]: ")
        if response.lower() != 'y':
            print("Cancelled by user")
            return False

        data = {
            "x": x,
            "y": y,
            "z": z,
            "roll": 180,
            "pitch": 0,
            "yaw": 0,
            "speed": speed,
            "wait": True
        }

        response = requests.post(f"{API_BASE}/move/position", json=data)
        if response.status_code == 200:
            print(f"✓ Moved to ({x}, {y}, {z})")
            return True
        else:
            print(f"✗ Move failed: {response.text}")
            return False
    except ValueError as e:
        print(f"✗ Invalid input: {e}")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def set_orientation():
    """Set robot orientation (yaw, pitch, roll) at current position."""
    print("\n" + "=" * 60)
    print("Set Robot Orientation (Euler Angles)")
    print("=" * 60)
    print("  Current default: Roll=180° (down), Pitch=0°, Yaw=0°")
    print("  Roll:  Rotation around X-axis (tool pointing direction)")
    print("  Pitch: Rotation around Y-axis (tilt forward/back)")
    print("  Yaw:   Rotation around Z-axis (rotate left/right)")

    try:
        roll_input = input("  Roll (degrees, default 180): ").strip()
        roll = float(roll_input) if roll_input else 180.0

        pitch_input = input("  Pitch (degrees, default 0): ").strip()
        pitch = float(pitch_input) if pitch_input else 0.0

        yaw_input = input("  Yaw (degrees, default 0): ").strip()
        yaw = float(yaw_input) if yaw_input else 0.0

        # Get current position
        status_response = requests.get(f"{API_BASE}/status", timeout=5)
        if status_response.status_code != 200 or not status_response.json().get('position'):
            print("✗ Could not get current position")
            return False

        current_pos = status_response.json()['position']
        x, y, z = current_pos[0], current_pos[1], current_pos[2]

        print(f"\n  Current position: X={x:.1f}, Y={y:.1f}, Z={z:.1f}")
        print(f"  New orientation: Roll={roll}°, Pitch={pitch}°, Yaw={yaw}°")

        response = input("\n⚠️  Robot will change orientation! Continue? [y/N]: ")
        if response.lower() != 'y':
            print("Cancelled by user")
            return False

        data = {
            "x": x,
            "y": y,
            "z": z,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "speed": 30,  # Slow speed for orientation changes
            "wait": True
        }

        response = requests.post(f"{API_BASE}/move/position", json=data)
        if response.status_code == 200:
            print(f"✓ Orientation set to Roll={roll}°, Pitch={pitch}°, Yaw={yaw}°")
            return True
        else:
            print(f"✗ Orientation change failed: {response.text}")
            return False
    except ValueError as e:
        print(f"✗ Invalid input: {e}")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def set_start_position():
    """Set/save a custom start position."""
    global START_POSITION

    print("\n" + "=" * 60)
    print("Set Custom Start Position")
    print("=" * 60)
    print(f"  Current start position: X={START_POSITION['x']}, Y={START_POSITION['y']}, Z={START_POSITION['z']}")
    print(f"  Current orientation: Roll={START_POSITION['roll']}°, Pitch={START_POSITION['pitch']}°, Yaw={START_POSITION['yaw']}°")
    print("\nOptions:")
    print("  1. Use current robot position as start")
    print("  2. Enter custom values")
    print("  3. Cancel")

    choice = input("\nChoice: ").strip()

    if choice == '1':
        # Get current position from robot
        try:
            status_response = requests.get(f"{API_BASE}/status", timeout=5)
            if status_response.status_code != 200 or not status_response.json().get('position'):
                print("✗ Could not get current position")
                return False

            current_pos = status_response.json()['position']
            START_POSITION['x'] = current_pos[0]
            START_POSITION['y'] = current_pos[1]
            START_POSITION['z'] = current_pos[2]
            START_POSITION['roll'] = current_pos[3]
            START_POSITION['pitch'] = current_pos[4]
            START_POSITION['yaw'] = current_pos[5]

            print(f"\n✓ Start position saved:")
            print(f"  Position: X={START_POSITION['x']:.1f}, Y={START_POSITION['y']:.1f}, Z={START_POSITION['z']:.1f}")
            print(f"  Orientation: Roll={START_POSITION['roll']:.1f}°, Pitch={START_POSITION['pitch']:.1f}°, Yaw={START_POSITION['yaw']:.1f}°")
            return True

        except Exception as e:
            print(f"✗ Error: {e}")
            return False

    elif choice == '2':
        # Enter custom values
        try:
            print("\nEnter position values (press Enter to keep current):")

            x_input = input(f"  X (mm, current {START_POSITION['x']}): ").strip()
            if x_input:
                START_POSITION['x'] = float(x_input)

            y_input = input(f"  Y (mm, current {START_POSITION['y']}): ").strip()
            if y_input:
                START_POSITION['y'] = float(y_input)

            z_input = input(f"  Z (mm, current {START_POSITION['z']}): ").strip()
            if z_input:
                START_POSITION['z'] = float(z_input)

            roll_input = input(f"  Roll (degrees, current {START_POSITION['roll']}): ").strip()
            if roll_input:
                START_POSITION['roll'] = float(roll_input)

            pitch_input = input(f"  Pitch (degrees, current {START_POSITION['pitch']}): ").strip()
            if pitch_input:
                START_POSITION['pitch'] = float(pitch_input)

            yaw_input = input(f"  Yaw (degrees, current {START_POSITION['yaw']}): ").strip()
            if yaw_input:
                START_POSITION['yaw'] = float(yaw_input)

            print(f"\n✓ Start position saved:")
            print(f"  Position: X={START_POSITION['x']:.1f}, Y={START_POSITION['y']:.1f}, Z={START_POSITION['z']:.1f}")
            print(f"  Orientation: Roll={START_POSITION['roll']:.1f}°, Pitch={START_POSITION['pitch']:.1f}°, Yaw={START_POSITION['yaw']:.1f}°")
            return True

        except ValueError as e:
            print(f"✗ Invalid input: {e}")
            return False
        except Exception as e:
            print(f"✗ Error: {e}")
            return False
    else:
        print("Cancelled")
        return False


def move_to_saved_start():
    """Move robot to the saved start position."""
    print("\n" + "=" * 60)
    print("Move to Saved Start Position")
    print("=" * 60)
    print(f"  Target position: X={START_POSITION['x']}, Y={START_POSITION['y']}, Z={START_POSITION['z']}")
    print(f"  Target orientation: Roll={START_POSITION['roll']}°, Pitch={START_POSITION['pitch']}°, Yaw={START_POSITION['yaw']}°")

    response = input("\n⚠️  Robot will move! Continue? [y/N]: ")
    if response.lower() != 'y':
        print("Cancelled by user")
        return False

    try:
        data = {
            "x": START_POSITION['x'],
            "y": START_POSITION['y'],
            "z": START_POSITION['z'],
            "roll": START_POSITION['roll'],
            "pitch": START_POSITION['pitch'],
            "yaw": START_POSITION['yaw'],
            "speed": 100,
            "wait": True
        }

        response = requests.post(f"{API_BASE}/move/position", json=data)
        if response.status_code == 200:
            print("✓ Moved to saved start position")
            return True
        else:
            print(f"✗ Move failed: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def move_home():
    """Move robot to home position."""
    print("\n" + "=" * 60)
    print("Moving to home position...")
    print("=" * 60)

    response = input("Move to home? [y/N]: ")
    if response.lower() != 'y':
        print("Cancelled by user")
        return False

    try:
        response = requests.post(f"{API_BASE}/move/home")
        if response.status_code == 200:
            print("✓ Moved to home")
            return True
        else:
            print(f"✗ Home failed: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def main():
    print("\n" + "=" * 60)
    print("xArm Real Path Execution Script")
    print("=" * 60)
    print("\n⚠️  WARNING: This will move the physical robot!")
    print("Make sure:")
    print("  • Workspace is clear of obstacles")
    print("  • Emergency stop is accessible")
    print("  • Robot is properly mounted")
    print("  • You're ready to press emergency stop if needed")

    response = input("\nReady to proceed? [y/N]: ")
    if response.lower() != 'y':
        print("\nExiting safely.")
        return

    # Step 1: Check connection
    if not check_connection():
        print("\n❌ Connection failed. Please fix and try again.")
        return

    time.sleep(1)

    # Step 2: Enable robot
    if not enable_robot():
        print("\n❌ Failed to enable robot.")
        return

    # Step 3: Move to start position
    print("\nThe robot will first move to a safe starting position.")
    if not move_to_start_position(x=300, y=0, z=300):
        print("\n❌ Failed to move to start position.")
        return

    # Main menu
    while True:
        print("\n" + "=" * 60)
        print("Select a path to execute:")
        print("=" * 60)
        print("PATH EXECUTION (DRY RUN):")
        print("  1. Simple wave pattern (safe)")
        print("  2. Circle path (configurable radius)")
        print("  3. S-shape curve")
        print("  4. Grid pick-and-place pattern (5x5)")
        print("  5. DXF file tracing (pen plotter)")
        print("  6. Custom path (advanced)")
        print("\nLASER CUTTING (GAS CONTROL):")
        print("  L. Laser cut circle 🔥")
        print("  D. Laser cut DXF file 🔥📐")
        print("\nMANUAL CONTROL:")
        print("  7. Move to XYZ point")
        print("  8. Set orientation (yaw, pitch, roll)")
        print("  S. Set start position")
        print("  M. Move to saved start position")
        print("  H. Move to home position")
        print("\nUTILITY:")
        print("  0. Emergency stop current path")
        print("  X. Exit")

        choice = input("\nChoice: ").strip()

        if choice == '1':
            # Simple wave
            control_points = [
                {"x": 300, "y": 0},
                {"x": 350, "y": 30},
                {"x": 400, "y": 0},
                {"x": 450, "y": -30},
                {"x": 500, "y": 0}
            ]

            if preview_path(control_points, z_height=300, spline_type="cubic"):
                execute_spline_path(control_points, z_height=300, spline_type="cubic", num_samples=200)

        elif choice == '2':
            # Circle path with safer defaults
            print("\nCircle configuration:")
            print("  SAFE WORKSPACE: X=[150-700mm], Y=[-500 to 500mm], Z=[100-700mm]")
            print("  Recommended: Center near (350, 0), Radius 20-30mm for testing")

            radius_input = input("  Radius (mm, default 25): ").strip()
            radius = float(radius_input) if radius_input else 25

            center_x_input = input("  Center X (mm, default 350): ").strip()
            center_x = float(center_x_input) if center_x_input else 350

            center_y_input = input("  Center Y (mm, default 0): ").strip()
            center_y = float(center_y_input) if center_y_input else 0

            z_height_input = input("  Z height (mm, default 300): ").strip()
            z_height = float(z_height_input) if z_height_input else 300

            num_points_input = input("  Number of points (default 120): ").strip()
            num_points = int(num_points_input) if num_points_input else 120

            num_cycles_input = input("  Number of cycles (default 1, 0 = infinite): ").strip()
            num_cycles = int(num_cycles_input) if num_cycles_input else 1

            # Convert 0 to very large number for "infinite" cycles
            if num_cycles == 0:
                num_cycles = 999999
                print("  ℹ️  Infinite cycles enabled - use option 6 to stop")

            circumference = 2 * 3.14159 * radius
            point_spacing = circumference / num_points

            # Validate workspace
            if center_x - radius < 150 or center_x + radius > 700:
                print(f"\n  ⚠️  WARNING: X range [{center_x-radius:.1f} to {center_x+radius:.1f}] may be out of bounds [150-700mm]")
            if center_y - radius < -500 or center_y + radius > 500:
                print(f"\n  ⚠️  WARNING: Y range [{center_y-radius:.1f} to {center_y+radius:.1f}] may be out of bounds [-500 to 500mm]")

            print(f"\n  Circle: radius={radius}mm at ({center_x}, {center_y}), Z={z_height}")
            print(f"  Circumference: {circumference:.1f}mm, Points: {num_points}")
            print(f"  Point spacing: {point_spacing:.1f}mm")
            if num_cycles < 999999:
                print(f"  Total cycles: {num_cycles}")
                print(f"  Estimated duration: {(num_points * num_cycles) / 100:.1f}s")
            else:
                print(f"  Total cycles: ∞ (infinite)")

            # Provide feedback on point density
            if point_spacing > 5.0:
                print(f"  ℹ️  Points are far apart ({point_spacing:.1f}mm). Motion may be jerky.")
                print(f"     Consider increasing points to {int(circumference / 2.5)} for smoother motion.")
            elif point_spacing < 1.0:
                print(f"  ℹ️  Very dense points ({point_spacing:.1f}mm). Good for smooth motion but slower.")

            execute_circle_path(center_x=center_x, center_y=center_y, radius=radius,
                              z_height=z_height, num_points=num_points, num_cycles=num_cycles)

        elif choice == '3':
            # S-shape
            control_points = [
                {"x": 300, "y": 50},
                {"x": 350, "y": 25},
                {"x": 400, "y": 0},
                {"x": 450, "y": -25},
                {"x": 500, "y": -50}
            ]

            if preview_path(control_points, z_height=300, spline_type="cubic"):
                execute_spline_path(control_points, z_height=300, spline_type="cubic", num_samples=200)

        elif choice == '4':
            # Grid pick-and-place pattern
            print("\nGrid pattern configuration:")
            print("  SAFE WORKSPACE: X=[150-700mm], Y=[-500 to 500mm], Z=[100-700mm]")
            print("  Recommended: 5x5 grid with 100mm spacing")

            grid_rows_input = input("  Grid rows (default 5): ").strip()
            grid_rows = int(grid_rows_input) if grid_rows_input else 5

            grid_cols_input = input("  Grid columns (default 5): ").strip()
            grid_cols = int(grid_cols_input) if grid_cols_input else 5

            spacing_input = input("  Spacing (mm, default 100): ").strip()
            spacing = float(spacing_input) if spacing_input else 100.0

            origin_x_input = input("  Origin X (mm, default 200): ").strip()
            origin_x = float(origin_x_input) if origin_x_input else 200.0

            origin_y_input = input("  Origin Y (mm, default -200): ").strip()
            origin_y = float(origin_y_input) if origin_y_input else -200.0

            z_travel_input = input("  Travel height Z (mm, default 300): ").strip()
            z_travel = float(z_travel_input) if z_travel_input else 300.0

            z_pick_input = input("  Pick/place height Z (mm, default 100): ").strip()
            z_pick = float(z_pick_input) if z_pick_input else 100.0

            execute_grid_pattern(
                grid_rows=grid_rows,
                grid_cols=grid_cols,
                spacing=spacing,
                origin_x=origin_x,
                origin_y=origin_y,
                z_travel=z_travel,
                z_pick=z_pick
            )

        elif choice == '5':
            # DXF file tracing
            print("\nDXF file tracing configuration:")
            print("  Note: Traces LINE and POLYLINE entities from DXF")

            dxf_file_path = input("  DXF file path: ").strip()

            if not dxf_file_path:
                print("  ⚠️  No file path provided")
                continue

            scale_input = input("  Scale factor (default 1.0): ").strip()
            scale = float(scale_input) if scale_input else 1.0

            offset_x_input = input("  Offset X (mm, default 300): ").strip()
            offset_x = float(offset_x_input) if offset_x_input else 300.0

            offset_y_input = input("  Offset Y (mm, default 0): ").strip()
            offset_y = float(offset_y_input) if offset_y_input else 0.0

            z_height_input = input("  Drawing height Z (mm, default 300): ").strip()
            z_height = float(z_height_input) if z_height_input else 300.0

            z_travel_input = input("  Travel height Z (mm, default 350): ").strip()
            z_travel = float(z_travel_input) if z_travel_input else 350.0

            points_per_unit_input = input("  Points per mm (default 10): ").strip()
            points_per_unit = int(points_per_unit_input) if points_per_unit_input else 10

            execute_dxf_path(
                dxf_file_path=dxf_file_path,
                scale=scale,
                offset_x=offset_x,
                offset_y=offset_y,
                z_height=z_height,
                z_travel=z_travel,
                points_per_unit=points_per_unit
            )

        elif choice == '6':
            # Custom path
            print("\nDefine your custom path:")
            print("Enter control points as X,Y pairs (one per line)")
            print("Type 'done' when finished")

            control_points = []
            while True:
                point_str = input(f"Point {len(control_points) + 1} (X,Y or 'done'): ").strip()
                if point_str.lower() == 'done':
                    break
                try:
                    x, y = map(float, point_str.split(','))
                    control_points.append({"x": x, "y": y})
                    print(f"  Added: X={x}, Y={y}")
                except:
                    print("  Invalid format. Use: X,Y (e.g., 300,50)")

            if len(control_points) >= 2:
                z_height = float(input("Z height (mm, default 300): ") or "300")
                spline_type = input("Spline type (cubic/quadratic/linear, default cubic): ").strip() or "cubic"

                if preview_path(control_points, z_height=z_height, spline_type=spline_type):
                    execute_spline_path(control_points, z_height=z_height, spline_type=spline_type)
            else:
                print("Need at least 2 control points")

        elif choice == '7':
            move_to_point()

        elif choice == '8':
            set_orientation()

        elif choice.upper() == 'L':
            # Laser cut circle with gas control
            print("\n🔥 Laser Cut Circle Configuration:")
            print("  SAFE WORKSPACE: X=[150-700mm], Y=[-500 to 500mm], Z=[100-700mm]")
            print("  Recommended: Center near (350, 0), Radius 20-30mm for testing")

            radius_input = input("  Radius (mm, default 25): ").strip()
            radius = float(radius_input) if radius_input else 25

            center_x_input = input("  Center X (mm, default 350): ").strip()
            center_x = float(center_x_input) if center_x_input else 350

            center_y_input = input("  Center Y (mm, default 0): ").strip()
            center_y = float(center_y_input) if center_y_input else 0

            z_height_input = input("  Z height (mm, default 300): ").strip()
            z_height = float(z_height_input) if z_height_input else 300

            num_points_input = input("  Number of points (default 120): ").strip()
            num_points = int(num_points_input) if num_points_input else 120

            num_cycles_input = input("  Number of cycles (default 1): ").strip()
            num_cycles = int(num_cycles_input) if num_cycles_input else 1

            # Validate workspace
            if center_x - radius < 150 or center_x + radius > 700:
                print(f"\n  ⚠️  WARNING: X range [{center_x-radius:.1f} to {center_x+radius:.1f}] may be out of bounds [150-700mm]")
            if center_y - radius < -500 or center_y + radius > 500:
                print(f"\n  ⚠️  WARNING: Y range [{center_y-radius:.1f} to {center_y+radius:.1f}] may be out of bounds [-500 to 500mm]")

            execute_laser_cut_circle(
                center_x=center_x,
                center_y=center_y,
                radius=radius,
                z_height=z_height,
                num_points=num_points,
                num_cycles=num_cycles
            )

        elif choice.upper() == 'D':
            # Laser cut DXF file
            print("\n🔥 Laser Cut DXF Configuration:")
            print("  SAFE WORKSPACE: Keep all points within 560mm radius for best results")
            print("  Use validate_dxf.py to check your file first!")

            dxf_path = input("\n  DXF file path: ").strip()
            if not dxf_path:
                print("  No file specified, cancelled")
                continue

            scale_input = input("  Scale factor (default 1.0): ").strip()
            scale = float(scale_input) if scale_input else 1.0

            offset_x_input = input("  Offset X (mm, default 300): ").strip()
            offset_x = float(offset_x_input) if offset_x_input else 300.0

            offset_y_input = input("  Offset Y (mm, default 0): ").strip()
            offset_y = float(offset_y_input) if offset_y_input else 0.0

            z_height_input = input("  Z cutting height (mm, default 300): ").strip()
            z_height = float(z_height_input) if z_height_input else 300.0

            z_travel_input = input("  Z travel height (mm, default 350): ").strip()
            z_travel = float(z_travel_input) if z_travel_input else 350.0

            points_input = input("  Points per mm (default 10): ").strip()
            points_per_unit = int(points_input) if points_input else 10

            execute_laser_dxf_path(
                dxf_file_path=dxf_path,
                scale=scale,
                offset_x=offset_x,
                offset_y=offset_y,
                z_height=z_height,
                z_travel=z_travel,
                points_per_unit=points_per_unit
            )

        elif choice.upper() == 'S':
            set_start_position()

        elif choice.upper() == 'M':
            move_to_saved_start()

        elif choice.upper() == 'H':
            move_home()

        elif choice == '0':
            stop_path()

        elif choice.upper() == 'X':
            print("\nExiting...")
            break

        else:
            print("Invalid choice")

    print("\n" + "=" * 60)
    print("Session complete!")
    print("=" * 60)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🛑 Interrupted by user!")
        print("Attempting emergency stop...")
        stop_path()
        print("Attempting emergency laser shutoff...")
        laser_off()
        print("Attempting emergency gas shutoff...")
        gas_off()
        print("\nExited safely.")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        print("Attempting emergency stop...")
        stop_path()
        print("Attempting emergency laser shutoff...")
        laser_off()
        print("Attempting emergency gas shutoff...")
        gas_off()
