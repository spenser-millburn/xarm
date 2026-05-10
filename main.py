#!/usr/bin/env python3
from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pydantic import BaseModel
from xarm.wrapper import XArmAPI
import uvicorn
from typing import List, Optional
import time
import numpy as np
from scipy import interpolate
import asyncio
import threading
import ezdxf
from pathlib import Path
import serial
import serial.tools.list_ports

app = FastAPI(title="xArm6 Control API")

# xArm connection settings
XARM_IP = "192.168.1.213"
arm: Optional[XArmAPI] = None

# Serial port configuration for laser gas control
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600
serial_connection = None

def send_gas_command(command: str) -> bool:
    """Send command to gas control system via serial."""
    global serial_connection
    try:
        if serial_connection is None or not serial_connection.is_open:
            serial_connection = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(0.1)

        serial_connection.write((command + '\n').encode())
        time.sleep(0.05)
        return True
    except Exception as e:
        print(f"Serial communication error: {e}")
        return False

def gas_on():
    """Turn on the gas for laser cutting."""
    print("[GAS] Turning gas ON...")
    return send_gas_command("gas on")

def gas_off():
    """Turn off the gas."""
    print("[GAS] Turning gas OFF...")
    return send_gas_command("gas off")

def laser_on():
    """Turn on the laser."""
    print("[LASER] Turning LASER ON...")
    return send_gas_command("onkey")

def laser_off():
    """Turn off the laser."""
    print("[LASER] Turning LASER OFF...")
    return send_gas_command("offkey")

# Pydantic models for request/response
class Position(BaseModel):
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    speed: Optional[int] = 100
    wait: Optional[bool] = True

class JointAngles(BaseModel):
    angles: List[float]
    speed: Optional[int] = 50
    wait: Optional[bool] = True

class GridPosition(BaseModel):
    row: int
    col: int
    origin_x: float
    origin_y: float
    origin_z: float
    spacing_x: float
    spacing_y: float
    height: float
    speed: Optional[int] = 100

class GridPatternPath(BaseModel):
    grid_rows: int = 5  # Number of rows
    grid_cols: int = 5  # Number of columns
    spacing: float = 100.0  # Spacing between grid points in mm
    origin_x: float = 200.0  # Origin X position
    origin_y: float = -200.0  # Origin Y position
    z_travel: float = 300.0  # Z height for travel between points
    z_pick: float = 100.0  # Z height for pick/place operations
    num_points_per_move: int = 50  # Points to interpolate between grid positions
    roll: float = 180.0
    pitch: float = 0.0
    yaw: float = 0.0

class DXFPathRequest(BaseModel):
    dxf_file_path: str  # Path to DXF file
    scale: float = 1.0  # Scale factor (1.0 = no scaling)
    offset_x: float = 300.0  # X offset for positioning
    offset_y: float = 0.0  # Y offset for positioning
    z_height: float = 300.0  # Z height for drawing
    z_travel: float = 350.0  # Z height for travel (pen up)
    points_per_unit: int = 10  # Points to interpolate per mm of line
    roll: float = 180.0
    pitch: float = 0.0
    yaw: float = 0.0

class LaserDXFPathRequest(BaseModel):
    dxf_file_path: str  # Path to DXF file
    scale: float = 1.0  # Scale factor (1.0 = no scaling)
    offset_x: float = 300.0  # X offset for positioning
    offset_y: float = 0.0  # Y offset for positioning
    z_height: float = 300.0  # Z height for laser cutting
    z_travel: float = 350.0  # Z height for travel (laser off)
    points_per_unit: int = 10  # Points to interpolate per mm of line
    roll: float = 180.0  # 180 = pointing down, 0 = pointing up, adjust if needed
    pitch: float = 0.0
    yaw: float = 0.0
    gas_lead_time_ms: int = 2000  # Time to wait for gas to stabilize before laser on
    laser_shutdown_delay_ms: int = 500  # Time between laser off and gas off

class SplineControlPoint(BaseModel):
    x: float
    y: float

class SplinePathRequest(BaseModel):
    control_points: List[SplineControlPoint]
    z_height: float
    num_samples: int = 200  # Points per second at 200 Hz
    roll: float = 180.0
    pitch: float = 0.0
    yaw: float = 0.0
    spline_type: str = "cubic"  # cubic, quadratic, or linear

class CirclePath(BaseModel):
    center_x: float
    center_y: float
    radius: float
    z_height: float
    num_points: int = 100
    num_cycles: int = 1  # Number of times to repeat the circle
    roll: float = 180.0
    pitch: float = 0.0
    yaw: float = 0.0

class StatusResponse(BaseModel):
    connected: bool
    state: int
    state_name: str
    mode: int
    error_code: int
    warn_code: int
    position: Optional[List[float]]
    joint_angles: Optional[List[float]]
    is_moving: bool

# Global flag to stop path execution
path_execution_active = False
path_execution_lock = threading.Lock()

# Helper function to get arm connection
def get_arm():
    global arm
    if arm is None:
        try:
            arm = XArmAPI(XARM_IP)
            time.sleep(0.5)
            # Clean any warnings/errors on connect
            if arm.warn_code != 0:
                arm.clean_warn()
            if arm.error_code != 0:
                arm.clean_error()
        except Exception as e:
            raise HTTPException(status_code=503, detail=f"Failed to connect to xArm: {str(e)}")
    return arm

# Spline generation functions
def generate_spline_path(control_points: List[SplineControlPoint], num_samples: int, spline_type: str = "cubic"):
    """
    Generate a smooth 2D spline path from control points.
    Returns arrays of x and y coordinates.
    """
    if len(control_points) < 2:
        raise ValueError("Need at least 2 control points")

    # Extract x and y coordinates
    x_points = np.array([p.x for p in control_points])
    y_points = np.array([p.y for p in control_points])

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

def generate_circle_path(center_x: float, center_y: float, radius: float, num_points: int):
    """
    Generate a circular path.
    Returns arrays of x and y coordinates.
    """
    theta = np.linspace(0, 2*np.pi, num_points)
    x = center_x + radius * np.cos(theta)
    y = center_y + radius * np.sin(theta)
    return x, y

def parse_dxf_to_path(dxf_file_path: str, scale: float, offset_x: float, offset_y: float,
                      z_height: float, z_travel: float, points_per_unit: int):
    """
    Parse a DXF file and convert it to a 3D path with pen up/down movements.

    Returns:
        x_path, y_path, z_path: Arrays of coordinates
    """
    print(f"[DXF] Loading file: {dxf_file_path}")

    try:
        doc = ezdxf.readfile(dxf_file_path)
        msp = doc.modelspace()

        x_points = []
        y_points = []
        z_points = []

        # Track entity types found
        entity_types = {}

        # Process entities in the DXF file
        for entity in msp:
            entity_type = entity.dxftype()
            entity_types[entity_type] = entity_types.get(entity_type, 0) + 1

            if entity_type == 'LINE':
                # Extract line endpoints
                start = entity.dxf.start
                end = entity.dxf.end

                # Calculate number of interpolation points based on line length
                length = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
                num_points = max(2, int(length * points_per_unit))

                # If not continuing from previous point, lift pen and move
                if len(x_points) == 0 or \
                   abs(x_points[-1] - (start[0] * scale + offset_x)) > 0.1 or \
                   abs(y_points[-1] - (start[1] * scale + offset_y)) > 0.1:

                    # Lift pen (if not first point)
                    if len(x_points) > 0:
                        # Move up (SLOW)
                        lift_points = 200  # Increased from 50 to 200 for smoother pen lift
                        x_points.extend([x_points[-1]] * lift_points)
                        y_points.extend([y_points[-1]] * lift_points)
                        z_points.extend(np.linspace(z_points[-1], z_travel, lift_points))

                        # Travel to new position
                        travel_points = 100  # Increased from 30 for smoother travel
                        x_points.extend(np.linspace(x_points[-1], start[0] * scale + offset_x, travel_points))
                        y_points.extend(np.linspace(y_points[-1], start[1] * scale + offset_y, travel_points))
                        z_points.extend([z_travel] * travel_points)

                    # Move down to drawing height (SLOW)
                    lower_points = 200  # Increased from 50 to 200 for smoother pen lower
                    if len(x_points) == 0:
                        # First point
                        x_points.extend([start[0] * scale + offset_x] * lower_points)
                        y_points.extend([start[1] * scale + offset_y] * lower_points)
                        z_points.extend(np.linspace(z_travel, z_height, lower_points))
                    else:
                        x_points.extend([start[0] * scale + offset_x] * lower_points)
                        y_points.extend([start[1] * scale + offset_y] * lower_points)
                        z_points.extend(np.linspace(z_travel, z_height, lower_points))

                # Draw the line
                x_line = np.linspace(start[0], end[0], num_points) * scale + offset_x
                y_line = np.linspace(start[1], end[1], num_points) * scale + offset_y
                z_line = np.full(num_points, z_height)

                x_points.extend(x_line[1:])  # Skip first point if continuing
                y_points.extend(y_line[1:])
                z_points.extend(z_line[1:])

            elif entity.dxftype() == 'LWPOLYLINE' or entity.dxftype() == 'POLYLINE':
                # Handle polylines
                if entity.dxftype() == 'LWPOLYLINE':
                    points = list(entity.get_points())
                else:  # POLYLINE
                    # For old-style POLYLINE, iterate through vertices
                    points = [(v.dxf.location.x, v.dxf.location.y) for v in entity.vertices]

                for i in range(len(points) - 1):
                    start = points[i]
                    end = points[i + 1]

                    length = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
                    num_points = max(2, int(length * points_per_unit))

                    # Move to start if needed (pen up/down)
                    if i == 0 and (len(x_points) == 0 or \
                       abs(x_points[-1] - (start[0] * scale + offset_x)) > 0.1 or \
                       abs(y_points[-1] - (start[1] * scale + offset_y)) > 0.1):

                        if len(x_points) > 0:
                            # Lift, travel, lower
                            lift_points = 50
                            x_points.extend([x_points[-1]] * lift_points)
                            y_points.extend([y_points[-1]] * lift_points)
                            z_points.extend(np.linspace(z_height, z_travel, lift_points))

                            travel_points = 30
                            x_points.extend(np.linspace(x_points[-1], start[0] * scale + offset_x, travel_points))
                            y_points.extend(np.linspace(y_points[-1], start[1] * scale + offset_y, travel_points))
                            z_points.extend([z_travel] * travel_points)

                            lower_points = 50
                            x_points.extend([start[0] * scale + offset_x] * lower_points)
                            y_points.extend([start[1] * scale + offset_y] * lower_points)
                            z_points.extend(np.linspace(z_travel, z_height, lower_points))
                        else:
                            # First point
                            lower_points = 50
                            x_points.extend([start[0] * scale + offset_x] * lower_points)
                            y_points.extend([start[1] * scale + offset_y] * lower_points)
                            z_points.extend(np.linspace(z_travel, z_height, lower_points))

                    # Draw line segment
                    x_line = np.linspace(start[0], end[0], num_points) * scale + offset_x
                    y_line = np.linspace(start[1], end[1], num_points) * scale + offset_y
                    z_line = np.full(num_points, z_height)

                    if i == 0 and len(x_points) > 0:
                        x_points.extend(x_line[1:])
                        y_points.extend(y_line[1:])
                        z_points.extend(z_line[1:])
                    else:
                        x_points.extend(x_line)
                        y_points.extend(y_line)
                        z_points.extend(z_line)

            elif entity_type == 'SPLINE':
                # Handle splines by flattening them into line segments
                try:
                    # Get flattened spline points with very fine resolution for smooth motion
                    # Use smaller distance for more points = smoother motion
                    flattened = list(entity.flattening(distance=0.5 / points_per_unit))

                    if len(flattened) < 2:
                        continue

                    # Move to start if needed (pen up/down)
                    start = flattened[0]
                    if len(x_points) == 0 or \
                       abs(x_points[-1] - (start[0] * scale + offset_x)) > 0.1 or \
                       abs(y_points[-1] - (start[1] * scale + offset_y)) > 0.1:

                        if len(x_points) > 0:
                            # Lift, travel, lower
                            lift_points = 50
                            x_points.extend([x_points[-1]] * lift_points)
                            y_points.extend([y_points[-1]] * lift_points)
                            z_points.extend(np.linspace(z_height, z_travel, lift_points))

                            travel_points = 30
                            x_points.extend(np.linspace(x_points[-1], start[0] * scale + offset_x, travel_points))
                            y_points.extend(np.linspace(y_points[-1], start[1] * scale + offset_y, travel_points))
                            z_points.extend([z_travel] * travel_points)

                            lower_points = 50
                            x_points.extend([start[0] * scale + offset_x] * lower_points)
                            y_points.extend([start[1] * scale + offset_y] * lower_points)
                            z_points.extend(np.linspace(z_travel, z_height, lower_points))
                        else:
                            # First point
                            lower_points = 50
                            x_points.extend([start[0] * scale + offset_x] * lower_points)
                            y_points.extend([start[1] * scale + offset_y] * lower_points)
                            z_points.extend(np.linspace(z_travel, z_height, lower_points))

                    # Add all flattened points
                    for pt in flattened:
                        x_points.append(pt[0] * scale + offset_x)
                        y_points.append(pt[1] * scale + offset_y)
                        z_points.append(z_height)

                except Exception as e:
                    print(f"[DXF] Warning: Failed to process SPLINE entity: {e}")
                    continue

        # Print entity type summary
        print(f"[DXF] Entity types found: {entity_types}")

        # Lift pen at end
        if len(x_points) > 0:
            lift_points = 50
            x_points.extend([x_points[-1]] * lift_points)
            y_points.extend([y_points[-1]] * lift_points)
            z_points.extend(np.linspace(z_height, z_travel, lift_points))

        if len(x_points) == 0:
            raise ValueError(f"No valid entities found in DXF. Found: {entity_types}")

        print(f"[DXF] Parsed {len(x_points)} points from DXF")
        return np.array(x_points), np.array(y_points), np.array(z_points)

    except Exception as e:
        print(f"[DXF] Error parsing file: {e}")
        raise ValueError(f"Failed to parse DXF file: {e}")

def parse_dxf_to_laser_path(dxf_file_path: str, scale: float, offset_x: float, offset_y: float,
                             z_height: float, z_travel: float, points_per_unit: int):
    """
    Parse a DXF file for laser cutting and convert it to a 3D path with laser control markers.
    Returns x_path, y_path, z_path, and laser_markers (list of indices where laser state changes)

    laser_markers format: list of tuples (index, action)
        action can be: 'gas_on', 'laser_on', 'laser_off', 'gas_off'
    """
    print(f"[LASER-DXF] Loading file: {dxf_file_path}")

    try:
        doc = ezdxf.readfile(dxf_file_path)
        msp = doc.modelspace()

        x_points = []
        y_points = []
        z_points = []
        laser_markers = []  # (index, action) tuples

        entity_types = {}

        # Process entities in the DXF file
        for entity in msp:
            entity_type = entity.dxftype()
            entity_types[entity_type] = entity_types.get(entity_type, 0) + 1

            if entity_type == 'LINE':
                start = entity.dxf.start
                end = entity.dxf.end

                length = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
                num_points = max(2, int(length * points_per_unit))

                # Check if we need to travel to a new starting position
                need_travel = (len(x_points) == 0 or
                              abs(x_points[-1] - (start[0] * scale + offset_x)) > 0.1 or
                              abs(y_points[-1] - (start[1] * scale + offset_y)) > 0.1)

                if need_travel:
                    if len(x_points) > 0:
                        # LASER OFF before moving
                        laser_markers.append((len(x_points), 'laser_off'))

                        # Wait for laser to fully turn off (add dwell points)
                        dwell_points = 25
                        x_points.extend([x_points[-1]] * dwell_points)
                        y_points.extend([y_points[-1]] * dwell_points)
                        z_points.extend([z_points[-1]] * dwell_points)

                        # GAS OFF after laser is off
                        laser_markers.append((len(x_points), 'gas_off'))

                        # Lift to travel height
                        lift_points = 50
                        x_points.extend([x_points[-1]] * lift_points)
                        y_points.extend([y_points[-1]] * lift_points)
                        z_points.extend(np.linspace(z_points[-1], z_travel, lift_points))

                        # Travel to new position
                        travel_points = 100
                        x_points.extend(np.linspace(x_points[-1], start[0] * scale + offset_x, travel_points))
                        y_points.extend(np.linspace(y_points[-1], start[1] * scale + offset_y, travel_points))
                        z_points.extend([z_travel] * travel_points)

                    # Lower to cutting height
                    lower_points = 50
                    if len(x_points) == 0:
                        x_points.extend([start[0] * scale + offset_x] * lower_points)
                        y_points.extend([start[1] * scale + offset_y] * lower_points)
                        z_points.extend(np.linspace(z_travel, z_height, lower_points))
                    else:
                        x_points.extend([start[0] * scale + offset_x] * lower_points)
                        y_points.extend([start[1] * scale + offset_y] * lower_points)
                        z_points.extend(np.linspace(z_travel, z_height, lower_points))

                    # GAS ON at cutting height
                    laser_markers.append((len(x_points), 'gas_on'))

                    # Wait for gas to stabilize (dwell at position)
                    gas_stabilize_points = 100
                    x_points.extend([x_points[-1]] * gas_stabilize_points)
                    y_points.extend([y_points[-1]] * gas_stabilize_points)
                    z_points.extend([z_points[-1]] * gas_stabilize_points)

                    # LASER ON after gas is flowing
                    laser_markers.append((len(x_points), 'laser_on'))

                # Cut the line
                x_line = np.linspace(start[0], end[0], num_points) * scale + offset_x
                y_line = np.linspace(start[1], end[1], num_points) * scale + offset_y
                z_line = np.full(num_points, z_height)

                x_points.extend(x_line[1:])  # Skip first point if continuing
                y_points.extend(y_line[1:])
                z_points.extend(z_line[1:])

            elif entity.dxftype() == 'LWPOLYLINE' or entity.dxftype() == 'POLYLINE':
                if entity.dxftype() == 'LWPOLYLINE':
                    points = list(entity.get_points())
                else:
                    points = [(v.dxf.location.x, v.dxf.location.y) for v in entity.vertices]

                for i in range(len(points) - 1):
                    start = points[i]
                    end = points[i + 1]

                    length = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
                    num_points = max(2, int(length * points_per_unit))

                    # Check if first segment needs travel
                    if i == 0 and (len(x_points) == 0 or
                                  abs(x_points[-1] - (start[0] * scale + offset_x)) > 0.1 or
                                  abs(y_points[-1] - (start[1] * scale + offset_y)) > 0.1):

                        if len(x_points) > 0:
                            # LASER OFF, wait, GAS OFF, lift, travel, lower
                            laser_markers.append((len(x_points), 'laser_off'))

                            dwell_points = 25
                            x_points.extend([x_points[-1]] * dwell_points)
                            y_points.extend([y_points[-1]] * dwell_points)
                            z_points.extend([z_points[-1]] * dwell_points)

                            laser_markers.append((len(x_points), 'gas_off'))

                            lift_points = 50
                            x_points.extend([x_points[-1]] * lift_points)
                            y_points.extend([y_points[-1]] * lift_points)
                            z_points.extend(np.linspace(z_height, z_travel, lift_points))

                            travel_points = 100
                            x_points.extend(np.linspace(x_points[-1], start[0] * scale + offset_x, travel_points))
                            y_points.extend(np.linspace(y_points[-1], start[1] * scale + offset_y, travel_points))
                            z_points.extend([z_travel] * travel_points)

                            lower_points = 50
                            x_points.extend([start[0] * scale + offset_x] * lower_points)
                            y_points.extend([start[1] * scale + offset_y] * lower_points)
                            z_points.extend(np.linspace(z_travel, z_height, lower_points))
                        else:
                            # First point
                            lower_points = 50
                            x_points.extend([start[0] * scale + offset_x] * lower_points)
                            y_points.extend([start[1] * scale + offset_y] * lower_points)
                            z_points.extend(np.linspace(z_travel, z_height, lower_points))

                        # GAS ON, wait, LASER ON
                        laser_markers.append((len(x_points), 'gas_on'))

                        gas_stabilize_points = 100
                        x_points.extend([x_points[-1]] * gas_stabilize_points)
                        y_points.extend([y_points[-1]] * gas_stabilize_points)
                        z_points.extend([z_points[-1]] * gas_stabilize_points)

                        laser_markers.append((len(x_points), 'laser_on'))

                    # Cut line segment
                    x_line = np.linspace(start[0], end[0], num_points) * scale + offset_x
                    y_line = np.linspace(start[1], end[1], num_points) * scale + offset_y
                    z_line = np.full(num_points, z_height)

                    if i == 0 and len(x_points) > 0:
                        x_points.extend(x_line[1:])
                        y_points.extend(y_line[1:])
                        z_points.extend(z_line[1:])
                    else:
                        x_points.extend(x_line)
                        y_points.extend(y_line)
                        z_points.extend(z_line)

        # Final shutdown sequence: LASER OFF -> wait -> GAS OFF -> lift
        if len(x_points) > 0:
            laser_markers.append((len(x_points), 'laser_off'))

            dwell_points = 25
            x_points.extend([x_points[-1]] * dwell_points)
            y_points.extend([y_points[-1]] * dwell_points)
            z_points.extend([z_points[-1]] * dwell_points)

            laser_markers.append((len(x_points), 'gas_off'))

            lift_points = 50
            x_points.extend([x_points[-1]] * lift_points)
            y_points.extend([y_points[-1]] * lift_points)
            z_points.extend(np.linspace(z_height, z_travel, lift_points))

        if len(x_points) == 0:
            raise ValueError(f"No valid entities found in DXF. Found: {entity_types}")

        print(f"[LASER-DXF] Parsed {len(x_points)} points with {len(laser_markers)} laser control markers")
        return np.array(x_points), np.array(y_points), np.array(z_points), laser_markers

    except Exception as e:
        print(f"[LASER-DXF] Error parsing file: {e}")
        raise ValueError(f"Failed to parse DXF file for laser cutting: {e}")

def generate_grid_pattern_path(grid_rows: int, grid_cols: int, spacing: float,
                                origin_x: float, origin_y: float,
                                z_travel: float, z_pick: float,
                                num_points_per_move: int):
    """
    Generate a smooth grid pattern path for pick and place operations.
    Uses serpentine (snake) pattern to minimize travel distance.

    Returns:
        x_path, y_path, z_path: Arrays of coordinates including vertical movements
    """
    x_points = []
    y_points = []
    z_points = []

    # Generate serpentine pattern through grid
    for row in range(grid_rows):
        # Alternate direction each row (serpentine pattern)
        if row % 2 == 0:
            cols = range(grid_cols)
        else:
            cols = range(grid_cols - 1, -1, -1)

        for col in cols:
            # Calculate grid position
            grid_x = origin_x + (col * spacing)
            grid_y = origin_y + (row * spacing)

            # Add points for: travel -> descend -> pick/place -> ascend

            # 1. Travel to position at safe height (if not first point)
            if len(x_points) > 0:
                # Interpolate from current position to new position at travel height
                x_interp = np.linspace(x_points[-1], grid_x, num_points_per_move)
                y_interp = np.linspace(y_points[-1], grid_y, num_points_per_move)
                z_interp = np.full(num_points_per_move, z_travel)

                x_points.extend(x_interp[1:])  # Skip first point (duplicate)
                y_points.extend(y_interp[1:])
                z_points.extend(z_interp[1:])
            else:
                # First point - start at travel height
                x_points.append(grid_x)
                y_points.append(grid_y)
                z_points.append(z_travel)

            # 2. Descend to pick/place height (SLOW - 200 points for smooth motion)
            descend_points = 200  # Smooth descent - 2 seconds @ 100Hz
            x_descend = np.full(descend_points, grid_x)
            y_descend = np.full(descend_points, grid_y)
            z_descend = np.linspace(z_travel, z_pick, descend_points)

            x_points.extend(x_descend[1:])
            y_points.extend(y_descend[1:])
            z_points.extend(z_descend[1:])

            # 3. Pause at pick/place height (brief pause for stability)
            pause_points = 50  # 0.5 second pause
            x_points.extend([grid_x] * pause_points)
            y_points.extend([grid_y] * pause_points)
            z_points.extend([z_pick] * pause_points)

            # 4. Ascend back to travel height (SLOW - 200 points for smooth motion)
            ascend_points = 200  # Smooth ascent - 2 seconds @ 100Hz
            x_ascend = np.full(ascend_points, grid_x)
            y_ascend = np.full(ascend_points, grid_y)
            z_ascend = np.linspace(z_pick, z_travel, ascend_points)

            x_points.extend(x_ascend[1:])
            y_points.extend(y_ascend[1:])
            z_points.extend(z_ascend[1:])

    return np.array(x_points), np.array(y_points), np.array(z_points)

def validate_workspace(x_path, y_path, z_height):
    """
    Validate that all points are within safe workspace boundaries.
    Returns (is_valid, error_message)
    """
    # Conservative workspace limits for xArm6 (adjust based on your model)
    X_MIN, X_MAX = 50, 700  # mm
    Y_MIN, Y_MAX = -500, 500  # mm
    Z_MIN, Z_MAX = 0, 700   # mm (widened to allow lower Z for drawing)

    for i, (x, y) in enumerate(zip(x_path, y_path)):
        if not (X_MIN <= x <= X_MAX):
            return False, f"Point {i}: X={x:.1f} out of range [{X_MIN}, {X_MAX}]"
        if not (Y_MIN <= y <= Y_MAX):
            return False, f"Point {i}: Y={y:.1f} out of range [{Y_MIN}, {Y_MAX}]"
        if not (Z_MIN <= z_height <= Z_MAX):
            return False, f"Z={z_height:.1f} out of range [{Z_MIN}, {Z_MAX}]"

    return True, "OK"

def follow_path_servo_mode(robot: XArmAPI, x_path, y_path, z_height, roll, pitch, yaw, frequency_hz=100, num_cycles=1, z_path=None):
    """
    Follow a 2D or 3D path using Mode 1 (Servoj mode) with servo_cartesian_aa().
    Streams points at the specified frequency (recommended 100-200 Hz per manual).

    Based on xArm manual section 4.1 - Servo mode recommendations:
    - Frequency: 100-200 Hz recommended (30-250 Hz supported)
    - Points should be close together (smooth interpolation)
    - Speed/acceleration parameters ignored in Mode 1 (uses max speed)

    Args:
        num_cycles: Number of times to repeat the path (for continuous circles)
        z_path: Optional array of Z coordinates (for 3D paths like grid patterns)
                If None, uses constant z_height
    """
    global path_execution_active

    # Use variable Z if provided, otherwise constant
    use_variable_z = z_path is not None

    print(f"\n[PATH] Starting path execution")
    print(f"[PATH] Points per cycle: {len(x_path)}")
    print(f"[PATH] Number of cycles: {num_cycles}")
    print(f"[PATH] Total points: {len(x_path) * num_cycles}")
    print(f"[PATH] Frequency: {frequency_hz} Hz")
    print(f"[PATH] Time step: {1000.0/frequency_hz:.2f} ms")
    print(f"[PATH] Estimated duration: {(len(x_path) * num_cycles) / frequency_hz:.1f}s")
    if use_variable_z:
        print(f"[PATH] Using variable Z (3D path)")
        print(f"[PATH] Z range: [{z_path.min():.1f}, {z_path.max():.1f}]")

    # Validate workspace before starting
    if use_variable_z:
        # For 3D paths, check each Z value
        for i, z in enumerate(z_path):
            is_valid, msg = validate_workspace(x_path[i:i+1], y_path[i:i+1], z)
            if not is_valid:
                print(f"[ERROR] Workspace validation failed at point {i}: {msg}")
                raise ValueError(f"Path outside safe workspace: {msg}")
    else:
        is_valid, msg = validate_workspace(x_path, y_path, z_height)
        if not is_valid:
            print(f"[ERROR] Workspace validation failed: {msg}")
            raise ValueError(f"Path outside safe workspace: {msg}")

    try:
        # First, ensure we're in Mode 0 for the approach move
        print(f"[APPROACH] Moving to start position slowly...")
        robot.set_mode(0)
        robot.set_state(0)
        time.sleep(0.1)

        # Get current position to check distance
        start_z = z_path[0] if use_variable_z else z_height
        code, current_pos = robot.get_position()
        if code == 0 and current_pos:
            distance = ((current_pos[0] - x_path[0])**2 +
                       (current_pos[1] - y_path[0])**2 +
                       (current_pos[2] - start_z)**2) ** 0.5
            print(f"[APPROACH] Current: [{current_pos[0]:.1f}, {current_pos[1]:.1f}, {current_pos[2]:.1f}]")
            print(f"[APPROACH] Target: [{x_path[0]:.1f}, {y_path[0]:.1f}, {start_z:.1f}]")
            print(f"[APPROACH] Distance: {distance:.1f}mm")

        # Move to start position using Mode 0 with SLOW speed
        # Use conservative speed based on distance
        approach_speed = 50 if distance > 100 else 30  # Very slow speeds (mm/s)
        approach_acc = 200  # Low acceleration (mm/s²)

        print(f"[APPROACH] Using speed={approach_speed}mm/s, acc={approach_acc}mm/s²")

        code = robot.set_position_aa(
            axis_angle_pose=[x_path[0], y_path[0], start_z, roll, pitch, yaw],
            speed=approach_speed,
            mvacc=approach_acc,
            wait=True,  # Wait for completion
            is_radian=False
        )

        if code != 0:
            print(f"[ERROR] Failed to move to start position, code: {code}")
            raise Exception(f"Approach move failed with code {code}")

        print(f"[APPROACH] Reached start position")
        time.sleep(0.5)  # Brief pause at start position

    except Exception as e:
        print(f"[EXCEPTION] Error during approach: {e}")
        raise e

    try:
        # Enter servo mode (Mode 1) - per manual section 4.1
        print(f"[MODE] Switching to Mode 1 (Servoj)")
        code = robot.set_mode(1)
        print(f"[MODE] set_mode(1) returned: {code}")

        if code != 0:
            print(f"[ERROR] Failed to enter Mode 1, code: {code}")
            return False

        code = robot.set_state(0)
        print(f"[STATE] set_state(0) returned: {code}")

        time.sleep(0.1)

        # Check current mode
        print(f"[STATUS] Current mode: {robot.mode}, state: {robot.state}")

        if robot.mode != 1:
            print(f"[WARNING] Robot not in Mode 1! Current mode: {robot.mode}")

        # Calculate time between points
        dt = 1.0 / frequency_hz

        path_execution_active = True

        # Stream trajectory points
        print(f"[PATH] Starting point streaming...")
        start_time = time.time()
        error_count = 0
        last_error_code = None
        total_points_executed = 0

        # Execute multiple cycles
        for cycle in range(num_cycles):
            if not path_execution_active:
                print(f"[PATH] Execution stopped by user at cycle {cycle+1}/{num_cycles}")
                break

            if num_cycles > 1:
                print(f"\n[CYCLE] Starting cycle {cycle+1}/{num_cycles}")

            for i in range(len(x_path)):
                if not path_execution_active:
                    print(f"[PATH] Execution stopped by user at point {i} of cycle {cycle+1}")
                    break

                # Send point using set_servo_cartesian_aa (Mode 1)
                # Per manual: send smoothed track points at 100-200 Hz
                # NOTE: speed/mvacc parameters are IGNORED in Mode 1 per manual
                point_start = time.time()

                # Use variable Z if provided
                current_z = z_path[i] if use_variable_z else z_height

                code = robot.set_servo_cartesian_aa(
                    axis_angle_pose=[x_path[i], y_path[i], current_z, roll, pitch, yaw],
                    speed=100,  # Ignored in Mode 1 - included for API compatibility
                    mvacc=200,  # Ignored in Mode 1 - included for API compatibility
                    is_radian=False,
                    is_tool_coord=False
                )

                if code != 0:
                    error_count += 1
                    # Only print if error code changed or every 10th error
                    if code != last_error_code or error_count % 10 == 1:
                        print(f"[ERROR] Point {i}/{len(x_path)}: code={code}, pos=[{x_path[i]:.1f}, {y_path[i]:.1f}, {current_z:.1f}]")
                        last_error_code = code

                # Log every 20th point for progress tracking (or on cycle boundaries)
                if i % 20 == 0 or i == len(x_path) - 1:
                    elapsed = time.time() - start_time
                    cycle_progress = (cycle * len(x_path) + i) / (num_cycles * len(x_path)) * 100
                    print(f"[PROGRESS] Cycle {cycle+1}/{num_cycles}, Point {i}/{len(x_path)} "
                          f"({cycle_progress:.0f}% total) - elapsed: {elapsed:.2f}s, "
                          f"pos: [{x_path[i]:.1f}, {y_path[i]:.1f}, {current_z:.1f}]")

                # Maintain consistent frequency
                point_duration = time.time() - point_start
                sleep_time = max(0, dt - point_duration)

                if sleep_time == 0 and i % 50 == 0:
                    print(f"[TIMING] Warning: Point {i} took {point_duration*1000:.1f}ms (target: {dt*1000:.1f}ms)")

                time.sleep(sleep_time)
                total_points_executed += 1

            if num_cycles > 1 and cycle < num_cycles - 1:
                print(f"[CYCLE] Completed cycle {cycle+1}/{num_cycles}")

        total_time = time.time() - start_time
        print(f"\n[COMPLETE] Path execution finished")
        print(f"[STATS] Total time: {total_time:.2f}s")
        print(f"[STATS] Cycles completed: {min(cycle+1, num_cycles)}/{num_cycles}")
        print(f"[STATS] Points executed: {total_points_executed} (target: {len(x_path) * num_cycles})")
        print(f"[STATS] Average rate: {total_points_executed/total_time:.1f} Hz")
        print(f"[STATS] Errors: {error_count}")

        if error_count > 0:
            print(f"[WARNING] Path had {error_count} errors - consider reducing radius or adjusting center position")

        # Return to position control mode (Mode 0)
        print(f"[MODE] Returning to Mode 0")
        robot.set_mode(0)
        robot.set_state(0)
        print(f"[set_state], xArm is ready to move")

        return True

    except Exception as e:
        print(f"[EXCEPTION] Error during path execution: {e}")
        # Always return to Mode 0 on error
        robot.set_mode(0)
        robot.set_state(0)
        raise e
    finally:
        path_execution_active = False
        print(f"[PATH] Execution flag cleared")
        # Clean any controller errors
        if robot.error_code != 0:
            robot.clean_error()
            print(f"ControllerError had clean")

# API Endpoints

@app.get("/")
async def read_root():
    return FileResponse("static/index.html")

@app.post("/api/connect")
async def connect():
    """Connect to the xArm"""
    try:
        get_arm()
        return {"status": "connected", "message": "Successfully connected to xArm"}
    except Exception as e:
        raise HTTPException(status_code=503, detail=str(e))

@app.post("/api/disconnect")
async def disconnect():
    """Disconnect from the xArm"""
    global arm
    if arm:
        arm.disconnect()
        arm = None
    return {"status": "disconnected"}

@app.get("/api/status", response_model=StatusResponse)
async def get_status():
    """Get current status of the xArm"""
    try:
        robot = get_arm()

        code, state = robot.get_state()
        state_names = {0: "Ready", 1: "Running", 2: "Paused", 3: "Stopped", 4: "Error", 5: "Warning"}

        code, position = robot.get_position()
        code, angles = robot.get_servo_angle()
        is_moving = robot.get_is_moving()

        return StatusResponse(
            connected=True,
            state=robot.state,
            state_name=state_names.get(robot.state, "Unknown"),
            mode=robot.mode,
            error_code=robot.error_code,
            warn_code=robot.warn_code,
            position=position if position else None,
            joint_angles=angles if angles else None,
            is_moving=is_moving
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/motion/enable")
async def enable_motion():
    """Enable robot motion"""
    try:
        robot = get_arm()
        robot.motion_enable(enable=True)
        robot.set_mode(0)
        robot.set_state(state=0)
        time.sleep(0.5)
        return {"status": "enabled"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/motion/disable")
async def disable_motion():
    """Disable robot motion"""
    try:
        robot = get_arm()
        robot.motion_enable(enable=False)
        return {"status": "disabled"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/move/position")
async def move_to_position(pos: Position):
    """Move to a Cartesian position"""
    try:
        robot = get_arm()
        code = robot.set_position(
            x=pos.x, y=pos.y, z=pos.z,
            roll=pos.roll, pitch=pos.pitch, yaw=pos.yaw,
            speed=pos.speed,
            wait=pos.wait
        )
        if code != 0:
            raise HTTPException(status_code=400, detail=f"Move failed with code: {code}")
        return {"status": "moving", "code": code}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/move/joints")
async def move_joints(joint_data: JointAngles):
    """Move to joint angles"""
    try:
        robot = get_arm()
        code = robot.set_servo_angle(
            angle=joint_data.angles,
            speed=joint_data.speed,
            wait=joint_data.wait
        )
        if code != 0:
            raise HTTPException(status_code=400, detail=f"Move failed with code: {code}")
        return {"status": "moving", "code": code}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/move/home")
async def move_home():
    """Move to home position"""
    try:
        robot = get_arm()
        code = robot.move_gohome(wait=True)
        if code != 0:
            raise HTTPException(status_code=400, detail=f"Home failed with code: {code}")
        return {"status": "home", "code": code}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/stop")
async def emergency_stop():
    """Emergency stop"""
    try:
        robot = get_arm()
        robot.emergency_stop()
        return {"status": "stopped"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/clear/errors")
async def clear_errors():
    """Clear errors and warnings"""
    try:
        robot = get_arm()
        robot.clean_error()
        robot.clean_warn()
        return {"status": "cleared"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/gripper/open")
async def gripper_open():
    """Open gripper"""
    try:
        robot = get_arm()
        code = robot.set_gripper_position(850, wait=True)  # 850 is fully open
        return {"status": "opened", "code": code}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/gripper/close")
async def gripper_close():
    """Close gripper"""
    try:
        robot = get_arm()
        code = robot.set_gripper_position(0, wait=True)  # 0 is fully closed
        return {"status": "closed", "code": code}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/pickplace/grid")
async def move_to_grid_position(grid_pos: GridPosition):
    """Move to a position on the cartesian grid with end effector pointing down"""
    try:
        robot = get_arm()

        # Calculate actual position from grid coordinates
        x = grid_pos.origin_x + (grid_pos.col * grid_pos.spacing_x)
        y = grid_pos.origin_y + (grid_pos.row * grid_pos.spacing_y)
        z = grid_pos.origin_z + grid_pos.height

        # End effector pointing straight down: roll=180, pitch=0, yaw=0
        code = robot.set_position(
            x=x, y=y, z=z,
            roll=180, pitch=0, yaw=0,
            speed=grid_pos.speed,
            wait=True
        )

        if code != 0:
            raise HTTPException(status_code=400, detail=f"Move to grid position failed with code: {code}")

        return {
            "status": "moved",
            "code": code,
            "position": {"row": grid_pos.row, "col": grid_pos.col, "x": x, "y": y, "z": z}
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/pickplace/pick")
async def pick_at_grid_position(grid_pos: GridPosition):
    """Pick operation: move down, close gripper, move up"""
    try:
        robot = get_arm()

        # Calculate positions
        x = grid_pos.origin_x + (grid_pos.col * grid_pos.spacing_x)
        y = grid_pos.origin_y + (grid_pos.row * grid_pos.spacing_y)
        z_safe = grid_pos.origin_z + grid_pos.height
        z_pick = grid_pos.origin_z

        # Move to safe height above position
        code = robot.set_position(x=x, y=y, z=z_safe, roll=180, pitch=0, yaw=0, speed=grid_pos.speed, wait=True)
        if code != 0:
            raise HTTPException(status_code=400, detail=f"Move to safe height failed with code: {code}")

        # Open gripper
        robot.set_gripper_position(850, wait=True)

        # Move down to pick position
        code = robot.set_position(x=x, y=y, z=z_pick, roll=180, pitch=0, yaw=0, speed=grid_pos.speed // 2, wait=True)
        if code != 0:
            raise HTTPException(status_code=400, detail=f"Move down failed with code: {code}")

        # Close gripper
        robot.set_gripper_position(0, wait=True)
        time.sleep(0.5)

        # Move back up to safe height
        code = robot.set_position(x=x, y=y, z=z_safe, roll=180, pitch=0, yaw=0, speed=grid_pos.speed // 2, wait=True)
        if code != 0:
            raise HTTPException(status_code=400, detail=f"Move up failed with code: {code}")

        return {"status": "picked", "position": {"row": grid_pos.row, "col": grid_pos.col}}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/pickplace/place")
async def place_at_grid_position(grid_pos: GridPosition):
    """Place operation: move down, open gripper, move up"""
    try:
        robot = get_arm()

        # Calculate positions
        x = grid_pos.origin_x + (grid_pos.col * grid_pos.spacing_x)
        y = grid_pos.origin_y + (grid_pos.row * grid_pos.spacing_y)
        z_safe = grid_pos.origin_z + grid_pos.height
        z_place = grid_pos.origin_z

        # Move to safe height above position
        code = robot.set_position(x=x, y=y, z=z_safe, roll=180, pitch=0, yaw=0, speed=grid_pos.speed, wait=True)
        if code != 0:
            raise HTTPException(status_code=400, detail=f"Move to safe height failed with code: {code}")

        # Move down to place position
        code = robot.set_position(x=x, y=y, z=z_place, roll=180, pitch=0, yaw=0, speed=grid_pos.speed // 2, wait=True)
        if code != 0:
            raise HTTPException(status_code=400, detail=f"Move down failed with code: {code}")

        # Open gripper
        robot.set_gripper_position(850, wait=True)
        time.sleep(0.5)

        # Move back up to safe height
        code = robot.set_position(x=x, y=y, z=z_safe, roll=180, pitch=0, yaw=0, speed=grid_pos.speed // 2, wait=True)
        if code != 0:
            raise HTTPException(status_code=400, detail=f"Move up failed with code: {code}")

        return {"status": "placed", "position": {"row": grid_pos.row, "col": grid_pos.col}}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# Spline path following endpoints
@app.post("/api/path/spline/preview")
async def preview_spline_path(spline_req: SplinePathRequest):
    """
    Generate and return spline path points for preview (doesn't move robot).
    """
    try:
        if len(spline_req.control_points) < 2:
            raise HTTPException(status_code=400, detail="Need at least 2 control points")

        # Generate spline path
        x_path, y_path = generate_spline_path(
            spline_req.control_points,
            spline_req.num_samples,
            spline_req.spline_type
        )

        # Return path points for visualization
        path_points = [
            {"x": float(x), "y": float(y), "z": spline_req.z_height}
            for x, y in zip(x_path, y_path)
        ]

        return {
            "status": "preview_generated",
            "num_points": len(path_points),
            "path_points": path_points[:100],  # Return first 100 for preview
            "control_points": [{"x": p.x, "y": p.y} for p in spline_req.control_points]
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/path/spline/execute")
async def execute_spline_path(spline_req: SplinePathRequest):
    """
    Execute spline path following using Mode 1 (Servoj mode).
    Runs in background thread to avoid blocking.
    """
    global path_execution_active

    try:
        print(f"\n[API] Received spline execution request")
        print(f"[API] Control points: {len(spline_req.control_points)}")
        print(f"[API] Spline type: {spline_req.spline_type}")
        print(f"[API] Samples: {spline_req.num_samples}")
        print(f"[API] Z height: {spline_req.z_height}")

        if path_execution_active:
            print(f"[API] ERROR: Path already executing")
            raise HTTPException(status_code=409, detail="Path execution already in progress")

        robot = get_arm()

        # Generate spline path
        print(f"[API] Generating spline path...")
        x_path, y_path = generate_spline_path(
            spline_req.control_points,
            spline_req.num_samples,
            spline_req.spline_type
        )
        print(f"[API] Generated {len(x_path)} points")
        print(f"[API] X range: [{x_path.min():.1f}, {x_path.max():.1f}]")
        print(f"[API] Y range: [{y_path.min():.1f}, {y_path.max():.1f}]")

        # Calculate execution frequency (100-200 Hz per xArm manual)
        # Match frequency to point density for smooth motion
        frequency = min(200, max(100, spline_req.num_samples // 2))
        print(f"[API] Execution frequency: {frequency} Hz")

        # Execute path in background thread
        def run_path():
            try:
                print(f"[THREAD] Background thread started")
                follow_path_servo_mode(
                    robot, x_path, y_path,
                    spline_req.z_height,
                    spline_req.roll, spline_req.pitch, spline_req.yaw,
                    frequency_hz=frequency
                )
                print(f"[THREAD] Background thread completed successfully")
            except Exception as e:
                print(f"[THREAD] Path execution error: {e}")
                import traceback
                traceback.print_exc()

        thread = threading.Thread(target=run_path, daemon=True)
        thread.start()
        print(f"[API] Background thread launched")

        return {
            "status": "executing",
            "num_points": len(x_path),
            "frequency_hz": frequency,
            "estimated_duration_sec": len(x_path) / frequency
        }
    except HTTPException:
        raise
    except Exception as e:
        print(f"[API] Exception: {e}")
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/path/circle/execute")
async def execute_circle_path(circle_req: CirclePath):
    """
    Execute circular path following using Mode 1 (Servoj mode).
    """
    global path_execution_active

    try:
        if path_execution_active:
            raise HTTPException(status_code=409, detail="Path execution already in progress")

        robot = get_arm()

        # Generate circle path
        x_path, y_path = generate_circle_path(
            circle_req.center_x,
            circle_req.center_y,
            circle_req.radius,
            circle_req.num_points
        )

        # Calculate appropriate frequency based on path density
        # For 100 points: 100 Hz = 1 second duration
        # For 200 points: 100 Hz = 2 second duration
        # Keep frequency in 100-200 Hz range per xArm manual
        circumference = 2 * 3.14159 * circle_req.radius
        point_spacing = circumference / circle_req.num_points

        # Adjust frequency based on point density
        # Closer points (< 2mm) -> higher frequency for smooth motion
        # Farther points (> 5mm) -> lower frequency to avoid aggressive motion
        if point_spacing < 2.0:
            frequency = 150  # High frequency for dense points
        elif point_spacing < 5.0:
            frequency = 100  # Standard frequency
        else:
            frequency = 80   # Lower frequency for sparse points (still > 30 Hz min)

        print(f"[CIRCLE] Circumference: {circumference:.1f}mm")
        print(f"[CIRCLE] Point spacing: {point_spacing:.1f}mm")
        print(f"[CIRCLE] Number of cycles: {circle_req.num_cycles}")
        print(f"[CIRCLE] Selected frequency: {frequency} Hz")

        # Execute path in background thread
        def run_path():
            try:
                follow_path_servo_mode(
                    robot, x_path, y_path,
                    circle_req.z_height,
                    circle_req.roll, circle_req.pitch, circle_req.yaw,
                    frequency_hz=frequency,
                    num_cycles=circle_req.num_cycles
                )
            except Exception as e:
                print(f"Path execution error: {e}")
                import traceback
                traceback.print_exc()

        thread = threading.Thread(target=run_path, daemon=True)
        thread.start()

        total_points = len(x_path) * circle_req.num_cycles

        return {
            "status": "executing",
            "num_points": len(x_path),
            "num_cycles": circle_req.num_cycles,
            "total_points": total_points,
            "frequency_hz": frequency,
            "estimated_duration_sec": total_points / frequency,
            "point_spacing_mm": round(point_spacing, 2)
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/path/grid/execute")
async def execute_grid_pattern_path(grid_req: GridPatternPath):
    """
    Execute a 5x5 grid pick-and-place pattern using Mode 1 (Servoj mode).
    Creates a smooth serpentine path through the grid with vertical movements.
    """
    global path_execution_active

    try:
        if path_execution_active:
            raise HTTPException(status_code=409, detail="Path execution already in progress")

        robot = get_arm()

        # Generate grid pattern path
        print(f"\n[GRID] Generating {grid_req.grid_rows}x{grid_req.grid_cols} grid pattern")
        print(f"[GRID] Spacing: {grid_req.spacing}mm")
        print(f"[GRID] Origin: ({grid_req.origin_x}, {grid_req.origin_y})")
        print(f"[GRID] Z travel: {grid_req.z_travel}mm, Z pick: {grid_req.z_pick}mm")

        x_path, y_path, z_path = generate_grid_pattern_path(
            grid_req.grid_rows,
            grid_req.grid_cols,
            grid_req.spacing,
            grid_req.origin_x,
            grid_req.origin_y,
            grid_req.z_travel,
            grid_req.z_pick,
            grid_req.num_points_per_move
        )

        print(f"[GRID] Generated {len(x_path)} points")
        print(f"[GRID] X range: [{x_path.min():.1f}, {x_path.max():.1f}]")
        print(f"[GRID] Y range: [{y_path.min():.1f}, {y_path.max():.1f}]")
        print(f"[GRID] Z range: [{z_path.min():.1f}, {z_path.max():.1f}]")

        # Use fixed frequency for grid pattern (slower for precision)
        frequency = 100  # 100 Hz for controlled movements

        # Execute path in background thread
        def run_path():
            try:
                follow_path_servo_mode(
                    robot, x_path, y_path,
                    z_height=grid_req.z_travel,  # Not used due to z_path
                    roll=grid_req.roll,
                    pitch=grid_req.pitch,
                    yaw=grid_req.yaw,
                    frequency_hz=frequency,
                    num_cycles=1,
                    z_path=z_path  # Variable Z for 3D path
                )
            except Exception as e:
                print(f"Grid path execution error: {e}")
                import traceback
                traceback.print_exc()

        thread = threading.Thread(target=run_path, daemon=True)
        thread.start()

        total_grid_positions = grid_req.grid_rows * grid_req.grid_cols

        return {
            "status": "executing",
            "grid_size": f"{grid_req.grid_rows}x{grid_req.grid_cols}",
            "total_grid_positions": total_grid_positions,
            "total_points": len(x_path),
            "frequency_hz": frequency,
            "estimated_duration_sec": len(x_path) / frequency
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/path/dxf/execute")
async def execute_dxf_path(dxf_req: DXFPathRequest):
    """
    Execute a path from a DXF file using Mode 1 (Servoj mode).
    Traces lines and polylines from the DXF with pen up/down movements.
    """
    global path_execution_active

    try:
        if path_execution_active:
            raise HTTPException(status_code=409, detail="Path execution already in progress")

        robot = get_arm()

        # Parse DXF file
        print(f"\n[DXF] Loading file: {dxf_req.dxf_file_path}")
        print(f"[DXF] Scale: {dxf_req.scale}, Offset: ({dxf_req.offset_x}, {dxf_req.offset_y})")

        x_path, y_path, z_path = parse_dxf_to_path(
            dxf_req.dxf_file_path,
            dxf_req.scale,
            dxf_req.offset_x,
            dxf_req.offset_y,
            dxf_req.z_height,
            dxf_req.z_travel,
            dxf_req.points_per_unit
        )

        print(f"[DXF] Generated {len(x_path)} points")
        print(f"[DXF] X range: [{x_path.min():.1f}, {x_path.max():.1f}]")
        print(f"[DXF] Y range: [{y_path.min():.1f}, {y_path.max():.1f}]")
        print(f"[DXF] Z range: [{z_path.min():.1f}, {z_path.max():.1f}]")

        # Calculate point density to choose appropriate frequency
        total_distance = 0
        for i in range(1, len(x_path)):
            dx = x_path[i] - x_path[i-1]
            dy = y_path[i] - y_path[i-1]
            total_distance += np.sqrt(dx*dx + dy*dy)

        avg_point_spacing = total_distance / len(x_path) if len(x_path) > 1 else 1.0
        print(f"[DXF] Average point spacing: {avg_point_spacing:.2f}mm")

        # Adjust frequency based on point density
        # Dense points (< 1mm) = use lower frequency for smoother motion
        # Sparse points (> 2mm) = can use higher frequency
        if avg_point_spacing < 1.0:
            frequency = 50  # Very dense points - slow down
        elif avg_point_spacing < 2.0:
            frequency = 80  # Dense points - medium speed
        else:
            frequency = 100  # Normal spacing

        print(f"[DXF] Selected frequency: {frequency} Hz (slower = smoother for dense paths)")

        # Execute path in background thread
        def run_path():
            try:
                follow_path_servo_mode(
                    robot, x_path, y_path,
                    z_height=dxf_req.z_height,  # Not used due to z_path
                    roll=dxf_req.roll,
                    pitch=dxf_req.pitch,
                    yaw=dxf_req.yaw,
                    frequency_hz=frequency,
                    num_cycles=1,
                    z_path=z_path  # Variable Z for pen up/down
                )
            except Exception as e:
                print(f"DXF path execution error: {e}")
                import traceback
                traceback.print_exc()

        thread = threading.Thread(target=run_path, daemon=True)
        thread.start()

        return {
            "status": "executing",
            "dxf_file": dxf_req.dxf_file_path,
            "total_points": len(x_path),
            "frequency_hz": frequency,
            "estimated_duration_sec": len(x_path) / frequency
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/path/laser-dxf/execute")
async def execute_laser_dxf_path(laser_req: LaserDXFPathRequest):
    """
    Execute a laser cutting path from a DXF file using Mode 1 (Servoj mode).
    Controls gas and laser in proper sequence: gas on -> laser on -> cut -> laser off -> gas off.
    """
    global path_execution_active

    try:
        if path_execution_active:
            raise HTTPException(status_code=409, detail="Path execution already in progress")

        robot = get_arm()

        # Parse DXF file for laser cutting
        print(f"\n[LASER-DXF] Loading file: {laser_req.dxf_file_path}")
        print(f"[LASER-DXF] Scale: {laser_req.scale}, Offset: ({laser_req.offset_x}, {laser_req.offset_y})")

        x_path, y_path, z_path, laser_markers = parse_dxf_to_laser_path(
            laser_req.dxf_file_path,
            laser_req.scale,
            laser_req.offset_x,
            laser_req.offset_y,
            laser_req.z_height,
            laser_req.z_travel,
            laser_req.points_per_unit
        )

        print(f"[LASER-DXF] Generated {len(x_path)} points with {len(laser_markers)} laser control markers")
        print(f"[LASER-DXF] X range: [{x_path.min():.1f}, {x_path.max():.1f}]")
        print(f"[LASER-DXF] Y range: [{y_path.min():.1f}, {y_path.max():.1f}]")
        print(f"[LASER-DXF] Z range: [{z_path.min():.1f}, {z_path.max():.1f}]")

        # Calculate frequency
        total_distance = 0
        for i in range(1, len(x_path)):
            dx = x_path[i] - x_path[i-1]
            dy = y_path[i] - y_path[i-1]
            total_distance += np.sqrt(dx*dx + dy*dy)

        avg_point_spacing = total_distance / len(x_path) if len(x_path) > 1 else 1.0
        print(f"[LASER-DXF] Average point spacing: {avg_point_spacing:.2f}mm")

        if avg_point_spacing < 1.0:
            frequency = 50
        elif avg_point_spacing < 2.0:
            frequency = 80
        else:
            frequency = 100

        print(f"[LASER-DXF] Selected frequency: {frequency} Hz")

        # Execute path with laser control in background thread
        def run_laser_path():
            global path_execution_active
            try:
                path_execution_active = True

                # Modified servo mode with laser control
                robot.set_mode(1)
                robot.set_state(0)
                time.sleep(0.1)

                print(f"[LASER-PATH] Starting execution with {len(laser_markers)} control points")

                # Track next laser marker
                marker_idx = 0

                for i in range(len(x_path)):
                    if not path_execution_active:
                        print("[LASER-PATH] Execution stopped by user")
                        break

                    # Check if we need to execute laser control at this point
                    while marker_idx < len(laser_markers) and laser_markers[marker_idx][0] == i:
                        action = laser_markers[marker_idx][1]
                        print(f"[LASER-CONTROL] Point {i}: {action}")

                        if action == 'gas_on':
                            gas_on()
                        elif action == 'gas_off':
                            gas_off()
                        elif action == 'laser_on':
                            laser_on()
                        elif action == 'laser_off':
                            laser_off()

                        marker_idx += 1

                    # Move to next point
                    code = robot.set_servo_cartesian_aa(
                        [x_path[i], y_path[i], z_path[i], laser_req.roll, laser_req.pitch, laser_req.yaw],
                        speed=100.0,
                        acceleration=200.0,
                        is_radian=False,
                        is_tool_coord=False
                    )

                    if code != 0:
                        print(f"[ERROR] Point {i}/{len(x_path)}: code={code}, pos=[{x_path[i]:.1f}, {y_path[i]:.1f}, {z_path[i]:.1f}]")

                    if i % 500 == 0:
                        print(f"[PROGRESS] Point {i}/{len(x_path)} ({100*i//len(x_path)}%)")

                    time.sleep(1.0 / frequency)

                # Emergency shutdown: ensure laser and gas are off
                print("\n[LASER-PATH] Completing final shutdown sequence...")
                laser_off()
                time.sleep(0.5)
                gas_off()

                print("[LASER-PATH] Execution complete")
                robot.set_mode(0)
                robot.set_state(0)

            except Exception as e:
                print(f"[ERROR] Laser path execution error: {e}")
                # EMERGENCY SHUTDOWN
                print("[EMERGENCY] Shutting down laser and gas...")
                laser_off()
                time.sleep(0.5)
                gas_off()
                import traceback
                traceback.print_exc()
            finally:
                path_execution_active = False

        thread = threading.Thread(target=run_laser_path, daemon=True)
        thread.start()

        return {
            "status": "executing",
            "dxf_file": laser_req.dxf_file_path,
            "total_points": len(x_path),
            "laser_markers": len(laser_markers),
            "frequency_hz": frequency,
            "estimated_duration_sec": len(x_path) / frequency
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/path/stop")
async def stop_path_execution():
    """
    Stop current path execution.
    """
    global path_execution_active
    path_execution_active = False

    try:
        robot = get_arm()
        # Return to Mode 0
        robot.set_mode(0)
        robot.set_state(0)
        return {"status": "path_stopped"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/path/status")
async def get_path_status():
    """
    Get current path execution status.
    """
    return {
        "is_executing": path_execution_active
    }


# ---------- trajectory recording ----------
#
# Pose/joint timeseries fed by the xArm SDK's report callback (native
# rate, ~100Hz) so trajectory-quality assertions (overshoot, settling,
# max velocity) don't have to poll /api/status from the test harness
# and pay HTTP roundtrip + jitter for every sample. One recording at a
# time; the test starts before a move and pulls /data after stop.

recording_active: bool = False
recording_buffer: List[dict] = []
recording_start_mono: Optional[float] = None
recording_lock = threading.Lock()
recording_callback = None
RECORDING_MAX_SAMPLES = 50_000  # ~8min at 100Hz; guard against leaks

class RecordingSample(BaseModel):
    t: float            # seconds since recording start (monotonic)
    pose: Optional[List[float]] = None
    joints: Optional[List[float]] = None

class RecordingStartResponse(BaseModel):
    status: str
    started_at: float

class RecordingStopResponse(BaseModel):
    status: str
    samples: int
    duration_s: float

class RecordingDataResponse(BaseModel):
    active: bool
    samples: List[RecordingSample]
    count: int
    duration_s: float


def _make_recording_callback():
    """Build the report-callback closure registered with the SDK.

    Captures pose + joint angles + a monotonic timestamp into the
    shared buffer, drops samples once RECORDING_MAX_SAMPLES is hit
    (we don't want a forgotten recording to chew memory)."""
    def _cb(item):
        global recording_active, recording_start_mono
        if not recording_active or recording_start_mono is None:
            return
        with recording_lock:
            if len(recording_buffer) >= RECORDING_MAX_SAMPLES:
                return
            try:
                t = time.monotonic() - recording_start_mono
                pose = item.get("pose") if isinstance(item, dict) else None
                angle = item.get("angle") if isinstance(item, dict) else None
                recording_buffer.append({
                    "t": t,
                    "pose": list(pose) if pose else None,
                    "joints": list(angle) if angle else None,
                })
            except Exception as e:
                print(f"[recording] sample drop: {e}")
    return _cb


@app.post("/api/recording/start", response_model=RecordingStartResponse)
async def recording_start():
    """Start a trajectory recording.

    Clears any prior buffer and registers an SDK report callback so
    samples flow at the arm's native rate. Idempotent: starting while
    already active resets the buffer + start time.
    """
    global recording_active, recording_start_mono, recording_callback
    try:
        robot = get_arm()
        with recording_lock:
            recording_buffer.clear()
            recording_start_mono = time.monotonic()
            if recording_callback is None:
                recording_callback = _make_recording_callback()
                # `register_report_callback` SDK signature: (callback,
                # report_cartesian=True, report_joints=True, ...). We
                # take the defaults — it pushes the full state dict.
                robot.register_report_callback(recording_callback)
            recording_active = True
        return RecordingStartResponse(status="recording", started_at=time.time())
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/recording/stop", response_model=RecordingStopResponse)
async def recording_stop():
    """Stop the active recording. Buffer is preserved for /data."""
    global recording_active, recording_start_mono
    if not recording_active:
        return RecordingStopResponse(status="idle", samples=0, duration_s=0.0)
    with recording_lock:
        recording_active = False
        duration = (time.monotonic() - recording_start_mono) if recording_start_mono else 0.0
        count = len(recording_buffer)
    return RecordingStopResponse(
        status="stopped",
        samples=count,
        duration_s=duration,
    )


@app.get("/api/recording/data", response_model=RecordingDataResponse)
async def recording_data():
    """Return the captured samples (whether the recording is still active or stopped)."""
    with recording_lock:
        samples = list(recording_buffer)
        active = recording_active
        duration = (time.monotonic() - recording_start_mono) if recording_start_mono else 0.0
    return RecordingDataResponse(
        active=active,
        samples=[RecordingSample(**s) for s in samples],
        count=len(samples),
        duration_s=duration,
    )

# Mount static files
app.mount("/static", StaticFiles(directory="static"), name="static")

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
