#!/usr/bin/env python3
"""
Test script for xArm spline path following.
Demonstrates how to use the spline API endpoints.
"""

import requests
import json
import time

# Configuration
API_BASE = "http://localhost:8000/api"

def test_spline_preview():
    """Test spline path preview generation."""
    print("\n=== Testing Spline Preview ===")

    # Define control points for a simple curve
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

    response = requests.post(f"{API_BASE}/path/spline/preview", json=data)

    if response.status_code == 200:
        result = response.json()
        print(f"✓ Preview generated successfully")
        print(f"  Number of points: {result['num_points']}")
        print(f"  First few points: {result['path_points'][:3]}")
    else:
        print(f"✗ Preview failed: {response.text}")

def test_circle_path():
    """Test circular path execution."""
    print("\n=== Testing Circle Path ===")

    data = {
        "center_x": 400,
        "center_y": 0,
        "radius": 50,
        "z_height": 300,
        "num_points": 100,
        "roll": 180,
        "pitch": 0,
        "yaw": 0
    }

    response = requests.post(f"{API_BASE}/path/circle/execute", json=data)

    if response.status_code == 200:
        result = response.json()
        print(f"✓ Circle path started")
        print(f"  Number of points: {result['num_points']}")
        print(f"  Frequency: {result['frequency_hz']} Hz")
        print(f"  Estimated duration: {result['estimated_duration_sec']:.2f} seconds")

        # Monitor execution
        time.sleep(1)
        status = requests.get(f"{API_BASE}/path/status").json()
        print(f"  Execution status: {'Running' if status['is_executing'] else 'Completed'}")
    else:
        print(f"✗ Circle path failed: {response.text}")

def test_spline_execution():
    """Test spline path execution."""
    print("\n=== Testing Spline Execution ===")

    # Define a wave pattern
    data = {
        "control_points": [
            {"x": 300, "y": -50},
            {"x": 350, "y": 50},
            {"x": 400, "y": -50},
            {"x": 450, "y": 50},
            {"x": 500, "y": -50}
        ],
        "z_height": 300,
        "num_samples": 200,
        "spline_type": "cubic",
        "roll": 180,
        "pitch": 0,
        "yaw": 0
    }

    response = requests.post(f"{API_BASE}/path/spline/execute", json=data)

    if response.status_code == 200:
        result = response.json()
        print(f"✓ Spline path started")
        print(f"  Number of points: {result['num_points']}")
        print(f"  Frequency: {result['frequency_hz']} Hz")
        print(f"  Estimated duration: {result['estimated_duration_sec']:.2f} seconds")

        # Monitor execution
        for i in range(3):
            time.sleep(1)
            status = requests.get(f"{API_BASE}/path/status").json()
            print(f"  Status after {i+1}s: {'Running' if status['is_executing'] else 'Completed'}")
    else:
        print(f"✗ Spline execution failed: {response.text}")

def test_stop_path():
    """Test stopping path execution."""
    print("\n=== Testing Path Stop ===")

    response = requests.post(f"{API_BASE}/path/stop")

    if response.status_code == 200:
        print(f"✓ Path stopped successfully")
    else:
        print(f"✗ Stop failed: {response.text}")

def check_connection():
    """Check if API is accessible."""
    try:
        response = requests.get(f"{API_BASE}/status", timeout=2)
        if response.status_code == 200:
            print("✓ API is accessible")
            result = response.json()
            print(f"  Robot connected: {result['connected']}")
            print(f"  Mode: {result['mode']}")
            print(f"  State: {result['state_name']}")
            return True
        else:
            print(f"✗ API returned error: {response.status_code}")
            return False
    except requests.exceptions.ConnectionError:
        print("✗ Cannot connect to API. Make sure the server is running.")
        return False

if __name__ == "__main__":
    print("xArm Spline Path Following Test")
    print("=" * 50)

    if not check_connection():
        print("\nPlease start the server first:")
        print("  cd /home/mclovin/xarm")
        print("  ./start.sh")
        exit(1)

    # Run tests
    test_spline_preview()

    # Uncomment to test actual robot movement (requires robot connected and enabled)
    # WARNING: Robot will move! Ensure workspace is clear.

    # input("\nPress Enter to test circle path (robot will move)...")
    # test_circle_path()

    # input("\nPress Enter to test spline execution (robot will move)...")
    # test_spline_execution()

    # input("\nPress Enter to test stop...")
    # test_stop_path()

    print("\n" + "=" * 50)
    print("Tests completed!")
    print("\nTo run movement tests, uncomment the test calls in the script.")
