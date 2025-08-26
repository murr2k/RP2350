#!/usr/bin/env python3
"""
Simple regression test for Kalman filter
Tests basic injection and tracking
"""

import socket
import time
import numpy as np

def test_rotation(axis_name, gyro_rate, duration=2.0):
    """Test rotation on a single axis"""
    print(f"\n=== Testing {axis_name} Axis Rotation ===")
    print(f"Rate: {gyro_rate} deg/s for {duration} seconds")
    
    try:
        # Connect
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0)
        sock.connect(('192.168.64.1', 9999))
        
        # Reset device
        sock.send(b'r\n')
        time.sleep(0.5)
        
        # Clear buffer
        try:
            while True:
                sock.recv(1024)
        except socket.timeout:
            pass
        
        # Set up gyro values based on axis
        if axis_name == "X":
            gx, gy, gz = gyro_rate, 0, 0
            expected_angle_idx = 7  # pitch
            angle_name = "Pitch"
        elif axis_name == "Y":
            gx, gy, gz = 0, gyro_rate, 0
            expected_angle_idx = 8  # roll
            angle_name = "Roll"
        else:  # Z
            gx, gy, gz = 0, 0, gyro_rate
            expected_angle_idx = 9  # yaw
            angle_name = "Yaw"
        
        # Run test
        steps = int(duration * 50)  # 50Hz
        dt = 0.02  # 20ms per step
        angles = []
        
        print(f"Injecting {steps} samples...")
        for i in range(steps):
            # Inject IMU data
            cmd = f"I0.0,0.0,1.0,{gx:.1f},{gy:.1f},{gz:.1f}\n"
            sock.send(cmd.encode())
            
            # Small delay
            time.sleep(dt)
            
            # Try to read response
            sock.settimeout(0.01)
            try:
                response = sock.recv(1024).decode('utf-8', errors='ignore')
                for line in response.split('\n'):
                    if 'CSV' in line:
                        parts = line.split(',')
                        if len(parts) > expected_angle_idx:
                            try:
                                angle = float(parts[expected_angle_idx])
                                angles.append(angle)
                                # Print progress every 10 samples
                                if i % 10 == 0:
                                    expected = gyro_rate * i * dt
                                    print(f"  Step {i}: {angle_name}={angle:.1f}° (expected={expected:.1f}°)")
                            except ValueError:
                                pass
                        break
            except socket.timeout:
                pass
        
        # Analyze results
        if angles:
            final_angle = angles[-1]
            expected_final = gyro_rate * duration
            error = abs(final_angle - expected_final)
            
            print(f"\nResults:")
            print(f"  Expected final angle: {expected_final:.1f}°")
            print(f"  Actual final angle: {final_angle:.1f}°")
            print(f"  Error: {error:.1f}°")
            
            if error < 5.0:
                print(f"  ✓ PASS - {axis_name} axis tracking within tolerance")
                return True
            else:
                print(f"  ✗ FAIL - {axis_name} axis error too large")
                return False
        else:
            print(f"  ✗ FAIL - No data received")
            return False
        
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return False
    finally:
        sock.close()

def main():
    print("=" * 60)
    print("Simple Kalman Filter Regression Test")
    print("=" * 60)
    
    # Check bridge connection
    try:
        test_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        test_sock.settimeout(1)
        test_sock.connect(('192.168.64.1', 9999))
        test_sock.close()
        print("✓ Serial bridge connected\n")
    except:
        print("✗ Cannot connect to bridge at 192.168.64.1:9999")
        print("Make sure serial_bridge.py is running on Windows")
        return
    
    # Test each axis
    results = []
    
    # Test positive rotations
    results.append(("X+", test_rotation("X", 30.0, duration=2.0)))
    time.sleep(1)
    
    results.append(("Y+", test_rotation("Y", 30.0, duration=2.0)))
    time.sleep(1)
    
    results.append(("Z+", test_rotation("Z", 30.0, duration=2.0)))
    time.sleep(1)
    
    # Test negative rotations
    results.append(("X-", test_rotation("X", -30.0, duration=2.0)))
    time.sleep(1)
    
    results.append(("Y-", test_rotation("Y", -30.0, duration=2.0)))
    time.sleep(1)
    
    results.append(("Z-", test_rotation("Z", -30.0, duration=2.0)))
    time.sleep(1)
    
    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    passed = sum(1 for _, p in results if p)
    total = len(results)
    
    for test_name, passed_test in results:
        status = "✓ PASS" if passed_test else "✗ FAIL"
        print(f"{test_name}: {status}")
    
    print(f"\nTotal: {passed}/{total} tests passed")
    
    if passed == total:
        print("✓ All tests passed!")
    else:
        print("✗ Some tests failed - check Kalman filter processing")

if __name__ == "__main__":
    main()