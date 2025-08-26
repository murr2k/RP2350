#!/usr/bin/env python3
"""
Simple test of IMU data injection system
Tests the basic injection command and response
"""

import socket
import time

def test_injection():
    """Test basic IMU data injection"""
    print("Testing IMU Data Injection System")
    print("=" * 40)
    
    try:
        # Connect to serial bridge
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('192.168.64.1', 9999))
        sock.settimeout(0.5)
        print("✓ Connected to serial bridge")
        
        # Reset device
        print("\n1. Resetting device...")
        sock.send(b'r\n')
        time.sleep(0.5)
        
        # Clear any pending data
        try:
            while True:
                sock.recv(1024)
        except socket.timeout:
            pass
        
        # Test single injection
        print("\n2. Testing single data injection...")
        test_data = "I0.0,0.0,1.0,30.0,0.0,0.0"  # 30 deg/s on X axis
        sock.send((test_data + '\n').encode())
        time.sleep(0.1)
        
        # Read response
        try:
            response = sock.recv(1024).decode('utf-8', errors='ignore')
            if 'CSV' in response:
                print(f"✓ Received response: {response.strip()}")
                # Parse CSV
                parts = response.split(',')
                if len(parts) >= 10:
                    pitch = float(parts[7])
                    roll = float(parts[8])
                    yaw = float(parts[9])
                    print(f"  Orientation: Pitch={pitch:.1f}° Roll={roll:.1f}° Yaw={yaw:.1f}°")
            else:
                print(f"Response: {response.strip()}")
        except socket.timeout:
            print("✗ No response received")
        
        # Test sequence of injections
        print("\n3. Testing injection sequence...")
        for i in range(10):
            # Inject data simulating rotation
            angle = i * 3.0  # degrees
            test_data = f"I0.0,0.0,1.0,30.0,0.0,0.0"
            sock.send((test_data + '\n').encode())
            time.sleep(0.02)  # 50Hz
            
            # Read response
            try:
                response = sock.recv(1024).decode('utf-8', errors='ignore')
                if 'CSV' in response:
                    lines = response.strip().split('\n')
                    for line in lines:
                        if 'CSV' in line:
                            parts = line.split(',')
                            if len(parts) >= 10:
                                pitch = float(parts[7])
                                print(f"  Step {i+1}: Pitch={pitch:.1f}°")
                                break
            except socket.timeout:
                pass
        
        # Test with debug stream
        print("\n4. Testing with debug stream...")
        sock.send(b'd\n')  # Enable debug
        time.sleep(0.1)
        
        # Clear buffer
        try:
            while True:
                sock.recv(1024)
        except socket.timeout:
            pass
        
        # Inject and monitor
        print("Injecting rotation and monitoring...")
        for i in range(5):
            test_data = f"I0.0,0.0,1.0,{i*10.0},0.0,0.0"
            sock.send((test_data + '\n').encode())
            time.sleep(0.1)
            
            # Read multiple responses
            try:
                data = sock.recv(4096).decode('utf-8', errors='ignore')
                lines = data.strip().split('\n')
                csv_count = sum(1 for line in lines if 'CSV' in line)
                print(f"  Received {csv_count} CSV lines")
            except socket.timeout:
                print("  Timeout")
        
        sock.send(b'd\n')  # Disable debug
        
        print("\n✓ All tests completed!")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
    finally:
        if sock:
            sock.close()

def test_quadrant_motion():
    """Test a simple quadrant motion pattern"""
    print("\n" + "=" * 40)
    print("Testing Quadrant Motion Pattern")
    print("=" * 40)
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('192.168.64.1', 9999))
        sock.settimeout(0.1)
        
        # Reset and calibrate
        sock.send(b'r\n')
        time.sleep(0.5)
        sock.send(b'c\n')
        time.sleep(1.0)
        
        print("\nSimulating 90° rotation around X axis...")
        
        # Clear buffer
        try:
            while True:
                sock.recv(1024)
        except socket.timeout:
            pass
        
        # Simulate 90 degree rotation over 3 seconds
        rate = 30.0  # deg/s
        duration = 3.0
        steps = int(duration * 50)  # 50Hz
        dt = 1.0 / 50.0
        
        angles = []
        for i in range(steps):
            # Calculate expected angle
            expected_angle = rate * i * dt
            
            # Inject rotation
            test_data = f"I0.0,0.0,1.0,{rate},0.0,0.0"
            sock.send((test_data + '\n').encode())
            
            # Small delay
            time.sleep(dt)
            
            # Read response
            try:
                response = sock.recv(1024).decode('utf-8', errors='ignore')
                for line in response.split('\n'):
                    if 'CSV' in line:
                        parts = line.split(',')
                        if len(parts) >= 10:
                            pitch = float(parts[7])
                            angles.append(pitch)
                            
                            # Print every 10th step
                            if i % 10 == 0:
                                error = abs(pitch - expected_angle)
                                print(f"  t={i*dt:.1f}s: Expected={expected_angle:.1f}° "
                                     f"Actual={pitch:.1f}° Error={error:.1f}°")
                            break
            except socket.timeout:
                pass
        
        # Final analysis
        if angles:
            final_angle = angles[-1]
            expected_final = 90.0
            final_error = abs(final_angle - expected_final)
            
            print(f"\nFinal Result:")
            print(f"  Expected: {expected_final:.1f}°")
            print(f"  Actual: {final_angle:.1f}°")
            print(f"  Error: {final_error:.1f}°")
            
            if final_error < 5.0:
                print("✓ PASS: Tracking within tolerance")
            else:
                print("✗ FAIL: Excessive error")
        
        sock.close()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    print("IMU Data Injection Test Suite")
    print("=============================\n")
    
    # Check if bridge is running
    try:
        test_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        test_sock.settimeout(1)
        test_sock.connect(('192.168.64.1', 9999))
        test_sock.close()
        print("✓ Serial bridge is running\n")
    except:
        print("✗ Serial bridge not running!")
        print("Start it on Windows with:")
        print("  cd C:\\Users\\Public")
        print("  python serial_bridge.py")
        exit(1)
    
    # Run tests
    test_injection()
    test_quadrant_motion()
    
    print("\n" + "=" * 40)
    print("Test suite completed!")