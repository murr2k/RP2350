#!/usr/bin/env python3
"""
Simple test script for axis configuration
"""
import socket
import time
import sys

def send_command(sock, cmd):
    """Send command and get response"""
    sock.send(f"{cmd}\n".encode())
    time.sleep(0.1)
    
def read_data(sock, duration=2):
    """Read data for specified duration"""
    sock.settimeout(0.1)
    start = time.time()
    data = []
    
    while time.time() - start < duration:
        try:
            chunk = sock.recv(1024).decode('utf-8', errors='ignore')
            if chunk:
                data.append(chunk)
        except socket.timeout:
            continue
    
    return ''.join(data)

def main():
    HOST = '192.168.64.1'
    PORT = 9999
    
    print("Connecting to device...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))
    print("Connected!")
    
    # Reset orientation
    print("\n1. Resetting orientation...")
    send_command(sock, 'r')
    time.sleep(0.5)
    
    # Enable debug stream
    print("\n2. Enabling debug stream...")
    send_command(sock, 'd')
    
    # Collect baseline data
    print("\n3. Collecting baseline data (device stationary)...")
    baseline = read_data(sock, 3)
    
    # Parse CSV data
    lines = baseline.split('\n')
    csv_lines = [l for l in lines if 'CSV' in l]
    
    if csv_lines:
        print(f"   Received {len(csv_lines)} samples")
        # Parse last few samples
        for line in csv_lines[-5:]:
            parts = line.split(',')
            if len(parts) >= 10:
                try:
                    pitch = float(parts[8])
                    roll = float(parts[9])
                    yaw = float(parts[10]) if len(parts) > 10 else 0
                    print(f"   Pitch: {pitch:6.1f}° Roll: {roll:6.1f}° Yaw: {yaw:6.1f}°")
                except:
                    pass
    
    # Test gyro X axis
    print("\n4. Testing Gyro X axis (30° amplitude)...")
    send_command(sock, 'TX=30')
    test_data = read_data(sock, 3)
    send_command(sock, 'T0=0')  # Stop test
    
    # Analyze response
    lines = test_data.split('\n')
    csv_lines = [l for l in lines if 'CSV' in l]
    
    if csv_lines:
        pitch_vals = []
        roll_vals = []
        yaw_vals = []
        
        for line in csv_lines:
            parts = line.split(',')
            if len(parts) >= 10:
                try:
                    pitch_vals.append(float(parts[8]))
                    roll_vals.append(float(parts[9]))
                    yaw_vals.append(float(parts[10]) if len(parts) > 10 else 0)
                except:
                    pass
        
        if pitch_vals:
            pitch_range = max(pitch_vals) - min(pitch_vals)
            roll_range = max(roll_vals) - min(roll_vals)
            yaw_range = max(yaw_vals) - min(yaw_vals)
            
            print(f"   Response ranges:")
            print(f"     Pitch: {pitch_range:.1f}°")
            print(f"     Roll:  {roll_range:.1f}°")
            print(f"     Yaw:   {yaw_range:.1f}°")
            
            max_response = max(pitch_range, roll_range, yaw_range)
            if pitch_range == max_response:
                print("   → Gyro X primarily affects PITCH")
            elif roll_range == max_response:
                print("   → Gyro X primarily affects ROLL")
            else:
                print("   → Gyro X primarily affects YAW")
    
    # Test gyro Y axis
    print("\n5. Testing Gyro Y axis (30° amplitude)...")
    send_command(sock, 'TY=30')
    test_data = read_data(sock, 3)
    send_command(sock, 'T0=0')
    
    # Analyze Y response
    lines = test_data.split('\n')
    csv_lines = [l for l in lines if 'CSV' in l]
    
    if csv_lines:
        pitch_vals = []
        roll_vals = []
        yaw_vals = []
        
        for line in csv_lines:
            parts = line.split(',')
            if len(parts) >= 10:
                try:
                    pitch_vals.append(float(parts[8]))
                    roll_vals.append(float(parts[9]))
                    yaw_vals.append(float(parts[10]) if len(parts) > 10 else 0)
                except:
                    pass
        
        if pitch_vals:
            pitch_range = max(pitch_vals) - min(pitch_vals)
            roll_range = max(roll_vals) - min(roll_vals)
            yaw_range = max(yaw_vals) - min(yaw_vals)
            
            print(f"   Response ranges:")
            print(f"     Pitch: {pitch_range:.1f}°")
            print(f"     Roll:  {roll_range:.1f}°")
            print(f"     Yaw:   {yaw_range:.1f}°")
            
            max_response = max(pitch_range, roll_range, yaw_range)
            if pitch_range == max_response:
                print("   → Gyro Y primarily affects PITCH")
            elif roll_range == max_response:
                print("   → Gyro Y primarily affects ROLL")
            else:
                print("   → Gyro Y primarily affects YAW")
    
    # Test gyro Z axis
    print("\n6. Testing Gyro Z axis (30° amplitude)...")
    send_command(sock, 'TZ=30')
    test_data = read_data(sock, 3)
    send_command(sock, 'T0=0')
    
    # Analyze Z response
    lines = test_data.split('\n')
    csv_lines = [l for l in lines if 'CSV' in l]
    
    if csv_lines:
        pitch_vals = []
        roll_vals = []
        yaw_vals = []
        
        for line in csv_lines:
            parts = line.split(',')
            if len(parts) >= 10:
                try:
                    pitch_vals.append(float(parts[8]))
                    roll_vals.append(float(parts[9]))
                    yaw_vals.append(float(parts[10]) if len(parts) > 10 else 0)
                except:
                    pass
        
        if pitch_vals:
            pitch_range = max(pitch_vals) - min(pitch_vals)
            roll_range = max(roll_vals) - min(roll_vals)
            yaw_range = max(yaw_vals) - min(yaw_vals)
            
            print(f"   Response ranges:")
            print(f"     Pitch: {pitch_range:.1f}°")
            print(f"     Roll:  {roll_range:.1f}°")
            print(f"     Yaw:   {yaw_range:.1f}°")
            
            max_response = max(pitch_range, roll_range, yaw_range)
            if pitch_range == max_response:
                print("   → Gyro Z primarily affects PITCH")
            elif roll_range == max_response:
                print("   → Gyro Z primarily affects ROLL")
            else:
                print("   → Gyro Z primarily affects YAW")
    
    # Disable debug
    send_command(sock, 'd')
    
    print("\n7. Test complete!")
    print("\nRecommendations:")
    print("  - If axes are swapped, use AMAP command (e.g., AMAP=1,0,2)")
    print("  - If axes are inverted, use AG commands (e.g., AG0=-1)")
    print("  - If scaling needed, use AS commands (e.g., AS0=1.5)")
    
    sock.close()

if __name__ == "__main__":
    main()