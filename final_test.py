#!/usr/bin/env python3
"""
Final axis configuration test after remapping
"""
import socket
import time
import statistics

def test_axis(host, port, axis_cmd, duration=3):
    """Test a single axis and return statistics"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    sock.settimeout(0.1)
    
    # Send test command
    sock.send(f"{axis_cmd}\n".encode())
    time.sleep(0.2)
    
    # Collect data
    samples = []
    buffer = ""
    start = time.time()
    
    while time.time() - start < duration:
        try:
            data = sock.recv(1024).decode('utf-8', errors='ignore')
            buffer += data
            
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                if 'CSV' in line:
                    parts = line.split(',')
                    if len(parts) >= 11:
                        try:
                            samples.append({
                                'pitch': float(parts[8]),
                                'roll': float(parts[9]),
                                'yaw': float(parts[10])
                            })
                        except:
                            pass
        except socket.timeout:
            continue
    
    # Stop test
    sock.send("T0=0\n".encode())
    time.sleep(0.2)
    
    sock.close()
    
    # Analyze
    if samples:
        pitch_range = max(s['pitch'] for s in samples) - min(s['pitch'] for s in samples)
        roll_range = max(s['roll'] for s in samples) - min(s['roll'] for s in samples)
        yaw_range = max(s['yaw'] for s in samples) - min(s['yaw'] for s in samples)
        return pitch_range, roll_range, yaw_range
    
    return 0, 0, 0

def main():
    HOST = '192.168.64.1'
    PORT = 9999
    
    print("Final Axis Configuration Test")
    print("=" * 60)
    print("Testing with remapped axes (AMAP=1,0,2)")
    print()
    
    # Test each axis
    tests = [
        ('TX=30', 'Gyro X'),
        ('TY=30', 'Gyro Y'),
        ('TZ=30', 'Gyro Z')
    ]
    
    for cmd, name in tests:
        print(f"Testing {name}...")
        pitch_r, roll_r, yaw_r = test_axis(HOST, PORT, cmd, 3)
        
        print(f"  Response ranges:")
        print(f"    Pitch: {pitch_r:6.1f}°")
        print(f"    Roll:  {roll_r:6.1f}°")
        print(f"    Yaw:   {yaw_r:6.1f}°")
        
        # Determine primary
        max_r = max(pitch_r, roll_r, yaw_r)
        if pitch_r == max_r:
            primary = "PITCH"
        elif roll_r == max_r:
            primary = "ROLL"
        else:
            primary = "YAW"
        
        print(f"  → Primary: {primary}")
        
        # Check if correct
        expected = {'Gyro X': 'ROLL', 'Gyro Y': 'PITCH', 'Gyro Z': 'YAW'}
        if primary == expected[name]:
            print(f"  ✓ CORRECT!")
        else:
            print(f"  ✗ Should be {expected[name]}")
        
        print()
        time.sleep(1)
    
    print("=" * 60)
    print("Summary:")
    print("  With AMAP=1,0,2 the axes should now be properly mapped.")
    print("  If axes are still wrong, additional adjustments needed.")

if __name__ == "__main__":
    main()