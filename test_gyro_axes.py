#!/usr/bin/env python3
"""
Test each gyro axis individually
"""
import socket
import time
import statistics

def connect():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('192.168.64.1', 9999))
    sock.settimeout(0.5)
    return sock

def send_cmd(sock, cmd):
    sock.send(f"{cmd}\n".encode())
    time.sleep(0.2)

def collect_samples(sock, duration=2):
    """Collect orientation samples"""
    buffer = ""
    samples = []
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
            
    return samples

def analyze_axis_response(baseline, test_data):
    """Analyze which axis responds most"""
    if not baseline or not test_data:
        return None, None, None
        
    # Get baseline averages
    base_pitch = statistics.mean([s['pitch'] for s in baseline])
    base_roll = statistics.mean([s['roll'] for s in baseline])
    base_yaw = statistics.mean([s['yaw'] for s in baseline])
    
    # Get test ranges
    test_pitches = [s['pitch'] for s in test_data]
    test_rolls = [s['roll'] for s in test_data]
    test_yaws = [s['yaw'] for s in test_data]
    
    pitch_range = max(test_pitches) - min(test_pitches)
    roll_range = max(test_rolls) - min(test_rolls)
    yaw_range = max(test_yaws) - min(test_yaws)
    
    return pitch_range, roll_range, yaw_range

def main():
    print("Gyro Axis Configuration Test")
    print("=" * 60)
    
    sock = connect()
    print("Connected!")
    
    # Reset and setup
    print("\n1. Resetting orientation...")
    send_cmd(sock, 'r')
    time.sleep(0.5)
    
    print("2. Enabling debug stream...")
    send_cmd(sock, 'd')
    time.sleep(0.5)
    
    # Collect baseline
    print("\n3. Collecting baseline (keep device still)...")
    baseline = collect_samples(sock, 2)
    print(f"   Got {len(baseline)} baseline samples")
    
    if baseline:
        avg_pitch = statistics.mean([s['pitch'] for s in baseline])
        avg_roll = statistics.mean([s['roll'] for s in baseline])
        avg_yaw = statistics.mean([s['yaw'] for s in baseline])
        print(f"   Baseline: Pitch={avg_pitch:.1f}° Roll={avg_roll:.1f}° Yaw={avg_yaw:.1f}°")
    
    # Test each axis
    axes = ['X', 'Y', 'Z']
    cmds = ['TX', 'TY', 'TZ']
    results = {}
    
    for i, (axis, cmd) in enumerate(zip(axes, cmds)):
        print(f"\n4.{i+1}. Testing Gyro {axis} (30° test signal)...")
        
        # Start test signal
        send_cmd(sock, f'{cmd}=30')
        
        # Collect data during test
        test_data = collect_samples(sock, 3)
        
        # Stop test signal
        send_cmd(sock, 'T0=0')
        
        # Analyze response
        pitch_r, roll_r, yaw_r = analyze_axis_response(baseline, test_data)
        
        if pitch_r is not None:
            print(f"   Response ranges:")
            print(f"     Pitch: {pitch_r:6.1f}°")
            print(f"     Roll:  {roll_r:6.1f}°")
            print(f"     Yaw:   {yaw_r:6.1f}°")
            
            # Determine primary axis
            max_r = max(pitch_r, roll_r, yaw_r)
            if max_r < 10:
                print(f"   ⚠ WEAK RESPONSE - may need scaling or polarity flip")
            elif pitch_r == max_r:
                print(f"   → Gyro {axis} primarily affects PITCH")
                results[axis] = 'pitch'
            elif roll_r == max_r:
                print(f"   → Gyro {axis} primarily affects ROLL")
                results[axis] = 'roll'
            else:
                print(f"   → Gyro {axis} primarily affects YAW")
                results[axis] = 'yaw'
        
        time.sleep(1)
    
    # Disable debug
    send_cmd(sock, 'd')
    
    # Analysis and recommendations
    print("\n" + "=" * 60)
    print("ANALYSIS AND RECOMMENDATIONS:")
    print("-" * 60)
    
    if len(results) == 3:
        # Check mapping
        expected = {'X': 'roll', 'Y': 'pitch', 'Z': 'yaw'}
        
        if results == expected:
            print("✓ Axis mapping is correct!")
        else:
            print("✗ Axis mapping needs adjustment:")
            
            # Suggest remapping
            mapping = [0, 1, 2]
            if results['X'] == 'pitch' and results['Y'] == 'roll':
                mapping = [1, 0, 2]
                print("  → Gyro X and Y are swapped")
                print(f"  → Use: AMAP={mapping[0]},{mapping[1]},{mapping[2]}")
            
            # Check for other swaps
            for axis, expected_effect in expected.items():
                if results.get(axis) != expected_effect:
                    print(f"  → Gyro {axis} affects {results.get(axis, 'nothing')} (should be {expected_effect})")
    
    print("\nTest complete!")
    sock.close()

if __name__ == "__main__":
    main()