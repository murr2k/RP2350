#!/usr/bin/env python3
"""
Single session axis test - maintains one connection throughout
"""
import socket
import time
import sys

def main():
    HOST = '192.168.64.1'
    PORT = 9999
    
    print("Single Session Axis Test")
    print("=" * 60)
    
    # Create socket and connect
    print(f"Connecting to {HOST}:{PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    try:
        sock.connect((HOST, PORT))
        sock.settimeout(0.5)
        print("Connected!\n")
        
        # Clear any initial data
        try:
            sock.recv(1024)
        except:
            pass
        
        # Series of commands in single session
        commands = [
            ('r', 'Reset orientation', 1.0),
            ('d', 'Enable debug stream', 1.0),
            (None, 'Collect baseline for 3 seconds', 3.0),
            ('TX=30', 'Test Gyro X with 30° signal', 3.0),
            ('T0=0', 'Stop test signal', 1.0),
            ('TY=30', 'Test Gyro Y with 30° signal', 3.0),
            ('T0=0', 'Stop test signal', 1.0),
            ('TZ=30', 'Test Gyro Z with 30° signal', 3.0),
            ('T0=0', 'Stop test signal', 1.0),
            ('d', 'Disable debug stream', 0.5),
        ]
        
        all_data = []
        
        for cmd, description, wait_time in commands:
            print(f"[{time.strftime('%H:%M:%S')}] {description}")
            
            # Send command if not None
            if cmd:
                sock.send(f"{cmd}\n".encode())
                time.sleep(0.1)
            
            # Collect data for specified time
            start = time.time()
            phase_data = []
            
            while time.time() - start < wait_time:
                try:
                    data = sock.recv(4096)
                    if data:
                        decoded = data.decode('utf-8', errors='ignore')
                        phase_data.append(decoded)
                        
                        # Show sample of data for CSV lines
                        lines = decoded.split('\n')
                        for line in lines:
                            if 'CSV' in line:
                                parts = line.split(',')
                                if len(parts) >= 11:
                                    try:
                                        pitch = float(parts[8])
                                        roll = float(parts[9])
                                        yaw = float(parts[10])
                                        # Print every 10th sample
                                        if int(time.time() * 10) % 10 == 0:
                                            print(f"    P:{pitch:6.1f}° R:{roll:6.1f}° Y:{yaw:6.1f}°")
                                    except:
                                        pass
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"    Error reading data: {e}")
                    break
            
            # Store phase data
            all_data.append({
                'command': cmd,
                'description': description,
                'data': ''.join(phase_data)
            })
            
            print(f"    Collected {len(''.join(phase_data))} bytes\n")
        
        print("Test sequence complete!")
        
        # Analyze collected data
        print("\n" + "=" * 60)
        print("ANALYSIS")
        print("=" * 60)
        
        for phase in all_data:
            csv_lines = [l for l in phase['data'].split('\n') if 'CSV' in l]
            if csv_lines and phase['command'] and phase['command'].startswith('T'):
                print(f"\n{phase['description']}:")
                
                # Parse orientation values
                pitch_vals = []
                roll_vals = []
                yaw_vals = []
                
                for line in csv_lines:
                    parts = line.split(',')
                    if len(parts) >= 11:
                        try:
                            pitch_vals.append(float(parts[8]))
                            roll_vals.append(float(parts[9]))
                            yaw_vals.append(float(parts[10]))
                        except:
                            pass
                
                if pitch_vals:
                    pitch_range = max(pitch_vals) - min(pitch_vals)
                    roll_range = max(roll_vals) - min(roll_vals)
                    yaw_range = max(yaw_vals) - min(yaw_vals)
                    
                    print(f"  Response ranges:")
                    print(f"    Pitch: {pitch_range:.1f}°")
                    print(f"    Roll:  {roll_range:.1f}°")
                    print(f"    Yaw:   {yaw_range:.1f}°")
                    
                    max_response = max(pitch_range, roll_range, yaw_range)
                    if pitch_range == max_response:
                        print(f"  → Primary response: PITCH")
                    elif roll_range == max_response:
                        print(f"  → Primary response: ROLL")
                    else:
                        print(f"  → Primary response: YAW")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("\nClosing connection...")
        sock.close()
        print("Done!")

if __name__ == "__main__":
    main()