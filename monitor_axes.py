#!/usr/bin/env python3
"""
Live monitoring of axis data
"""
import socket
import time
import sys

def connect():
    """Create new connection"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('192.168.64.1', 9999))
    sock.settimeout(0.1)
    return sock

def main():
    print("Axis Monitor - Press Ctrl+C to exit")
    print("=" * 60)
    
    sock = connect()
    print("Connected to device")
    
    # Send reset and enable debug
    sock.send(b"r\n")
    time.sleep(0.5)
    sock.send(b"d\n")
    time.sleep(0.5)
    
    print("\nMonitoring orientation (device should be stationary):")
    print("-" * 60)
    
    last_pitch = 0
    last_roll = 0
    last_yaw = 0
    sample_count = 0
    
    buffer = ""
    
    try:
        while True:
            try:
                data = sock.recv(1024).decode('utf-8', errors='ignore')
                buffer += data
                
                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    
                    if 'CSV' in line:
                        parts = line.split(',')
                        if len(parts) >= 11:
                            try:
                                # Parse CSV: CSV,time,raw_gx,raw_gy,raw_gz,ax,ay,az,pitch,roll,yaw
                                pitch = float(parts[8])
                                roll = float(parts[9])
                                yaw = float(parts[10])
                                
                                # Calculate drift
                                pitch_drift = pitch - last_pitch
                                roll_drift = roll - last_roll
                                yaw_drift = yaw - last_yaw
                                
                                sample_count += 1
                                
                                # Display every 10th sample
                                if sample_count % 10 == 0:
                                    print(f"\rPitch: {pitch:7.2f}° Roll: {roll:7.2f}° Yaw: {yaw:7.2f}° | "
                                          f"Drift: P:{pitch_drift:+6.2f} R:{roll_drift:+6.2f} Y:{yaw_drift:+6.2f}",
                                          end='', flush=True)
                                
                                last_pitch = pitch
                                last_roll = roll
                                last_yaw = yaw
                                
                            except (ValueError, IndexError):
                                pass
                                
            except socket.timeout:
                continue
            except (ConnectionResetError, BrokenPipeError):
                print("\n\nConnection lost, reconnecting...")
                sock.close()
                time.sleep(1)
                sock = connect()
                sock.send(b"d\n")
                
    except KeyboardInterrupt:
        print("\n\nStopping monitor...")
        sock.send(b"d\n")  # Disable debug
        sock.close()
        print("Done!")

if __name__ == "__main__":
    main()