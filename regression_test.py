#!/usr/bin/env python3
"""
Regression Test System for 6-DOF Kalman Filter
Tests 8-quadrant motion patterns with sensor fusion
"""

import socket
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import struct
import threading

class KalmanRegressionTest:
    def __init__(self, host='192.168.64.1', port=9999):
        self.host = host
        self.port = port
        self.sock = None
        
        # Data storage
        self.time_data = deque(maxlen=500)
        self.pitch_data = deque(maxlen=500)
        self.roll_data = deque(maxlen=500)
        self.yaw_data = deque(maxlen=500)
        
        # Expected values for current test
        self.expected_pitch = deque(maxlen=500)
        self.expected_roll = deque(maxlen=500)
        self.expected_yaw = deque(maxlen=500)
        
        # Test state
        self.current_test = ""
        self.test_start_time = 0
        self.collecting = False
        
    def connect(self):
        """Connect to device via TCP bridge"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))
        self.sock.settimeout(0.1)
        print(f"Connected to {self.host}:{self.port}")
        
    def disconnect(self):
        """Disconnect from device"""
        if self.sock:
            self.sock.close()
            self.sock = None
            
    def send_command(self, cmd):
        """Send command to device"""
        if self.sock:
            self.sock.send((cmd + '\n').encode())
            time.sleep(0.05)
            
    def reset_device(self):
        """Reset device and Kalman filter"""
        print("Resetting device...")
        self.send_command('r')  # Reset
        time.sleep(0.5)
        self.send_command('c')  # Calibrate
        time.sleep(1.0)
        
    def inject_imu_data(self, ax, ay, az, gx, gy, gz):
        """
        Inject synthetic IMU data directly to Kalman filter
        Format: I<ax>,<ay>,<az>,<gx>,<gy>,<gz>
        Values in g and deg/s
        """
        cmd = f"I{ax:.3f},{ay:.3f},{az:.3f},{gx:.1f},{gy:.1f},{gz:.1f}"
        self.send_command(cmd)
        
    def generate_quadrant_motion(self, quadrant, duration=3.0, sample_rate=50):
        """
        Generate motion pattern for specific quadrant
        Quadrants defined by primary rotation axes:
        1: +X+Y (positive pitch and roll)
        2: -X+Y (negative pitch, positive roll)
        3: -X-Y (negative pitch and roll)
        4: +X-Y (positive pitch, negative roll)
        5: +Z rotation (positive yaw)
        6: -Z rotation (negative yaw)
        7: Combined +X+Y+Z
        8: Combined -X-Y-Z
        """
        samples = int(duration * sample_rate)
        t = np.linspace(0, duration, samples)
        
        # Base gravity vector (device at rest)
        ax_base, ay_base, az_base = 0.0, 0.0, 1.0
        
        # Initialize arrays
        ax = np.full(samples, ax_base)
        ay = np.full(samples, ay_base)
        az = np.full(samples, az_base)
        gx = np.zeros(samples)
        gy = np.zeros(samples)
        gz = np.zeros(samples)
        
        # Expected angles
        exp_pitch = np.zeros(samples)
        exp_roll = np.zeros(samples)
        exp_yaw = np.zeros(samples)
        
        # Angular velocity for rotations (deg/s)
        rate = 30.0
        
        if quadrant == 1:  # +X+Y rotation
            # Rotate around X (affects pitch)
            gx[:] = rate
            exp_pitch = rate * t
            # Then rotate around Y (affects roll)
            gy[samples//2:] = rate
            exp_roll[samples//2:] = rate * (t[samples//2:] - t[samples//2])
            
        elif quadrant == 2:  # -X+Y rotation
            gx[:] = -rate
            exp_pitch = -rate * t
            gy[samples//2:] = rate
            exp_roll[samples//2:] = rate * (t[samples//2:] - t[samples//2])
            
        elif quadrant == 3:  # -X-Y rotation
            gx[:] = -rate
            exp_pitch = -rate * t
            gy[samples//2:] = -rate
            exp_roll[samples//2:] = -rate * (t[samples//2:] - t[samples//2])
            
        elif quadrant == 4:  # +X-Y rotation
            gx[:] = rate
            exp_pitch = rate * t
            gy[samples//2:] = -rate
            exp_roll[samples//2:] = -rate * (t[samples//2:] - t[samples//2])
            
        elif quadrant == 5:  # +Z rotation
            gz[:] = rate
            exp_yaw = rate * t
            
        elif quadrant == 6:  # -Z rotation
            gz[:] = -rate
            exp_yaw = -rate * t
            
        elif quadrant == 7:  # Combined +X+Y+Z
            # Simultaneous rotation on all axes
            gx[:] = rate * 0.7
            gy[:] = rate * 0.7
            gz[:] = rate * 0.7
            exp_pitch = rate * 0.7 * t
            exp_roll = rate * 0.7 * t
            exp_yaw = rate * 0.7 * t
            
        elif quadrant == 8:  # Combined -X-Y-Z
            gx[:] = -rate * 0.7
            gy[:] = -rate * 0.7
            gz[:] = -rate * 0.7
            exp_pitch = -rate * 0.7 * t
            exp_roll = -rate * 0.7 * t
            exp_yaw = -rate * 0.7 * t
            
        # Add gravity rotation based on orientation
        for i in range(samples):
            # Rotate gravity vector based on current orientation
            pitch_rad = np.radians(exp_pitch[i])
            roll_rad = np.radians(exp_roll[i])
            
            # Apply rotation to gravity
            ax[i] = np.sin(pitch_rad)
            ay[i] = -np.sin(roll_rad) * np.cos(pitch_rad)
            az[i] = np.cos(roll_rad) * np.cos(pitch_rad)
            
        return t, ax, ay, az, gx, gy, gz, exp_pitch, exp_roll, exp_yaw
        
    def run_quadrant_test(self, quadrant):
        """Run test for specific quadrant"""
        print(f"\n=== Testing Quadrant {quadrant} ===")
        self.current_test = f"Quadrant {quadrant}"
        
        # Reset device
        self.reset_device()
        
        # Clear data
        self.time_data.clear()
        self.pitch_data.clear()
        self.roll_data.clear()
        self.yaw_data.clear()
        self.expected_pitch.clear()
        self.expected_roll.clear()
        self.expected_yaw.clear()
        
        # Generate test pattern
        t, ax, ay, az, gx, gy, gz, exp_p, exp_r, exp_y = self.generate_quadrant_motion(quadrant)
        
        # Enable debug stream
        self.send_command('d')
        time.sleep(0.1)
        
        # Start data collection thread
        self.collecting = True
        collector = threading.Thread(target=self.collect_data)
        collector.start()
        
        # Inject test data
        self.test_start_time = time.time()
        for i in range(len(t)):
            self.inject_imu_data(ax[i], ay[i], az[i], gx[i], gy[i], gz[i])
            
            # Store expected values
            self.expected_pitch.append(exp_p[i])
            self.expected_roll.append(exp_r[i])
            self.expected_yaw.append(exp_y[i])
            
            time.sleep(0.02)  # 50Hz update rate
            
        # Stop after test
        time.sleep(0.5)
        self.send_command('d')  # Stop debug stream
        self.collecting = False
        collector.join()
        
        # Analyze results
        self.analyze_results(quadrant)
        
    def collect_data(self):
        """Collect orientation data from device"""
        buffer = ""
        while self.collecting:
            try:
                data = self.sock.recv(1024).decode('utf-8', errors='ignore')
                buffer += data
                
                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if 'CSV' in line:
                        self.parse_csv_line(line)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Collection error: {e}")
                break
                
    def parse_csv_line(self, line):
        """Parse CSV debug output"""
        try:
            # Format: CSV,timestamp,ax,ay,az,gx,gy,gz,pitch,roll,yaw,...
            parts = line.split(',')
            if len(parts) >= 11:
                timestamp = float(parts[1])
                pitch = float(parts[8])
                roll = float(parts[9])
                yaw = float(parts[10])
                
                # Store data
                rel_time = time.time() - self.test_start_time
                self.time_data.append(rel_time)
                self.pitch_data.append(pitch)
                self.roll_data.append(roll)
                self.yaw_data.append(yaw)
        except Exception:
            pass
            
    def analyze_results(self, quadrant):
        """Analyze test results for a quadrant"""
        if len(self.pitch_data) < 10:
            print(f"Insufficient data for quadrant {quadrant}")
            return
            
        # Convert to arrays
        pitch = np.array(self.pitch_data)
        roll = np.array(self.roll_data)
        yaw = np.array(self.yaw_data)
        
        exp_pitch = np.array(self.expected_pitch)[:len(pitch)]
        exp_roll = np.array(self.expected_roll)[:len(roll)]
        exp_yaw = np.array(self.expected_yaw)[:len(yaw)]
        
        # Calculate errors
        pitch_error = np.mean(np.abs(pitch - exp_pitch))
        roll_error = np.mean(np.abs(roll - exp_roll))
        yaw_error = np.mean(np.abs(yaw - exp_yaw))
        
        # Check drift (final vs expected)
        pitch_drift = abs(pitch[-1] - exp_pitch[-1])
        roll_drift = abs(roll[-1] - exp_roll[-1])
        yaw_drift = abs(yaw[-1] - exp_yaw[-1])
        
        print(f"\nQuadrant {quadrant} Results:")
        print(f"  Mean Absolute Error:")
        print(f"    Pitch: {pitch_error:.2f}°")
        print(f"    Roll:  {roll_error:.2f}°")
        print(f"    Yaw:   {yaw_error:.2f}°")
        print(f"  Final Drift:")
        print(f"    Pitch: {pitch_drift:.2f}°")
        print(f"    Roll:  {roll_drift:.2f}°")
        print(f"    Yaw:   {yaw_drift:.2f}°")
        
        # Pass/Fail criteria
        max_error = 5.0  # degrees
        max_drift = 3.0  # degrees
        
        passed = True
        if pitch_error > max_error or roll_error > max_error or yaw_error > max_error:
            print(f"  FAILED: Excessive tracking error")
            passed = False
        if pitch_drift > max_drift or roll_drift > max_drift or yaw_drift > max_drift:
            print(f"  FAILED: Excessive drift")
            passed = False
        if passed:
            print(f"  PASSED")
            
        return passed
        
    def plot_results(self):
        """Create interactive plot of all test results"""
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle('Kalman Filter Regression Test Results')
        
        # Setup axes
        for ax, label in zip(axes, ['Pitch (°)', 'Roll (°)', 'Yaw (°)']):
            ax.set_ylabel(label)
            ax.grid(True, alpha=0.3)
            ax.set_xlim(0, 4)
            ax.set_ylim(-100, 100)
        axes[-1].set_xlabel('Time (s)')
        
        # Plot lines
        lines = []
        for ax, data, exp_data, color in zip(
            axes,
            [self.pitch_data, self.roll_data, self.yaw_data],
            [self.expected_pitch, self.expected_roll, self.expected_yaw],
            ['r', 'g', 'b']
        ):
            line_actual, = ax.plot([], [], color=color, label='Actual')
            line_expected, = ax.plot([], [], f'{color}--', alpha=0.5, label='Expected')
            ax.legend(loc='upper right')
            lines.extend([line_actual, line_expected])
            
        def update(frame):
            # Update actual data
            if self.time_data:
                t = list(self.time_data)
                lines[0].set_data(t, list(self.pitch_data))
                lines[2].set_data(t, list(self.roll_data))
                lines[4].set_data(t, list(self.yaw_data))
                
                # Update expected data
                lines[1].set_data(t[:len(self.expected_pitch)], list(self.expected_pitch))
                lines[3].set_data(t[:len(self.expected_roll)], list(self.expected_roll))
                lines[5].set_data(t[:len(self.expected_yaw)], list(self.expected_yaw))
                
            return lines
            
        ani = FuncAnimation(fig, update, interval=100, blit=True)
        plt.show()
        return ani
        
    def run_all_tests(self):
        """Run complete regression test suite"""
        print("=" * 60)
        print("6-DOF Kalman Filter Regression Test Suite")
        print("=" * 60)
        
        results = {}
        
        try:
            self.connect()
            
            # Run tests for all 8 quadrants
            for quadrant in range(1, 9):
                passed = self.run_quadrant_test(quadrant)
                results[f"Quadrant {quadrant}"] = passed
                time.sleep(1)  # Pause between tests
                
        finally:
            self.disconnect()
            
        # Summary
        print("\n" + "=" * 60)
        print("TEST SUMMARY")
        print("=" * 60)
        
        total = len(results)
        passed = sum(1 for p in results.values() if p)
        
        for test, result in results.items():
            status = "PASSED" if result else "FAILED"
            print(f"{test}: {status}")
            
        print(f"\nTotal: {passed}/{total} tests passed")
        
        if passed == total:
            print("✓ All tests passed!")
        else:
            print("✗ Some tests failed - processing chain adjustment needed")
            
        return results

def main():
    """Main test execution"""
    tester = KalmanRegressionTest()
    
    # Run all regression tests
    results = tester.run_all_tests()
    
    # Show plot if any data collected
    if tester.time_data:
        print("\nShowing plot of last test...")
        ani = tester.plot_results()
        input("Press Enter to exit...")

if __name__ == "__main__":
    main()