#!/usr/bin/env python3
"""
Real-time visualization of sensor fusion with test data injection
Shows both expected and actual orientation during quadrant tests
"""

import socket
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, FancyBboxPatch
import threading
from collections import deque

class SensorFusionVisualizer:
    def __init__(self):
        self.host = '192.168.64.1'
        self.port = 9999
        self.sock = None
        
        # Data buffers
        self.max_points = 200
        self.time_buffer = deque(maxlen=self.max_points)
        self.pitch_actual = deque(maxlen=self.max_points)
        self.roll_actual = deque(maxlen=self.max_points)
        self.yaw_actual = deque(maxlen=self.max_points)
        self.pitch_expected = deque(maxlen=self.max_points)
        self.roll_expected = deque(maxlen=self.max_points)
        self.yaw_expected = deque(maxlen=self.max_points)
        
        # Test state
        self.test_running = False
        self.current_quadrant = 0
        self.test_start_time = 0
        self.collecting = True
        
        # Setup plot
        self.setup_plot()
        
    def setup_plot(self):
        """Setup the matplotlib figure and axes"""
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.suptitle('Kalman Filter Sensor Fusion - Real-time Analysis', fontsize=14, fontweight='bold')
        
        # Create grid layout
        gs = self.fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
        
        # Time series plots
        self.ax_pitch = self.fig.add_subplot(gs[0, :2])
        self.ax_roll = self.fig.add_subplot(gs[1, :2])
        self.ax_yaw = self.fig.add_subplot(gs[2, :2])
        
        # 3D orientation visualization
        self.ax_3d = self.fig.add_subplot(gs[:2, 2])
        
        # Status panel
        self.ax_status = self.fig.add_subplot(gs[2, 2])
        self.ax_status.axis('off')
        
        # Setup time series axes
        for ax, title, color in zip(
            [self.ax_pitch, self.ax_roll, self.ax_yaw],
            ['Pitch', 'Roll', 'Yaw'],
            ['red', 'green', 'blue']
        ):
            ax.set_title(title, fontweight='bold', color=color)
            ax.set_ylabel('Angle (°)')
            ax.set_xlim(0, 10)
            ax.set_ylim(-180, 180)
            ax.grid(True, alpha=0.3)
            ax.axhline(y=0, color='k', linestyle='-', alpha=0.2)
        
        self.ax_yaw.set_xlabel('Time (s)')
        
        # Initialize lines
        self.line_pitch_actual, = self.ax_pitch.plot([], [], 'r-', label='Actual', linewidth=2)
        self.line_pitch_expected, = self.ax_pitch.plot([], [], 'r--', label='Expected', alpha=0.5)
        
        self.line_roll_actual, = self.ax_roll.plot([], [], 'g-', label='Actual', linewidth=2)
        self.line_roll_expected, = self.ax_roll.plot([], [], 'g--', label='Expected', alpha=0.5)
        
        self.line_yaw_actual, = self.ax_yaw.plot([], [], 'b-', label='Actual', linewidth=2)
        self.line_yaw_expected, = self.ax_yaw.plot([], [], 'b--', label='Expected', alpha=0.5)
        
        # Add legends
        for ax in [self.ax_pitch, self.ax_roll, self.ax_yaw]:
            ax.legend(loc='upper right')
        
        # Setup 3D visualization
        self.setup_3d_view()
        
        # Setup status text
        self.status_text = self.ax_status.text(0.1, 0.5, '', fontsize=10, 
                                               transform=self.ax_status.transAxes)
        
    def setup_3d_view(self):
        """Setup the 3D orientation visualization"""
        self.ax_3d.set_title('3D Orientation', fontweight='bold')
        self.ax_3d.set_xlim(-1.5, 1.5)
        self.ax_3d.set_ylim(-1.5, 1.5)
        self.ax_3d.set_aspect('equal')
        self.ax_3d.grid(True, alpha=0.3)
        
        # Create a simple box representation
        self.box = FancyBboxPatch((-0.5, -0.3), 1.0, 0.6,
                                  boxstyle="round,pad=0.1",
                                  facecolor='lightblue',
                                  edgecolor='navy',
                                  linewidth=2)
        self.ax_3d.add_patch(self.box)
        
        # Reference axes
        self.ax_3d.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.1, 
                        fc='red', ec='red', alpha=0.3)
        self.ax_3d.arrow(0, 0, 0, 1, head_width=0.1, head_length=0.1, 
                        fc='green', ec='green', alpha=0.3)
        
        self.ax_3d.text(1.2, 0, 'X', color='red', fontsize=8)
        self.ax_3d.text(0, 1.2, 'Y', color='green', fontsize=8)
        
    def connect(self):
        """Connect to the device via TCP bridge"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(0.1)
            print(f"Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def collect_data(self):
        """Background thread to collect data from device"""
        buffer = ""
        while self.collecting:
            try:
                data = self.sock.recv(1024).decode('utf-8', errors='ignore')
                buffer += data
                
                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if 'CSV' in line:
                        self.parse_csv(line)
                        
            except socket.timeout:
                continue
            except Exception as e:
                if self.collecting:
                    print(f"Collection error: {e}")
                break
    
    def parse_csv(self, line):
        """Parse CSV line from device"""
        try:
            parts = line.split(',')
            if len(parts) >= 10:
                # Extract angles
                pitch = float(parts[7])
                roll = float(parts[8])
                yaw = float(parts[9])
                
                # Calculate time
                current_time = time.time() - self.test_start_time
                
                # Store data
                self.time_buffer.append(current_time)
                self.pitch_actual.append(pitch)
                self.roll_actual.append(roll)
                self.yaw_actual.append(yaw)
                
        except Exception:
            pass
    
    def run_quadrant_test(self, quadrant):
        """Run a specific quadrant test pattern"""
        self.current_quadrant = quadrant
        self.test_running = True
        
        # Reset device
        self.sock.send(b'r\n')
        time.sleep(0.5)
        
        # Clear buffers
        self.time_buffer.clear()
        self.pitch_actual.clear()
        self.roll_actual.clear()
        self.yaw_actual.clear()
        self.pitch_expected.clear()
        self.roll_expected.clear()
        self.yaw_expected.clear()
        
        self.test_start_time = time.time()
        
        # Test parameters
        rate = 30.0  # deg/s
        duration = 4.0  # seconds
        dt = 0.02  # 50Hz
        
        # Generate test pattern based on quadrant
        for i in range(int(duration / dt)):
            t = i * dt
            
            # Calculate expected angles and generate IMU data
            if quadrant == 1:  # +X rotation
                gx, gy, gz = rate, 0, 0
                exp_pitch = rate * t
                exp_roll = 0
                exp_yaw = 0
            elif quadrant == 2:  # +Y rotation
                gx, gy, gz = 0, rate, 0
                exp_pitch = 0
                exp_roll = rate * t
                exp_yaw = 0
            elif quadrant == 3:  # +Z rotation
                gx, gy, gz = 0, 0, rate
                exp_pitch = 0
                exp_roll = 0
                exp_yaw = rate * t
            elif quadrant == 4:  # -X rotation
                gx, gy, gz = -rate, 0, 0
                exp_pitch = -rate * t
                exp_roll = 0
                exp_yaw = 0
            elif quadrant == 5:  # Combined XY
                gx, gy, gz = rate/2, rate/2, 0
                exp_pitch = rate/2 * t
                exp_roll = rate/2 * t
                exp_yaw = 0
            elif quadrant == 6:  # Combined XZ
                gx, gy, gz = rate/2, 0, rate/2
                exp_pitch = rate/2 * t
                exp_roll = 0
                exp_yaw = rate/2 * t
            elif quadrant == 7:  # Combined YZ
                gx, gy, gz = 0, rate/2, rate/2
                exp_pitch = 0
                exp_roll = rate/2 * t
                exp_yaw = rate/2 * t
            else:  # All axes
                gx, gy, gz = rate/3, rate/3, rate/3
                exp_pitch = rate/3 * t
                exp_roll = rate/3 * t
                exp_yaw = rate/3 * t
            
            # Calculate gravity vector based on orientation
            pitch_rad = np.radians(exp_pitch)
            roll_rad = np.radians(exp_roll)
            
            ax = np.sin(pitch_rad)
            ay = -np.sin(roll_rad) * np.cos(pitch_rad)
            az = np.cos(roll_rad) * np.cos(pitch_rad)
            
            # Store expected values
            self.pitch_expected.append(exp_pitch)
            self.roll_expected.append(exp_roll)
            self.yaw_expected.append(exp_yaw)
            
            # Inject IMU data
            cmd = f"I{ax:.3f},{ay:.3f},{az:.3f},{gx:.1f},{gy:.1f},{gz:.1f}\n"
            self.sock.send(cmd.encode())
            
            time.sleep(dt)
        
        self.test_running = False
    
    def update_plot(self, frame):
        """Update plot with latest data"""
        if not self.time_buffer:
            return
        
        # Get data arrays
        t = np.array(self.time_buffer)
        
        # Update time series plots
        if len(self.pitch_actual) > 0:
            self.line_pitch_actual.set_data(t[:len(self.pitch_actual)], list(self.pitch_actual))
            self.line_roll_actual.set_data(t[:len(self.roll_actual)], list(self.roll_actual))
            self.line_yaw_actual.set_data(t[:len(self.yaw_actual)], list(self.yaw_actual))
        
        if len(self.pitch_expected) > 0:
            t_exp = t[:len(self.pitch_expected)]
            self.line_pitch_expected.set_data(t_exp, list(self.pitch_expected))
            self.line_roll_expected.set_data(t_exp, list(self.roll_expected))
            self.line_yaw_expected.set_data(t_exp, list(self.yaw_expected))
        
        # Update 3D visualization
        if self.pitch_actual and self.roll_actual:
            pitch = self.pitch_actual[-1]
            roll = self.roll_actual[-1]
            
            # Simple 2D rotation visualization
            angle = np.radians(roll)
            cos_a = np.cos(angle)
            sin_a = np.sin(angle)
            
            # Rotate box corners
            corners = np.array([[-0.5, -0.3], [0.5, -0.3], [0.5, 0.3], [-0.5, 0.3]])
            rotated = np.zeros_like(corners)
            for i, (x, y) in enumerate(corners):
                rotated[i] = [x * cos_a - y * sin_a, x * sin_a + y * cos_a]
            
            # Update box (simplified - just show roll)
            self.box.set_x(rotated[0, 0])
            self.box.set_y(rotated[0, 1])
            
        # Update status
        status_text = f"Quadrant Test: {self.current_quadrant}\n"
        if self.pitch_actual and self.pitch_expected:
            pitch_err = abs(self.pitch_actual[-1] - self.pitch_expected[-1]) if self.pitch_expected else 0
            roll_err = abs(self.roll_actual[-1] - self.roll_expected[-1]) if self.roll_expected else 0
            yaw_err = abs(self.yaw_actual[-1] - self.yaw_expected[-1]) if self.yaw_expected else 0
            
            status_text += f"Errors:\n"
            status_text += f"  Pitch: {pitch_err:.1f}°\n"
            status_text += f"  Roll: {roll_err:.1f}°\n"
            status_text += f"  Yaw: {yaw_err:.1f}°\n"
            
            # Overall status
            max_err = max(pitch_err, roll_err, yaw_err)
            if max_err < 3:
                status_text += "\nStatus: EXCELLENT"
            elif max_err < 5:
                status_text += "\nStatus: GOOD"
            elif max_err < 10:
                status_text += "\nStatus: ACCEPTABLE"
            else:
                status_text += "\nStatus: NEEDS TUNING"
        
        self.status_text.set_text(status_text)
        
        # Adjust x-axis limits
        if t[-1] > 10:
            for ax in [self.ax_pitch, self.ax_roll, self.ax_yaw]:
                ax.set_xlim(t[-1] - 10, t[-1])
    
    def run_visualization(self):
        """Main visualization loop"""
        if not self.connect():
            return
        
        # Start data collection thread
        collector = threading.Thread(target=self.collect_data)
        collector.daemon = True
        collector.start()
        
        # Start test sequence in background
        def test_sequence():
            time.sleep(1)  # Initial delay
            for quadrant in range(1, 9):
                print(f"Running Quadrant {quadrant} test...")
                self.run_quadrant_test(quadrant)
                time.sleep(2)  # Pause between tests
        
        tester = threading.Thread(target=test_sequence)
        tester.daemon = True
        tester.start()
        
        # Setup animation
        ani = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)
        
        plt.show()
        
        # Cleanup
        self.collecting = False
        if self.sock:
            self.sock.close()

def main():
    print("Kalman Filter Sensor Fusion Visualizer")
    print("=" * 40)
    print("This will run 8 quadrant tests and visualize the results")
    print("Make sure the serial bridge is running and device is connected")
    print()
    
    visualizer = SensorFusionVisualizer()
    visualizer.run_visualization()

if __name__ == "__main__":
    main()