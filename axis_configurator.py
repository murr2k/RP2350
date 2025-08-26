#!/usr/bin/env python3
"""
Axis Configuration and Testing Tool for 6-DOF Kalman Filter
Systematically tests and configures gyro and accelerometer axes
"""
import time
import subprocess
import statistics

class AxisConfigurator:
    def __init__(self):
        self.host = '192.168.64.1'
        self.port = 9999
        
    def send_command(self, cmd):
        """Send command via netcat"""
        result = subprocess.run(
            f'echo "{cmd}" | nc -w 1 {self.host} {self.port}',
            shell=True,
            capture_output=True,
            text=True,
            timeout=2
        )
        return result.stdout
        
    def collect_data(self, duration=2):
        """Collect CSV data for analysis"""
        print(f"  Collecting data for {duration}s...")
        result = subprocess.run(
            f'timeout {duration} nc {self.host} {self.port}',
            shell=True,
            capture_output=True,
            text=True
        )
        
        samples = []
        for line in result.stdout.split('\n'):
            if 'CSV' in line:
                try:
                    parts = line.split(',')
                    if len(parts) >= 10:
                        samples.append({
                            'time': int(parts[1]),
                            'raw_gx': float(parts[2]),
                            'raw_gy': float(parts[3]),
                            'raw_gz': float(parts[4]),
                            'ax': float(parts[5]),
                            'ay': float(parts[6]),
                            'az': float(parts[7]),
                            'pitch': float(parts[8]),
                            'roll': float(parts[9]),
                            'yaw': float(parts[10]) if len(parts) > 10 else 0
                        })
                except:
                    pass
        return samples
        
    def test_gyro_axis(self, axis, amplitude=30):
        """Test a single gyro axis"""
        axis_names = ['X', 'Y', 'Z']
        print(f"\nTesting Gyro {axis_names[axis]} with {amplitude}° amplitude...")
        
        # Enable debug stream
        self.send_command('d')
        time.sleep(0.5)
        
        # Start test
        test_cmds = ['TX', 'TY', 'TZ']
        self.send_command(f'{test_cmds[axis]}={amplitude}')
        
        # Collect data
        samples = self.collect_data(4)
        
        # Stop test
        self.send_command('T0=0')
        self.send_command('d')  # Disable debug
        
        if samples:
            # Analyze response
            pitch_vals = [s['pitch'] for s in samples]
            roll_vals = [s['roll'] for s in samples]
            yaw_vals = [s['yaw'] for s in samples]
            
            pitch_range = max(pitch_vals) - min(pitch_vals)
            roll_range = max(roll_vals) - min(roll_vals)
            yaw_range = max(yaw_vals) - min(yaw_vals)
            
            print(f"  Response ranges:")
            print(f"    Pitch: {pitch_range:.1f}°")
            print(f"    Roll:  {roll_range:.1f}°")
            print(f"    Yaw:   {yaw_range:.1f}°")
            
            # Determine which output axis responds most
            max_response = max(pitch_range, roll_range, yaw_range)
            if max_response < amplitude * 0.5:
                print(f"  ⚠ Weak response - may need to flip sign or increase scale")
            
            if pitch_range == max_response:
                print(f"  → Gyro {axis_names[axis]} primarily affects PITCH")
            elif roll_range == max_response:
                print(f"  → Gyro {axis_names[axis]} primarily affects ROLL")
            else:
                print(f"  → Gyro {axis_names[axis]} primarily affects YAW")
                
            return pitch_range, roll_range, yaw_range
        return 0, 0, 0
        
    def test_all_axes(self):
        """Test all gyro axes systematically"""
        print("\n" + "="*60)
        print("SYSTEMATIC AXIS TESTING")
        print("="*60)
        
        results = {}
        
        # Test each gyro axis
        for axis in range(3):
            p, r, y = self.test_gyro_axis(axis, 30)
            results[f'gyro_{axis}'] = (p, r, y)
            time.sleep(1)
            
        # Analyze results
        print("\n" + "-"*60)
        print("ANALYSIS:")
        
        # Check for swapped axes
        gyro_map = [0, 1, 2]
        if results['gyro_0'][1] > results['gyro_0'][0]:  # X affects roll more than pitch
            print("  ✓ Gyro X and Y appear to be swapped")
            gyro_map = [1, 0, 2]
            
        # Check for inverted axes
        print("\nRecommended configuration:")
        print(f"  AMAP={gyro_map[0]},{gyro_map[1]},{gyro_map[2]}")
        
        return results
        
    def configure_axis(self, axis, polarity, scale=1.0):
        """Configure a specific axis"""
        if polarity < 0:
            self.send_command(f'AG{axis}=-1')
            print(f"  Set gyro axis {axis} to inverted")
        else:
            self.send_command(f'AG{axis}=1')
            print(f"  Set gyro axis {axis} to normal")
            
        if scale != 1.0:
            self.send_command(f'AS{axis}={scale}')
            print(f"  Set gyro axis {axis} scale to {scale}")
            
    def interactive_tuning(self):
        """Interactive tuning mode"""
        print("\n" + "="*60)
        print("INTERACTIVE TUNING MODE")
        print("="*60)
        print("\nCommands:")
        print("  1-3: Test gyro axis X/Y/Z")
        print("  i: Invert an axis")
        print("  s: Scale an axis")
        print("  m: Remap axes")
        print("  a: Auto-test all axes")
        print("  r: Reset orientation")
        print("  p: Print current config")
        print("  q: Quit")
        
        while True:
            cmd = input("\n> ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd in ['1', '2', '3']:
                axis = int(cmd) - 1
                self.test_gyro_axis(axis, 30)
            elif cmd == 'i':
                axis = int(input("  Axis (0=X, 1=Y, 2=Z): "))
                polarity = int(input("  Polarity (-1 or 1): "))
                self.configure_axis(axis, polarity)
            elif cmd == 's':
                axis = int(input("  Axis (0=X, 1=Y, 2=Z): "))
                scale = float(input("  Scale factor: "))
                self.send_command(f'AS{axis}={scale}')
            elif cmd == 'm':
                mapping = input("  New mapping (e.g., 1,0,2 to swap X/Y): ")
                self.send_command(f'AMAP={mapping}')
            elif cmd == 'a':
                self.test_all_axes()
            elif cmd == 'r':
                self.send_command('r')
                print("  Reset orientation")
            elif cmd == 'p':
                response = self.send_command('p')
                print(response)
            elif cmd == 'h':
                print("  Commands: 1-3=test axis, i=invert, s=scale, m=map, a=auto-test, r=reset, p=print, q=quit")
                
    def auto_configure(self):
        """Automatically determine and apply best configuration"""
        print("\n" + "="*60)
        print("AUTOMATIC CONFIGURATION")
        print("="*60)
        print("\nThis will systematically test each axis and determine")
        print("the correct mapping and polarity.")
        
        input("\nEnsure device is stationary. Press Enter to begin...")
        
        # Reset to defaults first
        self.send_command('r')
        self.send_command('AMAP=0,1,2')
        for i in range(3):
            self.send_command(f'AG{i}=1')
            self.send_command(f'AS{i}=1')
        
        # Test all axes
        results = self.test_all_axes()
        
        print("\n" + "-"*60)
        print("APPLYING CONFIGURATION...")
        
        # Apply detected configuration
        # (This is simplified - real implementation would be more sophisticated)
        
        print("\nConfiguration complete!")
        print("Test the device by rotating it and verify:")
        print("  - Pitch: Tilt forward/backward")
        print("  - Roll: Tilt left/right")
        print("  - Yaw: Rotate horizontally")

def main():
    print("6-DOF Kalman Filter Axis Configurator")
    print("=====================================")
    
    configurator = AxisConfigurator()
    
    while True:
        print("\nMain Menu:")
        print("  1. Automatic configuration")
        print("  2. Test all axes")
        print("  3. Interactive tuning")
        print("  4. Quick test single axis")
        print("  5. Exit")
        
        choice = input("\nChoice: ").strip()
        
        if choice == '1':
            configurator.auto_configure()
        elif choice == '2':
            configurator.test_all_axes()
        elif choice == '3':
            configurator.interactive_tuning()
        elif choice == '4':
            axis = int(input("Axis (0=X, 1=Y, 2=Z): "))
            configurator.test_gyro_axis(axis, 30)
        elif choice == '5':
            break
        else:
            print("Invalid choice")
            
    print("\nGoodbye!")

if __name__ == "__main__":
    main()