#!/usr/bin/env python3
"""
Advanced Axis Testing and Configuration Tool
Performs comprehensive testing in a single connection session
"""
import socket
import time
import statistics
import math
import json
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
import numpy as np

@dataclass
class Sample:
    """Data sample from device"""
    timestamp: int
    raw_gx: float
    raw_gy: float
    raw_gz: float
    ax: float
    ay: float
    az: float
    pitch: float
    roll: float
    yaw: float
    
@dataclass
class AxisTestResult:
    """Result from testing a single axis"""
    axis_name: str
    test_amplitude: float
    pitch_response: float
    roll_response: float
    yaw_response: float
    primary_output: str
    response_quality: str
    phase_delay: float
    noise_level: float
    recommendations: List[str]

class AdvancedAxisTester:
    def __init__(self, host='192.168.64.1', port=9999):
        self.host = host
        self.port = port
        self.sock = None
        self.buffer = ""
        self.all_samples = []
        
    def connect(self):
        """Establish connection"""
        print(f"Connecting to {self.host}:{self.port}...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))
        self.sock.settimeout(0.1)
        print("Connected successfully!")
        
    def disconnect(self):
        """Close connection"""
        if self.sock:
            self.sock.close()
            self.sock = None
            print("Disconnected")
            
    def send_command(self, cmd: str, delay: float = 0.1):
        """Send command to device"""
        if self.sock:
            self.sock.send(f"{cmd}\n".encode())
            time.sleep(delay)
            
    def collect_samples(self, duration: float) -> List[Sample]:
        """Collect samples for specified duration"""
        samples = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            try:
                data = self.sock.recv(4096).decode('utf-8', errors='ignore')
                self.buffer += data
                
                # Process complete lines
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    
                    if 'CSV' in line:
                        parts = line.split(',')
                        if len(parts) >= 11:
                            try:
                                sample = Sample(
                                    timestamp=int(parts[1]),
                                    raw_gx=float(parts[2]),
                                    raw_gy=float(parts[3]),
                                    raw_gz=float(parts[4]),
                                    ax=float(parts[5]),
                                    ay=float(parts[6]),
                                    az=float(parts[7]),
                                    pitch=float(parts[8]),
                                    roll=float(parts[9]),
                                    yaw=float(parts[10])
                                )
                                samples.append(sample)
                                self.all_samples.append(sample)
                            except (ValueError, IndexError):
                                pass
                                
            except socket.timeout:
                continue
                
        return samples
    
    def analyze_noise(self, samples: List[Sample]) -> Dict[str, float]:
        """Analyze noise characteristics"""
        if len(samples) < 10:
            return {'pitch': 0, 'roll': 0, 'yaw': 0, 'overall': 0}
            
        pitch_std = statistics.stdev([s.pitch for s in samples])
        roll_std = statistics.stdev([s.roll for s in samples])
        yaw_std = statistics.stdev([s.yaw for s in samples])
        
        return {
            'pitch': pitch_std,
            'roll': roll_std,
            'yaw': yaw_std,
            'overall': math.sqrt(pitch_std**2 + roll_std**2 + yaw_std**2)
        }
    
    def analyze_drift(self, samples: List[Sample]) -> Dict[str, float]:
        """Analyze drift rates"""
        if len(samples) < 10:
            return {'pitch': 0, 'roll': 0, 'yaw': 0}
            
        # Linear regression for drift
        times = [(s.timestamp - samples[0].timestamp) / 1000000.0 for s in samples]
        
        # Calculate drift rate (degrees per second)
        pitch_vals = [s.pitch for s in samples]
        roll_vals = [s.roll for s in samples]
        yaw_vals = [s.yaw for s in samples]
        
        if times[-1] > 0:
            pitch_drift = (pitch_vals[-1] - pitch_vals[0]) / times[-1]
            roll_drift = (roll_vals[-1] - roll_vals[0]) / times[-1]
            yaw_drift = (yaw_vals[-1] - yaw_vals[0]) / times[-1]
        else:
            pitch_drift = roll_drift = yaw_drift = 0
            
        return {
            'pitch': pitch_drift,
            'roll': roll_drift,
            'yaw': yaw_drift
        }
    
    def test_gyro_axis(self, axis: int, amplitude: float = 30) -> AxisTestResult:
        """Test a single gyro axis comprehensively"""
        axis_names = ['X', 'Y', 'Z']
        axis_name = axis_names[axis]
        test_cmds = ['TX', 'TY', 'TZ']
        
        print(f"\nTesting Gyro {axis_name} with {amplitude}° amplitude...")
        
        # Start test signal
        self.send_command(f'{test_cmds[axis]}={amplitude}')
        
        # Collect test samples
        test_samples = self.collect_samples(4.0)
        
        # Stop test signal
        self.send_command('T0=0')
        time.sleep(0.5)
        
        if len(test_samples) < 10:
            return AxisTestResult(
                axis_name=axis_name,
                test_amplitude=amplitude,
                pitch_response=0, roll_response=0, yaw_response=0,
                primary_output="ERROR",
                response_quality="NO_DATA",
                phase_delay=0, noise_level=0,
                recommendations=["No data received - check connection"]
            )
        
        # Analyze response amplitude
        pitch_vals = [s.pitch for s in test_samples]
        roll_vals = [s.roll for s in test_samples]
        yaw_vals = [s.yaw for s in test_samples]
        
        pitch_range = max(pitch_vals) - min(pitch_vals)
        roll_range = max(roll_vals) - min(roll_vals)
        yaw_range = max(yaw_vals) - min(yaw_vals)
        
        # Determine primary output axis
        max_response = max(pitch_range, roll_range, yaw_range)
        if pitch_range == max_response:
            primary = "PITCH"
        elif roll_range == max_response:
            primary = "ROLL"
        else:
            primary = "YAW"
        
        # Assess response quality
        expected_response = amplitude * 1.8  # Expected peak-to-peak
        quality_ratio = max_response / expected_response
        
        if quality_ratio < 0.3:
            quality = "WEAK"
        elif quality_ratio < 0.7:
            quality = "MODERATE"
        elif quality_ratio < 1.3:
            quality = "GOOD"
        else:
            quality = "EXCESSIVE"
        
        # Calculate phase delay (simplified)
        phase_delay = 0.0
        if len(test_samples) > 20:
            # Find first peak
            for i in range(10, len(test_samples)-10):
                if primary == "PITCH" and pitch_vals[i] > pitch_vals[i-1] and pitch_vals[i] > pitch_vals[i+1]:
                    phase_delay = (test_samples[i].timestamp - test_samples[0].timestamp) / 1000000.0
                    break
        
        # Calculate noise during test
        noise = self.analyze_noise(test_samples)
        
        # Generate recommendations
        recommendations = []
        
        if quality == "WEAK":
            recommendations.append(f"Increase gain: AS{axis}=2.0")
            recommendations.append(f"Or try inverting: AG{axis}=-1")
        elif quality == "EXCESSIVE":
            recommendations.append(f"Reduce gain: AS{axis}=0.5")
        
        expected_axes = {0: "ROLL", 1: "PITCH", 2: "YAW"}
        if primary != expected_axes.get(axis, ""):
            if axis == 0 and primary == "PITCH":
                recommendations.append("X/Y axes swapped - use AMAP=1,0,2")
            elif axis == 1 and primary == "ROLL":
                recommendations.append("X/Y axes swapped - use AMAP=1,0,2")
        
        return AxisTestResult(
            axis_name=axis_name,
            test_amplitude=amplitude,
            pitch_response=pitch_range,
            roll_response=roll_range,
            yaw_response=yaw_range,
            primary_output=primary,
            response_quality=quality,
            phase_delay=phase_delay,
            noise_level=noise['overall'],
            recommendations=recommendations
        )
    
    def run_comprehensive_test(self):
        """Run complete test suite"""
        print("\n" + "="*70)
        print("COMPREHENSIVE AXIS TESTING AND CONFIGURATION")
        print("="*70)
        
        # Connect
        self.connect()
        
        try:
            # Initialize device
            print("\n[Phase 1: Initialization]")
            print("-" * 40)
            self.send_command('r', delay=0.5)  # Reset orientation
            print("✓ Orientation reset")
            
            self.send_command('d', delay=0.5)  # Enable debug stream
            print("✓ Debug stream enabled")
            
            # Baseline measurements
            print("\n[Phase 2: Baseline Measurements]")
            print("-" * 40)
            print("Collecting baseline (keep device stationary)...")
            
            baseline_samples = self.collect_samples(5.0)
            print(f"✓ Collected {len(baseline_samples)} baseline samples")
            
            if baseline_samples:
                noise = self.analyze_noise(baseline_samples)
                drift = self.analyze_drift(baseline_samples)
                
                print(f"\nNoise levels:")
                print(f"  Pitch: {noise['pitch']:.3f}° RMS")
                print(f"  Roll:  {noise['roll']:.3f}° RMS")
                print(f"  Yaw:   {noise['yaw']:.3f}° RMS")
                
                print(f"\nDrift rates:")
                print(f"  Pitch: {drift['pitch']:+.2f}°/sec")
                print(f"  Roll:  {drift['roll']:+.2f}°/sec")
                print(f"  Yaw:   {drift['yaw']:+.2f}°/sec")
            
            # Test each gyro axis
            print("\n[Phase 3: Gyroscope Axis Testing]")
            print("-" * 40)
            
            test_results = []
            for axis in range(3):
                result = self.test_gyro_axis(axis, amplitude=30)
                test_results.append(result)
                
                print(f"\n{result.axis_name} Axis Results:")
                print(f"  Primary output: {result.primary_output}")
                print(f"  Response quality: {result.response_quality}")
                print(f"  Response amplitudes:")
                print(f"    Pitch: {result.pitch_response:.1f}°")
                print(f"    Roll:  {result.roll_response:.1f}°")
                print(f"    Yaw:   {result.yaw_response:.1f}°")
                
                if result.recommendations:
                    print(f"  Recommendations:")
                    for rec in result.recommendations:
                        print(f"    • {rec}")
            
            # Configuration analysis
            print("\n[Phase 4: Configuration Analysis]")
            print("-" * 40)
            
            # Determine optimal mapping
            current_mapping = {}
            for i, result in enumerate(test_results):
                current_mapping[result.axis_name] = result.primary_output
            
            print("\nCurrent axis mapping:")
            for axis, output in current_mapping.items():
                print(f"  Gyro {axis} → {output}")
            
            # Generate optimal configuration
            optimal_config = self.generate_optimal_config(test_results)
            
            print("\n[Phase 5: Recommended Configuration]")
            print("-" * 40)
            
            if optimal_config['mapping'] != [0, 1, 2]:
                print(f"✓ Axis remapping: AMAP={','.join(map(str, optimal_config['mapping']))}")
                self.send_command(f"AMAP={','.join(map(str, optimal_config['mapping']))}")
                time.sleep(0.5)
            
            for i, sign in enumerate(optimal_config['signs']):
                if sign < 0:
                    print(f"✓ Invert axis {i}: AG{i}=-1")
                    self.send_command(f"AG{i}=-1")
                    time.sleep(0.2)
            
            for i, scale in enumerate(optimal_config['scales']):
                if abs(scale - 1.0) > 0.1:
                    print(f"✓ Scale axis {i}: AS{i}={scale:.1f}")
                    self.send_command(f"AS{i}={scale:.1f}")
                    time.sleep(0.2)
            
            # Verify configuration
            print("\n[Phase 6: Verification]")
            print("-" * 40)
            print("Testing new configuration...")
            
            self.send_command('r')  # Reset orientation
            time.sleep(0.5)
            
            verify_samples = self.collect_samples(3.0)
            if verify_samples:
                final_noise = self.analyze_noise(verify_samples)
                final_drift = self.analyze_drift(verify_samples)
                
                print(f"\nFinal noise levels:")
                print(f"  Pitch: {final_noise['pitch']:.3f}° RMS")
                print(f"  Roll:  {final_noise['roll']:.3f}° RMS")
                print(f"  Yaw:   {final_noise['yaw']:.3f}° RMS")
                
                print(f"\nFinal drift rates:")
                print(f"  Pitch: {final_drift['pitch']:+.2f}°/sec")
                print(f"  Roll:  {final_drift['roll']:+.2f}°/sec")
                print(f"  Yaw:   {final_drift['yaw']:+.2f}°/sec")
            
            # Save configuration
            print("\n[Phase 7: Configuration Summary]")
            print("-" * 40)
            
            config_summary = {
                'timestamp': time.strftime("%Y-%m-%d %H:%M:%S"),
                'mapping': optimal_config['mapping'],
                'signs': optimal_config['signs'],
                'scales': optimal_config['scales'],
                'baseline_noise': noise,
                'baseline_drift': drift,
                'test_results': [
                    {
                        'axis': r.axis_name,
                        'primary': r.primary_output,
                        'quality': r.response_quality,
                        'pitch_response': r.pitch_response,
                        'roll_response': r.roll_response,
                        'yaw_response': r.yaw_response
                    } for r in test_results
                ]
            }
            
            # Save to file
            with open('axis_config.json', 'w') as f:
                json.dump(config_summary, f, indent=2)
                print("✓ Configuration saved to axis_config.json")
            
            # Print final commands for manual application
            print("\nFinal configuration commands:")
            print("-" * 30)
            if optimal_config['mapping'] != [0, 1, 2]:
                print(f"AMAP={','.join(map(str, optimal_config['mapping']))}")
            for i, sign in enumerate(optimal_config['signs']):
                if sign < 0:
                    print(f"AG{i}=-1")
            for i, scale in enumerate(optimal_config['scales']):
                if abs(scale - 1.0) > 0.1:
                    print(f"AS{i}={scale:.1f}")
            
            # Disable debug mode
            self.send_command('d')
            print("\n✓ Debug stream disabled")
            
        finally:
            # Always disconnect
            self.disconnect()
            
        print("\n" + "="*70)
        print("TEST COMPLETE")
        print("="*70)
    
    def generate_optimal_config(self, test_results: List[AxisTestResult]) -> Dict:
        """Generate optimal configuration based on test results"""
        
        # Default configuration
        mapping = [0, 1, 2]
        signs = [1, 1, 1]
        scales = [1.0, 1.0, 1.0]
        
        # Check for axis swapping
        if (test_results[0].primary_output == "PITCH" and 
            test_results[1].primary_output == "ROLL"):
            mapping = [1, 0, 2]  # Swap X and Y
        
        # Check for weak responses needing scaling
        for i, result in enumerate(test_results):
            if result.response_quality == "WEAK":
                scales[i] = 2.0
            elif result.response_quality == "EXCESSIVE":
                scales[i] = 0.5
        
        # Check for inverted axes based on expected behavior
        # (This would need more sophisticated testing with actual movement)
        
        return {
            'mapping': mapping,
            'signs': signs,
            'scales': scales
        }

def main():
    tester = AdvancedAxisTester()
    try:
        tester.run_comprehensive_test()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        if tester.sock:
            tester.disconnect()
    except Exception as e:
        print(f"\n\nError: {e}")
        if tester.sock:
            tester.disconnect()

if __name__ == "__main__":
    main()