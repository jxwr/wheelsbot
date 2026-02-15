#!/usr/bin/env python3
"""
Automatic balance controller parameter tuning
Uses closed-loop feedback from serial debug interface
"""

import serial
import time
import math
import sys
from typing import List, Tuple, Dict

# Configuration
PORT = '/dev/cu.usbmodem5B5F1233271'
BAUD = 115200
MAX_ITERATIONS = 15
CONVERGENCE_STD = 0.015
TARGET_CROSSINGS = 15

# Safety limits
PARAM_LIMITS = {
    'angle_kp': (0.5, 3.0),
    'angle_gyro_kd': (0.2, 2.0),
    'velocity_kp': (0.02, 0.3),
    'velocity_ki': (0.0, 0.1),
    'yaw_kd': (0.5, 2.5),
}

class BalanceTuner:
    def __init__(self, port=PORT, baud=BAUD):
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(0.5)
        print(f"✓ Connected to {port}\n")

    def close(self):
        self.ser.close()

    def send_command(self, cmd: str) -> List[str]:
        """Send command and read response lines"""
        self.ser.reset_input_buffer()
        self.ser.write(f'{cmd}\n'.encode())
        self.ser.flush()
        time.sleep(0.2)

        lines = []
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line and not line.startswith('MOT,'):
                lines.append(line)
        return lines

    def get_param(self, name: str) -> float:
        """Get parameter value"""
        lines = self.send_command(f'GET {name}')
        for line in lines:
            if line.startswith('VALUE,'):
                parts = line.split(',')
                if len(parts) == 3:
                    return float(parts[2])
        raise ValueError(f"Failed to get parameter: {name}")

    def set_param(self, name: str, value: float) -> bool:
        """Set parameter value with safety check"""
        if name in PARAM_LIMITS:
            min_val, max_val = PARAM_LIMITS[name]
            if value < min_val or value > max_val:
                print(f"   ⚠ Warning: {name}={value:.4f} exceeds limits [{min_val}, {max_val}]")
                value = max(min_val, min(max_val, value))
                print(f"   → Clamped to {value:.4f}")

        lines = self.send_command(f'SET {name} {value:.4f}')
        return any('OK' in line for line in lines)

    def sample_system(self, n_samples=100, rate=50) -> Tuple[List[float], Dict]:
        """Collect sensor samples and calculate statistics"""
        self.ser.reset_input_buffer()
        self.ser.write(f'STREAM {rate}\n'.encode())
        self.ser.flush()
        time.sleep(0.1)

        pitches = []
        timeout = time.time() + (n_samples / rate) + 2

        while len(pitches) < n_samples and time.time() < timeout:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('DATA,'):
                    try:
                        fields = line.split(',')
                        pitch = float(fields[2])
                        pitches.append(pitch)
                    except (IndexError, ValueError):
                        continue

        self.ser.write(b'STREAM 0\n')
        self.ser.flush()
        time.sleep(0.1)

        if len(pitches) < n_samples * 0.8:
            raise ValueError(f"Only collected {len(pitches)}/{n_samples} samples")

        # Calculate statistics
        mean = sum(pitches) / len(pitches)
        variance = sum((x - mean) ** 2 for x in pitches) / len(pitches)
        std = math.sqrt(variance)
        max_abs = max(abs(x) for x in pitches)

        # Count zero crossings
        crossings = sum(1 for i in range(1, len(pitches))
                       if (pitches[i-1] > mean and pitches[i] < mean) or
                          (pitches[i-1] < mean and pitches[i] > mean))

        stats = {
            'mean': mean,
            'std': std,
            'max_abs': max_abs,
            'crossings': crossings,
            'samples': len(pitches)
        }

        return pitches, stats

    def diagnose(self, stats: Dict) -> str:
        """Diagnose system state"""
        std = stats['std']
        crossings = stats['crossings']

        if crossings > 20:
            return 'OSCILLATING'
        elif std > 0.05:
            return 'UNSTABLE'
        elif std < 0.01 and crossings < 5:
            return 'OVER_DAMPED'
        elif std < CONVERGENCE_STD and crossings < TARGET_CROSSINGS:
            return 'CONVERGED'
        else:
            return 'ACCEPTABLE'

    def adjust_parameters(self, diagnosis: str, params: Dict[str, float]) -> Dict[str, float]:
        """Adjust parameters based on diagnosis"""
        new_params = params.copy()

        if diagnosis == 'OSCILLATING':
            print("   → Reducing angle_kp by 15%, increasing angle_gyro_kd by 10%")
            new_params['angle_kp'] *= 0.85
            new_params['angle_gyro_kd'] *= 1.10

        elif diagnosis == 'UNSTABLE':
            print("   → Increasing angle_gyro_kd by 20%, reducing velocity_kp by 10%")
            new_params['angle_gyro_kd'] *= 1.20
            new_params['velocity_kp'] *= 0.90

        elif diagnosis == 'OVER_DAMPED':
            print("   → Increasing velocity_kp by 15%, reducing angle_gyro_kd by 10%")
            new_params['velocity_kp'] *= 1.15
            new_params['angle_gyro_kd'] *= 0.90

        elif diagnosis == 'ACCEPTABLE':
            print("   → Fine-tuning: increasing velocity_kp by 5%")
            new_params['velocity_kp'] *= 1.05

        return new_params

    def run_tuning(self):
        """Main tuning loop"""
        print("=== Balance Controller Auto-Tune ===\n")

        # Get initial parameters
        print("Reading current parameters...")
        current_params = {
            'angle_kp': self.get_param('angle_kp'),
            'angle_gyro_kd': self.get_param('angle_gyro_kd'),
            'velocity_kp': self.get_param('velocity_kp'),
            'velocity_ki': self.get_param('velocity_ki'),
        }
        print(f"  angle_kp: {current_params['angle_kp']:.4f}")
        print(f"  angle_gyro_kd: {current_params['angle_gyro_kd']:.4f}")
        print(f"  velocity_kp: {current_params['velocity_kp']:.4f}")
        print()

        # Baseline test
        print("Running baseline stability test...")
        _, initial_stats = self.sample_system(100, 50)
        initial_diagnosis = self.diagnose(initial_stats)
        print(f"  Mean: {initial_stats['mean']:.4f}, Std: {initial_stats['std']:.4f}")
        print(f"  Crossings: {initial_stats['crossings']}, Status: {initial_diagnosis}\n")

        if initial_diagnosis == 'CONVERGED':
            print("✓ System already converged!")
            return

        # Tuning iterations
        best_std = initial_stats['std']
        iterations_without_improvement = 0

        for iteration in range(1, MAX_ITERATIONS + 1):
            print(f"=== Iteration {iteration} ===")

            # Sample system
            _, stats = self.sample_system(100, 50)
            diagnosis = self.diagnose(stats)

            print(f"  Pitch: mean={stats['mean']:.4f}, std={stats['std']:.4f}, crossings={stats['crossings']}")
            print(f"  Status: {diagnosis}")

            # Check convergence
            if diagnosis == 'CONVERGED':
                print("\n✓ CONVERGED!")
                break

            # Check for improvement
            if stats['std'] < best_std:
                best_std = stats['std']
                iterations_without_improvement = 0
            else:
                iterations_without_improvement += 1

            if iterations_without_improvement >= 3:
                print("\n⚠ No improvement for 3 iterations, stopping.")
                break

            # Adjust parameters
            new_params = self.adjust_parameters(diagnosis, current_params)

            # Apply changes
            for name, value in new_params.items():
                if abs(value - current_params[name]) > 0.001:
                    self.set_param(name, value)
                    print(f"   {name}: {current_params[name]:.4f} → {value:.4f}")

            current_params = new_params
            print()

            # Wait for system to settle
            time.sleep(1.5)

        # Final verification
        print("\n=== Final Verification ===")
        _, final_stats = self.sample_system(200, 50)
        final_diagnosis = self.diagnose(final_stats)
        print(f"  Mean: {final_stats['mean']:.4f}, Std: {final_stats['std']:.4f}")
        print(f"  Crossings: {final_stats['crossings']}, Status: {final_diagnosis}")

        # Save parameters
        print("\nSaving parameters to flash...")
        self.send_command('SAVE')
        print("✓ Parameters saved")

        # Final report
        print("\n" + "="*50)
        print("=== Tuning Complete ===")
        print(f"Initial state: std={initial_stats['std']:.4f}, crossings={initial_stats['crossings']}")
        print(f"Final state: std={final_stats['std']:.4f}, crossings={final_stats['crossings']}")
        print(f"\nImprovement: {((initial_stats['std'] - final_stats['std']) / initial_stats['std'] * 100):.1f}%")
        print("\nFinal parameters:")
        for name in ['angle_kp', 'angle_gyro_kd', 'velocity_kp', 'velocity_ki']:
            value = self.get_param(name)
            print(f"  {name}: {value:.4f}")
        print("="*50)


def main():
    try:
        tuner = BalanceTuner()
        tuner.run_tuning()
        tuner.close()
    except KeyboardInterrupt:
        print("\n\nTuning interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
