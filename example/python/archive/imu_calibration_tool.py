#!/usr/bin/env python3
"""
IMU Calibration Tool for Unitree Robot
======================================

This tool helps identify the IMU sensor orientation relative to the body frame
and calibrate the transformation matrix.

Usage:
    1. Place the robot in a stable standing position
    2. Run this script to collect IMU data
    3. The script will compute the IMU-to-body transformation matrix

Author: IMU Calibration Helper
"""

import time
import sys
import numpy as np
from typing import Tuple

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

import config
if config.ROBOT == "g1":
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
else:
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_


class IMUCalibration:
    """IMU calibration tool"""

    def __init__(self):
        self.latest_lowstate = None
        self.imu_samples = []

        # Setup communication
        self._setup_communication()

    def _setup_communication(self):
        """Setup Unitree SDK communication"""
        if len(sys.argv) < 2:
            ChannelFactoryInitialize(1, "lo")
            print("Connected to simulation")
        else:
            ChannelFactoryInitialize(0, sys.argv[1])
            print(f"Connected to real robot: {sys.argv[1]}")

        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self._lowstate_handler, 10)

    def _lowstate_handler(self, msg: LowState_):
        """Handle incoming robot state"""
        self.latest_lowstate = msg

    def wait_for_data(self, timeout: float = 5.0):
        """Wait for first IMU data"""
        print("Waiting for IMU data...")
        start_time = time.time()

        while self.latest_lowstate is None:
            if time.time() - start_time > timeout:
                raise TimeoutError("Failed to receive IMU data")
            time.sleep(0.001)

        print("✓ IMU data received!\n")

    def collect_samples(self, duration: float = 3.0, sample_rate: float = 100.0):
        """Collect IMU samples while robot is stationary"""
        print(f"Collecting IMU samples for {duration} seconds...")
        print("Please ensure the robot is stationary and standing upright!\n")

        self.imu_samples = []
        start_time = time.time()
        dt = 1.0 / sample_rate

        while (time.time() - start_time) < duration:
            if self.latest_lowstate:
                # Collect IMU data
                quat = np.array(self.latest_lowstate.imu_state.quaternion, dtype=np.float32)
                gyro = np.array(self.latest_lowstate.imu_state.gyroscope, dtype=np.float32)
                acc = np.array(self.latest_lowstate.imu_state.accelerometer, dtype=np.float32)

                self.imu_samples.append({
                    'quat': quat,
                    'gyro': gyro,
                    'acc': acc,
                    'time': time.time() - start_time
                })

            time.sleep(dt)

        print(f"✓ Collected {len(self.imu_samples)} samples\n")

    def analyze_orientation(self) -> Tuple[np.ndarray, np.ndarray]:
        """Analyze IMU orientation and compute transformation matrix"""

        if len(self.imu_samples) == 0:
            raise ValueError("No samples collected!")

        # Average the samples
        avg_quat = np.mean([s['quat'] for s in self.imu_samples], axis=0)
        avg_gyro = np.mean([s['gyro'] for s in self.imu_samples], axis=0)
        avg_acc = np.mean([s['acc'] for s in self.imu_samples], axis=0)

        # Normalize quaternion
        avg_quat = avg_quat / np.linalg.norm(avg_quat)

        print("=" * 70)
        print("IMU CALIBRATION RESULTS")
        print("=" * 70)
        print("\n1. AVERAGE IMU READINGS (Robot Stationary)")
        print("-" * 70)
        print(f"   Quaternion [w,x,y,z]: [{avg_quat[0]:+.4f}, {avg_quat[1]:+.4f}, {avg_quat[2]:+.4f}, {avg_quat[3]:+.4f}]")
        print(f"   Gyroscope  [x,y,z]:   [{avg_gyro[0]:+.6f}, {avg_gyro[1]:+.6f}, {avg_gyro[2]:+.6f}] rad/s")
        print(f"   Accelerometer [x,y,z]: [{avg_acc[0]:+.4f}, {avg_acc[1]:+.4f}, {avg_acc[2]:+.4f}] m/s²")

        # Analyze gyroscope (should be near zero when stationary)
        gyro_magnitude = np.linalg.norm(avg_gyro)
        print(f"\n   Gyroscope magnitude: {gyro_magnitude:.6f} rad/s")
        if gyro_magnitude > 0.05:
            print("   ⚠️  WARNING: Robot may not be stationary! (gyro > 0.05 rad/s)")
        else:
            print("   ✓ Robot is stationary (gyro < 0.05 rad/s)")

        # Analyze accelerometer (should measure gravity ~9.81 m/s²)
        acc_magnitude = np.linalg.norm(avg_acc)
        print(f"\n   Accelerometer magnitude: {acc_magnitude:.4f} m/s²")
        print(f"   Expected gravity: 9.81 m/s²")
        print(f"   Difference: {abs(acc_magnitude - 9.81):.4f} m/s²")

        # Gravity direction in IMU frame
        gravity_imu = avg_acc / acc_magnitude  # Normalized
        print(f"\n   Gravity direction (IMU frame): [{gravity_imu[0]:+.4f}, {gravity_imu[1]:+.4f}, {gravity_imu[2]:+.4f}]")

        # Expected gravity in body frame (when standing upright)
        # Assuming body frame: X-forward, Y-left, Z-up
        # Gravity should point down: [0, 0, -1] in body frame
        gravity_body_expected = np.array([0.0, 0.0, -1.0])
        print(f"   Gravity expected (Body frame): [{gravity_body_expected[0]:+.4f}, {gravity_body_expected[1]:+.4f}, {gravity_body_expected[2]:+.4f}]")

        # Compute rotation matrix from quaternion (IMU frame to world)
        R_imu_to_world = self._quat_to_rotation_matrix(avg_quat)

        # Gravity in world frame (using quaternion rotation)
        gravity_world = R_imu_to_world.T @ gravity_imu  # R^T transforms from IMU to world
        print(f"\n   Gravity in world frame: [{gravity_world[0]:+.4f}, {gravity_world[1]:+.4f}, {gravity_world[2]:+.4f}]")

        print("\n2. IMU SENSOR ORIENTATION ANALYSIS")
        print("-" * 70)

        # Identify which IMU axis measures gravity most strongly
        max_axis = np.argmax(np.abs(gravity_imu))
        axis_names = ['X', 'Y', 'Z']
        sign = '+' if gravity_imu[max_axis] > 0 else '-'

        print(f"   Primary gravity axis: {sign}{axis_names[max_axis]} ({gravity_imu[max_axis]:+.4f})")

        # Check if IMU is upside down (gravity pointing up in IMU frame)
        if gravity_imu[2] > 0.5:  # Z-axis pointing up significantly
            print("   ⚠️  IMU appears to be UPSIDE DOWN! (Z-axis measures positive gravity)")
            imu_orientation = "upside_down"
        elif gravity_imu[2] < -0.5:  # Z-axis pointing down
            print("   ✓ IMU appears UPRIGHT (Z-axis measures negative gravity)")
            imu_orientation = "upright"
        else:
            print(f"   ⚠️  IMU may be tilted or rotated (Z-gravity: {gravity_imu[2]:+.4f})")
            imu_orientation = "tilted"

        # Compute IMU-to-Body transformation matrix
        print("\n3. IMU-TO-BODY TRANSFORMATION")
        print("-" * 70)

        # Simple approach: compute rotation matrix that aligns gravity vectors
        # R_imu_to_body transforms vectors from IMU frame to body frame
        R_imu_to_body = self._compute_alignment_matrix(gravity_imu, gravity_body_expected)

        print("   IMU-to-Body Rotation Matrix (R_imu_to_body):")
        print(f"   [{R_imu_to_body[0,0]:+.4f}, {R_imu_to_body[0,1]:+.4f}, {R_imu_to_body[0,2]:+.4f}]")
        print(f"   [{R_imu_to_body[1,0]:+.4f}, {R_imu_to_body[1,1]:+.4f}, {R_imu_to_body[1,2]:+.4f}]")
        print(f"   [{R_imu_to_body[2,0]:+.4f}, {R_imu_to_body[2,1]:+.4f}, {R_imu_to_body[2,2]:+.4f}]")

        # Verify the transformation
        gravity_body_corrected = R_imu_to_body @ gravity_imu
        print(f"\n   Corrected gravity (Body frame): [{gravity_body_corrected[0]:+.4f}, {gravity_body_corrected[1]:+.4f}, {gravity_body_corrected[2]:+.4f}]")
        print(f"   Expected:                        [{gravity_body_expected[0]:+.4f}, {gravity_body_expected[1]:+.4f}, {gravity_body_expected[2]:+.4f}]")

        error = np.linalg.norm(gravity_body_corrected - gravity_body_expected)
        print(f"   Alignment error: {error:.6f}")

        if error < 0.05:
            print("   ✓ Good alignment!")
        else:
            print("   ⚠️  Alignment may need manual adjustment")

        print("\n4. PYTHON CODE FOR IMU CORRECTION")
        print("-" * 70)
        print("   Add this to your ObservationProcessor class:\n")
        print("   # IMU-to-Body transformation matrix (from calibration)")
        print("   self.R_imu_to_body = np.array([")
        print(f"       [{R_imu_to_body[0,0]:+.6f}, {R_imu_to_body[0,1]:+.6f}, {R_imu_to_body[0,2]:+.6f}],")
        print(f"       [{R_imu_to_body[1,0]:+.6f}, {R_imu_to_body[1,1]:+.6f}, {R_imu_to_body[1,2]:+.6f}],")
        print(f"       [{R_imu_to_body[2,0]:+.6f}, {R_imu_to_body[2,1]:+.6f}, {R_imu_to_body[2,2]:+.6f}]")
        print("   ], dtype=np.float32)\n")
        print("   # Apply correction:")
        print("   acc_imu = np.array(lowstate.imu_state.accelerometer)")
        print("   acc_body = self.R_imu_to_body @ acc_imu\n")

        print("=" * 70)

        return R_imu_to_body, avg_acc

    def _quat_to_rotation_matrix(self, quat: np.ndarray) -> np.ndarray:
        """Convert quaternion [w,x,y,z] to rotation matrix"""
        w, x, y, z = quat

        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        ], dtype=np.float32)

        return R

    def _compute_alignment_matrix(self, v_imu: np.ndarray, v_body: np.ndarray) -> np.ndarray:
        """Compute rotation matrix that aligns v_imu to v_body

        This uses Rodrigues' rotation formula to find the rotation matrix R such that:
        R @ v_imu ≈ v_body
        """
        # Normalize vectors
        v_imu = v_imu / np.linalg.norm(v_imu)
        v_body = v_body / np.linalg.norm(v_body)

        # Rotation axis: cross product
        axis = np.cross(v_imu, v_body)
        axis_norm = np.linalg.norm(axis)

        # Check if vectors are already aligned or opposite
        if axis_norm < 1e-6:
            # Vectors are parallel or anti-parallel
            dot = np.dot(v_imu, v_body)
            if dot > 0:
                # Already aligned
                return np.eye(3, dtype=np.float32)
            else:
                # 180 degree rotation - find perpendicular axis
                if abs(v_imu[0]) < 0.9:
                    perp = np.array([1, 0, 0])
                else:
                    perp = np.array([0, 1, 0])
                axis = np.cross(v_imu, perp)
                axis = axis / np.linalg.norm(axis)
                return self._rotation_matrix_from_axis_angle(axis, np.pi)

        # Normalize axis
        axis = axis / axis_norm

        # Rotation angle
        angle = np.arccos(np.clip(np.dot(v_imu, v_body), -1.0, 1.0))

        # Rodrigues' rotation formula
        return self._rotation_matrix_from_axis_angle(axis, angle)

    def _rotation_matrix_from_axis_angle(self, axis: np.ndarray, angle: float) -> np.ndarray:
        """Compute rotation matrix from axis-angle representation (Rodrigues' formula)"""
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ], dtype=np.float32)

        R = np.eye(3, dtype=np.float32) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
        return R


def main():
    print("\n" + "=" * 70)
    print("IMU CALIBRATION TOOL")
    print("=" * 70)
    print("\nThis tool will help identify your IMU sensor orientation")
    print("and compute the transformation matrix to body frame.\n")
    print("INSTRUCTIONS:")
    print("1. Place the robot in a stable standing position")
    print("2. Ensure the robot is NOT moving")
    print("3. Press Enter to start calibration")
    print("=" * 70)

    input("\nPress Enter to start...")

    try:
        calibrator = IMUCalibration()
        calibrator.wait_for_data()
        calibrator.collect_samples(duration=3.0)
        R_imu_to_body, avg_acc = calibrator.analyze_orientation()

        print("\n✓ Calibration complete!")
        print("\nNext steps:")
        print("1. Copy the R_imu_to_body matrix from above")
        print("2. Add it to your ObservationProcessor class")
        print("3. Apply the transformation to IMU accelerometer data")

        return 0

    except KeyboardInterrupt:
        print("\n\n🛑 Calibration cancelled")
        return 0

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit(main())
