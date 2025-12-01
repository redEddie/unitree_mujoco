#!/usr/bin/env python3
"""
IMU Frame 검증 스크립트
======================

목적: lowstate.imu_state의 데이터가 어느 frame인지 확인

테스트 방법:
1. 로봇을 수평으로 놓고 IMU 데이터 확인
2. 로봇을 특정 방향으로 기울이고 데이터 변화 확인
3. 변환 행렬 적용 전/후 비교
"""

import sys
import numpy as np
import time

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_


class IMUFrameTest:
    """IMU frame 테스트"""

    def __init__(self):
        self.latest_state = None
        self.sample_count = 0

        # IMU to Base transformation (from user's matrix)
        # T_base_to_radar, but IMU is inside radar
        self.R_base_to_imu = np.array([
            [-0.9659,  0,      0.2588],
            [ 0,       1,      0     ],
            [ 0.2588,  0,     -0.9659]
        ])

        # Translation (not needed for rotation-only data)
        self.t_base_to_imu = np.array([0.28945, 0, -0.046825])

    def setup(self):
        """Setup communication"""
        ChannelFactoryInitialize(1, "lo")
        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self._state_handler, 10)
        print("✓ Subscribed to rt/lowstate")

    def _state_handler(self, msg: LowState_):
        """Handle state"""
        self.latest_state = msg

    def wait_for_data(self):
        """Wait for first data"""
        print("Waiting for IMU data...")
        while self.latest_state is None:
            time.sleep(0.01)
        print("✓ Receiving data\n")

    def analyze_frame(self):
        """Analyze which frame the data is in"""

        print("=" * 70)
        print("IMU Frame Analysis")
        print("=" * 70)

        # Collect samples
        samples = []
        for i in range(50):
            if self.latest_state is not None:
                samples.append({
                    'quat': np.array(self.latest_state.imu_state.quaternion),
                    'acc': np.array(self.latest_state.imu_state.accelerometer),
                    'gyro': np.array(self.latest_state.imu_state.gyroscope)
                })
            time.sleep(0.02)

        # Average
        avg_quat = np.mean([s['quat'] for s in samples], axis=0)
        avg_acc = np.mean([s['acc'] for s in samples], axis=0)
        avg_gyro = np.mean([s['gyro'] for s in samples], axis=0)

        print("\n1. Raw IMU Data (as received from SDK)")
        print("-" * 70)
        print(f"Quaternion [w,x,y,z]: {avg_quat}")
        print(f"Accelerometer [x,y,z]: {avg_acc} m/s²")
        print(f"Gyroscope [x,y,z]:     {avg_gyro} rad/s")

        # Compute gravity direction from quaternion
        w, x, y, z = avg_quat

        # Rotation matrix (world to body)
        R_world_to_body = np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        ])

        # Gravity in world frame
        gravity_world = np.array([0, 0, -9.81])

        # Expected gravity in body frame (if quaternion is body frame)
        gravity_body_expected = R_world_to_body @ gravity_world

        # Specific force (what IMU measures) = -gravity (when stationary)
        specific_force_expected = -gravity_body_expected

        print("\n2. Analysis: Is quaternion in BASE frame or IMU frame?")
        print("-" * 70)
        print(f"Expected specific force (if quat = base frame): {specific_force_expected}")
        print(f"Actual accelerometer reading:                    {avg_acc}")
        print(f"Difference:                                      {avg_acc - specific_force_expected}")
        print(f"Magnitude of difference:                         {np.linalg.norm(avg_acc - specific_force_expected):.4f} m/s²")

        # Test with transformation
        print("\n3. Test with IMU→Base Transformation")
        print("-" * 70)

        # If data is in IMU frame, transform to base
        acc_transformed = self.R_base_to_imu.T @ avg_acc

        print(f"Acc transformed to base frame: {acc_transformed}")
        print(f"Difference from expected:      {acc_transformed - specific_force_expected}")
        print(f"Magnitude of difference:       {np.linalg.norm(acc_transformed - specific_force_expected):.4f} m/s²")

        # Interpretation
        print("\n4. Interpretation")
        print("-" * 70)

        diff_raw = np.linalg.norm(avg_acc - specific_force_expected)
        diff_transformed = np.linalg.norm(acc_transformed - specific_force_expected)

        if diff_raw < 1.0:  # Less than 1 m/s² error
            print("✓ CONCLUSION: Data is already in BASE frame")
            print("  → Unitree SDK provides base frame data")
            print("  → No transformation needed in code")
            print("  → Current code is CORRECT")
        elif diff_transformed < 1.0:
            print("✗ CONCLUSION: Data is in IMU SENSOR frame")
            print("  → Unitree SDK provides raw IMU frame data")
            print("  → Transformation IS needed in code")
            print("  → Current code is INCORRECT - needs fixing!")
        else:
            print("? UNCERTAIN: Neither matches well")
            print("  → Need more investigation")
            print("  → Possible issues:")
            print("    - Robot not level")
            print("    - Transformation matrix incorrect")
            print("    - IMU calibration needed")

        print("\n5. Recommendations")
        print("-" * 70)
        print("Next steps:")
        print("1. Ensure robot is on level ground")
        print("2. Run this test multiple times")
        print("3. Tilt robot and observe changes")
        print("4. Check Unitree SDK documentation")
        print("=" * 70)


def main():
    print("IMU Frame Detection Test")
    print("Make sure the robot is on level ground!\n")

    tester = IMUFrameTest()
    tester.setup()
    tester.wait_for_data()
    tester.analyze_frame()


if __name__ == "__main__":
    main()
