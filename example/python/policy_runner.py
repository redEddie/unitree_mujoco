#!/usr/bin/env python3
"""
IsaacLab Policy Runner with ROS2 Diagnostics (Restructured)
===========================================================

This script executes a trained IsaacLab policy on the Unitree Go2 robot using MuJoCo simulation
and publishes detailed observation data to ROS2 topics for analysis with PlotJuggler.

Key Features:
- ONNX-based policy execution
- ROS2 publishers for all observation components
- Real-time diagnostics data streaming
- Proper observation scaling matching training conditions
- Joint index reordering between Unitree SDK/MuJoCo and IsaacLab
- Modular code structure with separated utilities and processors

Joint Index Conversion:
- Unitree/MuJoCo order: FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf,
                        RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf
- IsaacLab order: FL_hip, FR_hip, RL_hip, RR_hip,
                  FL_thigh, FR_thigh, RL_thigh, RR_thigh,
                  FL_calf, FR_calf, RL_calf, RR_calf

Author: Generated for IsaacLab policy integration with ROS2 diagnostics


기본 실행 (plotter 없이):

cd /home/user/unitree_mujoco/example/python
python3 policy_runner.py

IMU Plotter와 함께 실행:

python3 policy_runner.py --enable-plot

추가 옵션:

# 실행 시간 변경 (60초)
python3 policy_runner.py --enable-plot --duration 60.0

# Observation noise 비활성화
python3 policy_runner.py --disable-noise

# 도움말 보기
python3 policy_runner.py --help

"""

import time
import sys
import argparse
import numpy as np
import onnxruntime as ort
from typing import Optional, Tuple

import rclpy

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, SportModeState_
from unitree_sdk2py.utils.crc import CRC

from utils.config import PolicyConfig, UNITREE_TO_ISAACLAB, ISAACLAB_TO_UNITREE
from utils.ros2_publisher import Go2DiagnosticsPublisher
from processor.observation_processor import ObservationProcessor


class IsaacLabPolicyRunnerROS2:
    """Policy runner with ROS2 diagnostics publishing and joint reordering"""

    def __init__(self, policy_path: str, config: PolicyConfig, ros_node: Go2DiagnosticsPublisher,
                 network_interface: str = "lo", enable_imu_publish: bool = False):
        self.config = config
        self.crc = CRC()
        self.ros_node = ros_node
        self.network_interface = network_interface

        # Load ONNX policy
        print(f"Loading ONNX policy from: {policy_path}")
        self.ort_session = ort.InferenceSession(policy_path)

        self.input_name = self.ort_session.get_inputs()[0].name
        self.output_name = self.ort_session.get_outputs()[0].name

        print(f"Policy input: {self.input_name}")
        print(f"Policy output: {self.output_name}")

        # Initialize observation processor with optional ROS2 IMU publishing
        self.obs_processor = ObservationProcessor(
            config, ros_node=ros_node if enable_imu_publish else None)

        # Communication setup
        self.latest_lowstate: Optional[LowState_] = None
        self.latest_sportmode: Optional[SportModeState_] = None
        self._setup_communication()

    def _setup_communication(self):
        """Setup Unitree SDK communication"""
        # Use the network interface passed during initialization
        # For simulation: use "lo" (localhost)
        # For real robot: use the actual network interface (e.g., "eth0", "enp2s0")
        if self.network_interface == "lo":
            ChannelFactoryInitialize(1, "lo")
        else:
            ChannelFactoryInitialize(0, self.network_interface)

        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self._lowstate_handler, 10)

        # Add SportModeState subscriber for velocity data
        self.sportmode_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.sportmode_sub.Init(self._sportmode_handler, 10)

        self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_pub.Init()

        print(f"Communication channels initialized (interface: {self.network_interface})")

    def _lowstate_handler(self, msg: LowState_):
        """Handle incoming robot state"""
        self.latest_lowstate = msg

    def _sportmode_handler(self, msg: SportModeState_):
        """Handle incoming SportModeState"""
        self.latest_sportmode = msg
        # Update observation processor with velocity data
        velocity = [float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])]
        self.obs_processor.update_base_velocity(velocity)

    def wait_for_robot_state(self, timeout: float = 5.0):
        """Wait for first robot state"""
        print("Waiting for robot state...")
        start_time = time.time()

        while self.latest_lowstate is None:
            if time.time() - start_time > timeout:
                raise TimeoutError("Failed to receive robot state")
            time.sleep(0.001)

        print("Robot state received!")

    def execute_policy(self, obs: np.ndarray) -> Tuple[np.ndarray, float]:
        """Execute policy and return actions + inference time

        Returns:
            actions_raw: Policy output in IsaacLab order
            inference_time: Inference time in milliseconds
        """
        inference_start = time.perf_counter()

        obs_batch = obs.reshape(1, -1)
        outputs = self.ort_session.run([self.output_name], {self.input_name: obs_batch})
        actions_raw = outputs[0][0]  # IsaacLab 순서의 action

        inference_time = (time.perf_counter() - inference_start) * 1000  # ms

        return actions_raw.astype(np.float32), inference_time

    def _compute_torques(self, target_positions_unitree: np.ndarray,
                        current_positions_unitree: np.ndarray,
                        current_velocities_unitree: np.ndarray) -> np.ndarray:
        """Compute torques using PD controller (matching IsaacLab pipeline)

        Args:
            target_positions_unitree: Target joint positions in Unitree order
            current_positions_unitree: Current joint positions in Unitree order
            current_velocities_unitree: Current joint velocities in Unitree order

        Returns:
            computed_torques: Computed torques in Unitree order
        """
        # Position and velocity errors
        position_error = target_positions_unitree - current_positions_unitree
        velocity_error = np.zeros_like(current_velocities_unitree)  # Target velocity = 0
        velocity_error = velocity_error - current_velocities_unitree

        # PD controller (matching IsaacLab DCMotor parameters)
        kp = self.config.KP  # 25.0
        kd = self.config.KD  # 0.5

        computed_torques = kp * position_error + kd * velocity_error

        # Apply torque limits (matching IsaacLab DCMotor clipping)
        torque_limit = self.config.ACTION_CLIP  # 23.5 N⋅m
        computed_torques = np.clip(computed_torques, -torque_limit, torque_limit)

        return computed_torques

    def create_torque_command(self, target_positions_unitree: np.ndarray,
                             current_positions_unitree: np.ndarray,
                             current_velocities_unitree: np.ndarray) -> unitree_go_msg_dds__LowCmd_:
        """Create torque-based motor command (matching IsaacLab behavior)

        Args:
            target_positions_unitree: Target positions in Unitree/MuJoCo order
            current_positions_unitree: Current positions in Unitree/MuJoCo order
            current_velocities_unitree: Current velocities in Unitree/MuJoCo order
        """
        # Compute torques using PD controller
        torques = self._compute_torques(target_positions_unitree,
                                       current_positions_unitree,
                                       current_velocities_unitree)

        cmd = unitree_go_msg_dds__LowCmd_()

        cmd.head[0] = 0xFE
        cmd.head[1] = 0xEF
        cmd.level_flag = 0xFF
        cmd.gpio = 0

        # Set torque commands for leg joints (0-11)
        for i in range(12):
            cmd.motor_cmd[i].mode = 0x01  # Position mode but we'll use torque
            cmd.motor_cmd[i].q = 0.0      # Position target (not used in torque mode)
            cmd.motor_cmd[i].kp = 0.0     # Zero position gain (pure torque control)
            cmd.motor_cmd[i].kd = 0.0     # Zero damping gain (pure torque control)
            cmd.motor_cmd[i].dq = 0.0     # Target velocity
            cmd.motor_cmd[i].tau = float(torques[i])  # Computed torque command

        # Set other joints to zero
        for i in range(12, 20):
            cmd.motor_cmd[i].mode = 0x01
            cmd.motor_cmd[i].q = 0.0
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].kd = 0.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0

        cmd.crc = self.crc.Crc(cmd)

        return cmd

    def _create_zero_command(self) -> unitree_go_msg_dds__LowCmd_:
        """Create zero torque command to safely stop the robot"""
        cmd = unitree_go_msg_dds__LowCmd_()

        cmd.head[0] = 0xFE
        cmd.head[1] = 0xEF
        cmd.level_flag = 0xFF
        cmd.gpio = 0

        # Set all motors to zero torque mode
        for i in range(20):
            cmd.motor_cmd[i].mode = 0x01  # Keep position mode but with zero gains
            cmd.motor_cmd[i].q = 0.0      # Position doesn't matter with zero gains
            cmd.motor_cmd[i].kp = 0.0     # Zero position gain = no position control
            cmd.motor_cmd[i].kd = 0.0     # Zero damping gain = no velocity control
            cmd.motor_cmd[i].dq = 0.0     # Zero target velocity
            cmd.motor_cmd[i].tau = 0.0    # Zero torque command

        cmd.crc = self.crc.Crc(cmd)

        return cmd

    def run(self, duration: Optional[float] = None, enable_noise: bool = True):
        """Main execution loop with ROS2 publishing and joint reordering"""
        print("Starting policy execution with ROS2 diagnostics...")
        print(f"Control frequency: {1.0/self.config.CONTROL_DT:.1f} Hz")
        print(f"Observation noise: {'Enabled' if enable_noise else 'Disabled'}")
        print("Joint index reordering: Enabled (Unitree/MuJoCo <-> IsaacLab)")

        self.wait_for_robot_state()

        # Set initial velocity commands (all zeros initially)
        # For heading mode: (vx=0.0 m/s, vy=0.0, target_heading=0.0 rad)
        # For relative mode: (vx=0.0 m/s, vy=0.0, wz=0.0 rad/s)
        self.obs_processor.set_velocity_commands(0.0, 0.0, 0.0)

        start_time = time.time()
        step_count = 0

        # Debug: timing statistics
        obs_time_total = 0.0
        policy_time_total = 0.0
        command_time_total = 0.0

        try:
            while rclpy.ok():
                loop_start = time.perf_counter()

                # Check duration
                if duration and (time.time() - start_time) > duration:
                    break

                # Get robot state
                if self.latest_lowstate is None:
                    time.sleep(0.001)
                    continue

                # Process observations (MuJoCo -> IsaacLab 순서 변환)
                obs_start = time.perf_counter()
                obs, obs_components = self.obs_processor.process(
                    self.latest_lowstate, add_noise=enable_noise)
                obs_time = (time.perf_counter() - obs_start) * 1000
                obs_time_total += obs_time

                # Execute policy (IsaacLab 순서 입력 -> IsaacLab 순서 출력)
                policy_start = time.perf_counter()
                actions_raw_isaaclab, inference_time = self.execute_policy(obs)
                policy_time = (time.perf_counter() - policy_start) * 1000
                policy_time_total += policy_time

                # Scale actions (IsaacLab 순서)
                actions_scaled_isaaclab = actions_raw_isaaclab * self.config.ACTION_SCALE
                target_positions_isaaclab = actions_scaled_isaaclab + self.config.DEFAULT_JOINT_POS

                # ===== CRITICAL: Convert actions to Unitree order for robot control =====
                actions_raw_unitree = actions_raw_isaaclab[ISAACLAB_TO_UNITREE]
                actions_scaled_unitree = actions_scaled_isaaclab[ISAACLAB_TO_UNITREE]
                target_positions_unitree = target_positions_isaaclab[ISAACLAB_TO_UNITREE]

                # Update last actions (IsaacLab 순서로 저장)
                self.obs_processor.update_last_actions(actions_raw_isaaclab)

                # Get heading diagnostics for ROS2 publishing
                current_heading = 0.0
                target_heading = 0.0
                heading_error = 0.0

                if self.obs_processor.heading_command:
                    quat = np.array(self.latest_lowstate.imu_state.quaternion, dtype=np.float32)
                    current_heading = self.obs_processor._compute_heading_from_quat(quat)
                    target_heading = self.obs_processor.heading_target
                    heading_error = self.obs_processor._wrap_to_pi(target_heading - current_heading)

                # Get current joint states (Unitree 순서)
                current_positions_unitree = np.array([ms.q for ms in self.latest_lowstate.motor_state[:12]], dtype=np.float32)
                current_velocities_unitree = np.array([ms.dq for ms in self.latest_lowstate.motor_state[:12]], dtype=np.float32)

                # Compute torques for diagnostics (Unitree 순서)
                computed_torques_unitree = self._compute_torques(target_positions_unitree,
                                                               current_positions_unitree,
                                                               current_velocities_unitree)

                # Convert torques to IsaacLab order for ROS2 publishing
                computed_torques_isaaclab = computed_torques_unitree[UNITREE_TO_ISAACLAB]

                # Publish ROS2 diagnostics (obs_components는 IsaacLab 순서)
                current_time = time.time() - start_time
                self.ros_node.publish_observations(obs_components, current_time)
                # actions는 IsaacLab 순서로 publish (obs의 previous_actions와 일치)
                self.ros_node.publish_actions(actions_raw_isaaclab, actions_scaled_isaaclab,
                                              target_positions_isaaclab, computed_torques_isaaclab)

                # Spin ROS2 node to process callbacks and publish messages
                rclpy.spin_once(self.ros_node, timeout_sec=0.0)

                # Send torque-based motor command (Unitree 순서)
                cmd_start = time.perf_counter()
                cmd = self.create_torque_command(target_positions_unitree,
                                               current_positions_unitree,
                                               current_velocities_unitree)
                self.cmd_pub.Write(cmd)
                cmd_time = (time.perf_counter() - cmd_start) * 1000
                command_time_total += cmd_time

                # Calculate loop time
                loop_time = (time.perf_counter() - loop_start) * 1000  # ms
                self.ros_node.publish_diagnostics(inference_time, loop_time,
                                                 current_heading, target_heading, heading_error)

                # Additional spin to ensure diagnostics are published
                rclpy.spin_once(self.ros_node, timeout_sec=0.0)

                # Timing control
                elapsed = time.perf_counter() - loop_start
                sleep_time = self.config.CONTROL_DT - elapsed

                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < -0.001:
                    self.ros_node.get_logger().warning(
                        f'Loop running {-sleep_time*1000:.1f}ms behind')

                step_count += 1

                # Status update with timing stats
                if step_count % (int(2.0 / self.config.CONTROL_DT)) == 0:
                    avg_obs = obs_time_total / step_count
                    avg_policy = policy_time_total / step_count
                    avg_cmd = command_time_total / step_count
                    self.ros_node.get_logger().info(
                        f'Step {step_count}, Runtime: {current_time:.1f}s, '
                        f'Avg times - Obs: {avg_obs:.2f}ms, Policy: {avg_policy:.2f}ms, Cmd: {avg_cmd:.2f}ms')

        except KeyboardInterrupt:
            print("\nStopping...")

        finally:
            # Stop robot with zero torque command
            zero_cmd = self._create_zero_command()
            for _ in range(10):
                self.cmd_pub.Write(zero_cmd)
                time.sleep(0.01)

            print(f"Completed. Total steps: {step_count}")


# =============================================================================
# Main Entry Point
# =============================================================================

def main(args=None):
    """Main function"""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='IsaacLab Policy Runner with ROS2 Diagnostics')
    parser.add_argument('--enable-plot', action='store_true',
                       help='Enable real-time IMU vs Ground Truth plotting')
    parser.add_argument('--duration', type=float, default=30.0,
                       help='Duration to run policy in seconds (default: 30.0)')
    parser.add_argument('--disable-noise', action='store_true',
                       help='Disable observation noise')
    parser.add_argument('--interface', type=str, default='lo',
                       help='Network interface for Unitree SDK (default: lo for simulation)')
    cmd_args = parser.parse_args()

    print("=" * 60)
    print("IsaacLab Policy Runner with ROS2 Diagnostics")
    print("(Restructured with Modular Architecture)")
    print("=" * 60)

    # Initialize ROS2
    rclpy.init(args=args)
    ros_node = Go2DiagnosticsPublisher()

    # Configuration
    config = PolicyConfig()
    policy_path = "/home/user/unitree_mujoco/example/python/exported/policy.onnx"

    # Check if IMU plotting is requested
    enable_imu_publish = cmd_args.enable_plot
    if enable_imu_publish:
        print("\n✓ IMU data will be published to ROS2 topics:")
        print("    - /go2/imu/accelerometer (raw sensor)")
        print("    - /go2/imu/base_velocity (raw velocity)")
        print("  Run the plotter separately: python3 imu_plotter_ros2.py")
        print("  (GT acceleration computed in plotter via differentiation)")

    print("\nWARNING: Ensure robot area is clear!")
    print("ROS2 topics available for PlotJuggler:")
    print("  Policy Observations:")
    print("    - /go2/obs/base_lin_vel (from SportModeState)")
    print("    - /go2/obs/base_ang_vel")
    print("    - /go2/obs/projected_gravity")
    print("    - /go2/obs/velocity_commands")
    print("    - /go2/obs/joint_states (IsaacLab order)")
    print("  Policy Actions:")
    print("    - /go2/actions/raw (IsaacLab order)")
    print("    - /go2/actions/scaled (IsaacLab order)")
    print("    - /go2/actions/target_positions (IsaacLab order)")
    print("    - /go2/actions/computed_torques (IsaacLab order)")
    print("  Diagnostics:")
    print("    - /go2/diagnostics/inference_time_ms")
    print("    - /go2/diagnostics/loop_time_ms")
    print("\nUse: ros2 run plotjuggler plotjuggler")
    print("Note: Run ros2_bridge_sportmodestate.py separately if you need ROS2 bridge topics")
    if enable_imu_publish:
        print("\nIMU Publishing: ENABLED (10Hz decimated)")
        print("  -> Start plotter: python3 imu_plotter_ros2.py")
    input("\nPress Enter to start...")

    runner = None
    try:
        runner = IsaacLabPolicyRunnerROS2(
            policy_path, config, ros_node,
            network_interface=cmd_args.interface,
            enable_imu_publish=enable_imu_publish)
        runner.run(duration=cmd_args.duration, enable_noise=not cmd_args.disable_noise)

    except Exception as e:
        ros_node.get_logger().error(f'Error: {e}')
        return 1

    finally:
        # Clean up DDS resources
        if runner:
            try:
                # Clean up DDS subscribers
                if hasattr(runner, 'state_sub') and runner.state_sub:
                    del runner.state_sub
                    runner.state_sub = None
                if hasattr(runner, 'sportmode_sub') and runner.sportmode_sub:
                    del runner.sportmode_sub
                    runner.sportmode_sub = None
                # Clean up DDS publisher
                if hasattr(runner, 'cmd_pub') and runner.cmd_pub:
                    del runner.cmd_pub
                    runner.cmd_pub = None
                print("✓ DDS channels cleaned up")
            except Exception as e:
                print(f"⚠️  Warning: DDS cleanup error: {e}")

        # Clean up ROS2 node and publishers
        try:
            # Destroy all publishers
            publishers = [
                'pub_base_lin_vel', 'pub_base_ang_vel', 'pub_projected_gravity',
                'pub_velocity_commands', 'pub_joint_states', 'pub_actions',
                'pub_actions_scaled', 'pub_target_positions', 'pub_computed_torques',
                'pub_full_obs', 'pub_inference_time', 'pub_loop_time',
                'pub_current_heading', 'pub_target_heading', 'pub_heading_error'
            ]
            for pub_name in publishers:
                if hasattr(ros_node, pub_name):
                    pub = getattr(ros_node, pub_name)
                    if pub:
                        ros_node.destroy_publisher(pub)
            print("✓ ROS2 publishers destroyed")
        except Exception as e:
            print(f"⚠️  Warning: Publisher cleanup error: {e}")

        # Destroy ROS2 node
        try:
            ros_node.destroy_node()
            print("✓ ROS2 node destroyed")
        except Exception as e:
            print(f"⚠️  Warning: Node destruction error: {e}")

        # Shutdown ROS2
        try:
            rclpy.shutdown()
            print("✓ ROS2 shutdown complete")
        except Exception as e:
            print(f"⚠️  Warning: ROS2 shutdown error: {e}")

    return 0


if __name__ == "__main__":
    exit(main())
