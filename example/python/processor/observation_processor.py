"""
Observation processor for converting robot sensor data to policy observations
"""
import time
import numpy as np
from typing import Tuple, Optional
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

from utils.config import PolicyConfig, UNITREE_TO_ISAACLAB


class ObservationProcessor:
    """Processes raw sensor data into policy observations with joint reordering"""

    def __init__(self, config: PolicyConfig, ros_node=None):
        """
        Initialize observation processor

        Args:
            config: Policy configuration
            ros_node: Optional ROS2 node for publishing IMU data
        """
        self.config = config
        # last_actions는 IsaacLab 순서로 저장됨
        self.last_actions = np.zeros(12, dtype=np.float32)

        # Store latest velocity from SportModeState
        self.latest_base_velocity = np.zeros(3, dtype=np.float32)

        # ROS2 publishing support (optional)
        self.ros_node = ros_node

        # Debug: timing measurement
        self.process_count = 0
        self.imu_publish_time_total = 0.0

        # IMU data decimation: only publish every N iterations
        self.imu_decimation = 2.5  # 50Hz / 2.5 = 20Hz IMU publishing (plenty for plotting)
        self.imu_counter = 0

        # Velocity command configuration (matching IsaacLab pattern)
        self.heading_command = False  # Relative angular velocity mode (continuous rotation)
        self.heading_control_stiffness = 0.5  # IsaacLab default value (not used in relative mode)

        # Input velocity commands [vx, vy, yaw] - what user provides
        self.velocity_commands = np.zeros(3, dtype=np.float32)  # [vx, vy, yaw_target or yaw_rate]

        # Processed velocity commands [vx, vy, wz] - what robot actually uses and goes to policy
        self.vel_command_b = np.zeros(3, dtype=np.float32)  # processed velocity commands

        # Heading control state (for absolute heading mode)
        self.heading_target = 0.0      # target heading (radians)

    def update_base_velocity(self, velocity):
        """Update base velocity from SportModeState"""
        self.latest_base_velocity = np.array(velocity, dtype=np.float32)

    def process(self, lowstate: LowState_, add_noise: bool = True) -> Tuple[np.ndarray, dict]:
        """Process raw sensor data and return observation + components dict

        CRITICAL:
        - MuJoCo의 joint data를 받아서 IsaacLab 순서로 변환
        - Policy에 IsaacLab 순서의 observation 입력
        """

        # Use base velocity from bridge (SportModeState)
        base_lin_vel = self.latest_base_velocity.copy()

        # Get angular velocity from IMU
        base_ang_vel = np.array(lowstate.imu_state.gyroscope, dtype=np.float32)

        # Projected gravity and current orientation
        quat = np.array(lowstate.imu_state.quaternion, dtype=np.float32)
        projected_gravity = self._compute_projected_gravity(quat)

        # IMU data publishing (decimated, NO processing - just publish raw data!)
        # Only publish every N iterations: 50Hz / 2.5 = 20Hz IMU data
        # GT calculation (with gravity + rotational effects) done in plotter (separate process)
        imu_start = time.perf_counter()
        if self.ros_node:
            self.imu_counter += 1
            if self.imu_counter >= self.imu_decimation:
                self.imu_counter = 0

                # Get raw IMU accelerometer
                imu_acc = np.array(lowstate.imu_state.accelerometer, dtype=np.float32)

                # Publish raw data only (no computation in control loop!)
                self.ros_node.publish_imu_data(imu_acc, self.latest_base_velocity,
                                              projected_gravity, base_ang_vel)

        imu_elapsed = (time.perf_counter() - imu_start) * 1000
        self.imu_publish_time_total += imu_elapsed
        self.process_count += 1

        # Update processed velocity commands based on heading mode
        self._update_vel_command_b(quat)

        # ===== CRITICAL: Joint Index Reordering =====
        # MuJoCo에서 받은 joint 데이터 (Unitree 순서)
        joint_pos_unitree = np.array([ms.q for ms in lowstate.motor_state[:12]], dtype=np.float32)
        joint_vel_unitree = np.array([ms.dq for ms in lowstate.motor_state[:12]], dtype=np.float32)

        # Unitree 순서 -> IsaacLab 순서로 변환
        joint_pos = joint_pos_unitree[UNITREE_TO_ISAACLAB]
        joint_vel = joint_vel_unitree[UNITREE_TO_ISAACLAB]

        # Relative joint positions (IsaacLab 순서)
        joint_pos_rel = joint_pos - self.config.DEFAULT_JOINT_POS

        # Store clean observations before noise (모두 IsaacLab 순서)
        obs_clean = {
            'base_lin_vel': base_lin_vel.copy(),
            'base_ang_vel': base_ang_vel.copy(),
            'projected_gravity': projected_gravity.copy(),
            'velocity_commands': self.vel_command_b.copy(),
            'joint_pos': joint_pos_rel.copy(),
            'joint_vel': joint_vel.copy(),
            'last_actions': self.last_actions.copy()
        }

        # Add noise if enabled
        if add_noise:
            base_lin_vel += self._add_noise(base_lin_vel, 'base_lin_vel')
            base_ang_vel += self._add_noise(base_ang_vel, 'base_ang_vel')
            projected_gravity += self._add_noise(projected_gravity, 'projected_gravity')
            joint_pos_rel += self._add_noise(joint_pos_rel, 'joint_pos')
            joint_vel += self._add_noise(joint_vel, 'joint_vel')

        # Concatenate observation (모두 IsaacLab 순서)
        obs = np.concatenate([
            base_lin_vel,
            base_ang_vel,
            projected_gravity,
            self.vel_command_b,
            joint_pos_rel,
            joint_vel,
            self.last_actions
        ]).astype(np.float32)

        # Clip observations
        obs = np.clip(obs, -self.config.OBS_CLIP, self.config.OBS_CLIP)

        # Update components dict with noisy values
        obs_clean['full_obs'] = obs

        # Debug: log observation statistics every 100 steps
        if self.process_count % 100 == 0 and self.process_count > 0:
            avg_imu_time = self.imu_publish_time_total / self.process_count
            print(f"\n[DEBUG] Step {self.process_count} - Observation Stats:")
            print(f"  IMU ROS2 publish enabled: {self.ros_node is not None}")
            print(f"  Avg IMU publish time: {avg_imu_time:.4f} ms")
            print(f"  Base lin vel: [{base_lin_vel[0]:.3f}, {base_lin_vel[1]:.3f}, {base_lin_vel[2]:.3f}]")
            print(f"  Base ang vel: [{base_ang_vel[0]:.3f}, {base_ang_vel[1]:.3f}, {base_ang_vel[2]:.3f}]")
            print(f"  Projected gravity: [{projected_gravity[0]:.3f}, {projected_gravity[1]:.3f}, {projected_gravity[2]:.3f}]")
            print(f"  Velocity commands: [{self.vel_command_b[0]:.3f}, {self.vel_command_b[1]:.3f}, {self.vel_command_b[2]:.3f}]")
            print(f"  Joint pos (first 3): [{joint_pos_rel[0]:.3f}, {joint_pos_rel[1]:.3f}, {joint_pos_rel[2]:.3f}]")
            print(f"  Joint vel (first 3): [{joint_vel[0]:.3f}, {joint_vel[1]:.3f}, {joint_vel[2]:.3f}]")
            print(f"  Last actions (first 3): [{self.last_actions[0]:.3f}, {self.last_actions[1]:.3f}, {self.last_actions[2]:.3f}]")

        return obs, obs_clean

    def update_last_actions(self, actions: np.ndarray):
        """Update the last actions (IsaacLab 순서로 저장)"""
        self.last_actions = actions.copy()

    def set_velocity_commands(self, vx: float, vy: float, yaw: float):
        """Set velocity commands [vx, vy, yaw]

        Args:
            vx: Forward velocity command (m/s)
            vy: Lateral velocity command (m/s)
            yaw: Target heading (rad) if heading_command=True,
                 or angular velocity (rad/s) if heading_command=False
        """
        # Store input velocity commands
        self.velocity_commands = np.array([vx, vy, yaw], dtype=np.float32)

        # Copy linear velocities to processed commands
        self.vel_command_b[0] = vx  # vx always direct
        self.vel_command_b[1] = vy  # vy always direct

        if self.heading_command:
            # Store target heading for absolute heading control
            self.heading_target = yaw
            # Angular velocity will be computed in _update_vel_command_b()
        else:
            # Direct angular velocity command (relative mode)
            self.vel_command_b[2] = yaw

    def _update_vel_command_b(self, quat: np.ndarray):
        """Update vel_command_b based on heading mode (matching IsaacLab pattern exactly)

        Following IsaacLab code (simplified for single environment):
        if self.cfg.heading_command:
            heading_error = math_utils.wrap_to_pi(self.heading_target - self.robot.data.heading_w)
            self.vel_command_b[2] = torch.clip(
                self.cfg.heading_control_stiffness * heading_error,
                min=self.cfg.ranges.ang_vel_z[0],
                max=self.cfg.ranges.ang_vel_z[1],
            )
        """
        if self.heading_command:
            # Compute current heading from quaternion (equivalent to self.robot.data.heading_w)
            current_heading = self._compute_heading_from_quat(quat)

            # Compute heading error (target - current, wrapped to [-pi, pi])
            heading_error = self._wrap_to_pi(self.heading_target - current_heading)

            # Compute angular velocity with stiffness control and clipping
            # (equivalent to IsaacLab's torch.clip with cfg.ranges.ang_vel_z)
            angular_velocity = self.heading_control_stiffness * heading_error
            ang_vel_limit = 1.0  # equivalent to cfg.ranges.ang_vel_z
            self.vel_command_b[2] = np.clip(angular_velocity, -ang_vel_limit, ang_vel_limit)

        # If heading_command=False (relative mode), vel_command_b[2] is already set
        # directly in set_velocity_commands() as angular velocity command

    def _compute_projected_gravity(self, quat: np.ndarray) -> np.ndarray:
        """Compute normalized gravity vector in body frame (matching Isaac Lab)

        Isaac Lab process:
        1. Get gravity from simulator: [0, 0, -9.81]
        2. Normalize to unit vector: [0, 0, -1]
        3. Transform to body frame using quaternion inverse
        4. Result: normalized direction vector in body frame

        Args:
            quat: Quaternion [w, x, y, z]

        Returns:
            Normalized gravity vector in body frame [-1, 1]
        """
        w, x, y, z = quat

        # World-to-body rotation matrix (transpose of body-to-world)
        R_T = np.array([
            [1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)],
            [2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x)],
            [2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)]
        ], dtype=np.float32)

        # Gravity in world frame (normalized direction vector)
        # Isaac Lab normalizes: gravity_dir = normalize([0, 0, -9.81]) = [0, 0, -1]
        gravity_world_normalized = np.array([0.0, 0.0, -1.0], dtype=np.float32)

        # Transform to body frame
        gravity_body = R_T @ gravity_world_normalized

        # Ensure it's normalized (should already be unit vector from rotation)
        gravity_body_norm = np.linalg.norm(gravity_body)
        if gravity_body_norm > 0:
            gravity_body = gravity_body / gravity_body_norm

        return gravity_body

    def _compute_heading_from_quat(self, quat: np.ndarray) -> float:
        """Compute heading (yaw) angle from quaternion

        Args:
            quat: Quaternion [w, x, y, z]

        Returns:
            Heading angle in radians [-pi, pi]
        """
        w, x, y, z = quat

        # Convert quaternion to yaw angle
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        return float(yaw)

    def _wrap_to_pi(self, angle: float) -> float:
        """Wrap angle to [-pi, pi] range (matching IsaacLab math_utils.wrap_to_pi)"""
        return np.arctan2(np.sin(angle), np.cos(angle))

    def _add_noise(self, data: np.ndarray, noise_type: str) -> np.ndarray:
        """Add uniform noise"""
        if noise_type not in self.config.NOISE_LEVELS:
            return np.zeros_like(data)

        n_min, n_max = self.config.NOISE_LEVELS[noise_type]
        noise = np.random.uniform(n_min, n_max, size=data.shape).astype(np.float32)
        return noise
