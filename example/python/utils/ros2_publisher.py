"""
ROS2 publisher for Go2 diagnostics data
"""
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Header
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState


class Go2DiagnosticsPublisher(Node):
    """ROS2 node for publishing Go2 diagnostics data"""

    def __init__(self):
        super().__init__('go2_diagnostics_publisher')

        # Publishers for observation components
        self.pub_base_lin_vel = self.create_publisher(
            Vector3Stamped, '/go2/obs/base_lin_vel', 10)
        self.pub_base_ang_vel = self.create_publisher(
            Vector3Stamped, '/go2/obs/base_ang_vel', 10)
        self.pub_projected_gravity = self.create_publisher(
            Vector3Stamped, '/go2/obs/projected_gravity', 10)
        self.pub_velocity_commands = self.create_publisher(
            Vector3Stamped, '/go2/obs/velocity_commands', 10)

        # Joint states publisher (IsaacLab order for visualization)
        self.pub_joint_states = self.create_publisher(
            JointState, '/go2/obs/joint_states', 10)

        # Actions publisher
        self.pub_actions = self.create_publisher(
            Float32MultiArray, '/go2/actions/raw', 10)
        self.pub_actions_scaled = self.create_publisher(
            Float32MultiArray, '/go2/actions/scaled', 10)
        self.pub_target_positions = self.create_publisher(
            Float32MultiArray, '/go2/actions/target_positions', 10)
        self.pub_computed_torques = self.create_publisher(
            Float32MultiArray, '/go2/actions/computed_torques', 10)

        # Full observation vector
        self.pub_full_obs = self.create_publisher(
            Float32MultiArray, '/go2/obs/full_vector', 10)

        # Policy diagnostics
        self.pub_inference_time = self.create_publisher(
            Float32, '/go2/diagnostics/inference_time_ms', 10)
        self.pub_loop_time = self.create_publisher(
            Float32, '/go2/diagnostics/loop_time_ms', 10)

        # Heading command diagnostics
        self.pub_current_heading = self.create_publisher(
            Float32, '/go2/diagnostics/current_heading_rad', 10)
        self.pub_target_heading = self.create_publisher(
            Float32, '/go2/diagnostics/target_heading_rad', 10)
        self.pub_heading_error = self.create_publisher(
            Float32, '/go2/diagnostics/heading_error_rad', 10)

        # IMU data for plotting (separate from control loop)
        self.pub_imu_acc = self.create_publisher(
            Vector3Stamped, '/go2/imu/accelerometer', 10)
        self.pub_base_vel = self.create_publisher(
            Vector3Stamped, '/go2/imu/base_velocity', 10)
        self.pub_gravity = self.create_publisher(
            Vector3Stamped, '/go2/imu/projected_gravity', 10)
        self.pub_angular_vel = self.create_publisher(
            Vector3Stamped, '/go2/imu/angular_velocity', 10)

        # Joint names (IsaacLab order for JointState message)
        self.joint_names = [
            'FL_hip', 'FL_thigh', 'FL_calf',
            'FR_hip', 'FR_thigh', 'FR_calf',
            'RL_hip', 'RL_thigh', 'RL_calf',
            'RR_hip', 'RR_thigh', 'RR_calf'
        ]

        self.get_logger().info('Go2 Diagnostics Publisher initialized')

    def publish_observations(self, obs_dict: dict, timestamp: float):
        """Publish all observation components"""

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        # Base linear velocity
        msg = Vector3Stamped()
        msg.header = header
        msg.vector.x = float(obs_dict['base_lin_vel'][0])
        msg.vector.y = float(obs_dict['base_lin_vel'][1])
        msg.vector.z = float(obs_dict['base_lin_vel'][2])
        self.pub_base_lin_vel.publish(msg)

        # Base angular velocity
        msg = Vector3Stamped()
        msg.header = header
        msg.vector.x = float(obs_dict['base_ang_vel'][0])
        msg.vector.y = float(obs_dict['base_ang_vel'][1])
        msg.vector.z = float(obs_dict['base_ang_vel'][2])
        self.pub_base_ang_vel.publish(msg)

        # Projected gravity
        msg = Vector3Stamped()
        msg.header = header
        msg.vector.x = float(obs_dict['projected_gravity'][0])
        msg.vector.y = float(obs_dict['projected_gravity'][1])
        msg.vector.z = float(obs_dict['projected_gravity'][2])
        self.pub_projected_gravity.publish(msg)

        # Velocity commands
        msg = Vector3Stamped()
        msg.header = header
        msg.vector.x = float(obs_dict['velocity_commands'][0])
        msg.vector.y = float(obs_dict['velocity_commands'][1])
        msg.vector.z = float(obs_dict['velocity_commands'][2])
        self.pub_velocity_commands.publish(msg)

        # Joint states (already in IsaacLab order from obs_dict)
        joint_msg = JointState()
        joint_msg.header = header
        joint_msg.name = self.joint_names
        joint_msg.position = [float(p) for p in obs_dict['joint_pos']]
        joint_msg.velocity = [float(v) for v in obs_dict['joint_vel']]
        self.pub_joint_states.publish(joint_msg)

        # Full observation vector
        full_obs_msg = Float32MultiArray()
        full_obs_msg.data = [float(x) for x in obs_dict['full_obs']]
        self.pub_full_obs.publish(full_obs_msg)

    def publish_actions(self, actions_raw: np.ndarray, actions_scaled: np.ndarray,
                       target_positions: np.ndarray, computed_torques: np.ndarray = None):
        """Publish action data (all in IsaacLab order for consistency with observations)"""

        # Raw actions from policy (IsaacLab order)
        msg = Float32MultiArray()
        msg.data = [float(x) for x in actions_raw]
        self.pub_actions.publish(msg)

        # Scaled actions (IsaacLab order)
        msg = Float32MultiArray()
        msg.data = [float(x) for x in actions_scaled]
        self.pub_actions_scaled.publish(msg)

        # Target joint positions (IsaacLab order)
        msg = Float32MultiArray()
        msg.data = [float(x) for x in target_positions]
        self.pub_target_positions.publish(msg)

        # Computed torques (IsaacLab order)
        if computed_torques is not None:
            msg = Float32MultiArray()
            msg.data = [float(x) for x in computed_torques]
            self.pub_computed_torques.publish(msg)

    def publish_diagnostics(self, inference_time_ms: float, loop_time_ms: float,
                           current_heading: float = 0.0, target_heading: float = 0.0,
                           heading_error: float = 0.0):
        """Publish timing and heading diagnostics"""

        msg = Float32()
        msg.data = inference_time_ms
        self.pub_inference_time.publish(msg)

        msg = Float32()
        msg.data = loop_time_ms
        self.pub_loop_time.publish(msg)

        # Heading diagnostics
        msg = Float32()
        msg.data = current_heading
        self.pub_current_heading.publish(msg)

        msg = Float32()
        msg.data = target_heading
        self.pub_target_heading.publish(msg)

        msg = Float32()
        msg.data = heading_error
        self.pub_heading_error.publish(msg)

    def publish_imu_data(self, imu_acc: np.ndarray, base_vel: np.ndarray,
                        projected_gravity: np.ndarray, angular_vel: np.ndarray):
        """Publish raw IMU data (no processing in control loop!)

        Args:
            imu_acc: IMU specific force [x, y, z] in m/s² (at IMU location)
            base_vel: Base velocity [x, y, z] in m/s
            projected_gravity: Normalized gravity direction in body frame
            angular_vel: Angular velocity [x, y, z] in rad/s (body frame)
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        # IMU accelerometer (specific force at IMU location)
        msg = Vector3Stamped()
        msg.header = header
        msg.vector.x = float(imu_acc[0])
        msg.vector.y = float(imu_acc[1])
        msg.vector.z = float(imu_acc[2])
        self.pub_imu_acc.publish(msg)

        # Base velocity (raw)
        msg = Vector3Stamped()
        msg.header = header
        msg.vector.x = float(base_vel[0])
        msg.vector.y = float(base_vel[1])
        msg.vector.z = float(base_vel[2])
        self.pub_base_vel.publish(msg)

        # Projected gravity (normalized direction in body frame)
        msg = Vector3Stamped()
        msg.header = header
        msg.vector.x = float(projected_gravity[0])
        msg.vector.y = float(projected_gravity[1])
        msg.vector.z = float(projected_gravity[2])
        self.pub_gravity.publish(msg)

        # Angular velocity (body frame)
        msg = Vector3Stamped()
        msg.header = header
        msg.vector.x = float(angular_vel[0])
        msg.vector.y = float(angular_vel[1])
        msg.vector.z = float(angular_vel[2])
        self.pub_angular_vel.publish(msg)
