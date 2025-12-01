#!/usr/bin/env python3
"""
Standalone ROS2 IMU Data Plotter
=================================

This script runs as a separate process from the control loop, subscribing to
IMU topics and visualizing them in real-time using PyQtGraph.

Plots:
- Linear Velocity: Computed (from SportModeState) vs GT
- Linear Acceleration: Sensor data (specific force + gravity)
- Linear Specific Force: Sensor data (IMU raw)

Usage:
    # Terminal 1: Run policy with IMU publishing
    python3 policy_runner.py --enable-plot

    # Terminal 2: Run this plotter (separate process)
    python3 imu_plotter_ros2.py

ROS2 Topics:
    - /go2/imu/accelerometer: IMU specific force [x, y, z] m/s²
    - /go2/imu/base_velocity: Base linear velocity [x, y, z] m/s
    - /go2/imu/projected_gravity: Normalized gravity direction in body frame
"""

import sys
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore


class IMUPlotterNode(Node):
    """ROS2 node that subscribes to IMU topics and plots them"""

    def __init__(self, window_size=10):
        super().__init__('imu_plotter_node')

        self.window_size = window_size

        # Data buffers for 9 plots (3x3 grid)
        self.timestamps = deque(maxlen=window_size)

        # Row 0: Linear Velocity (computed vs GT)
        self.lin_vel_computed_x = deque(maxlen=window_size)
        self.lin_vel_computed_y = deque(maxlen=window_size)
        self.lin_vel_computed_z = deque(maxlen=window_size)
        self.lin_vel_gt_x = deque(maxlen=window_size)
        self.lin_vel_gt_y = deque(maxlen=window_size)
        self.lin_vel_gt_z = deque(maxlen=window_size)

        # Row 1: Linear Acceleration (sensor only)
        self.lin_acc_x = deque(maxlen=window_size)
        self.lin_acc_y = deque(maxlen=window_size)
        self.lin_acc_z = deque(maxlen=window_size)

        # Row 2: Linear Specific Force (sensor only)
        self.lin_spec_force_x = deque(maxlen=window_size)
        self.lin_spec_force_y = deque(maxlen=window_size)
        self.lin_spec_force_z = deque(maxlen=window_size)

        self.start_time = None

        # For velocity integration from IMU acceleration
        self.integrated_velocity = np.zeros(3, dtype=np.float32)
        self.prev_acceleration = None
        self.prev_acceleration_time = None

        # Latest sensor data
        self.latest_linear_velocity_computed = None
        self.latest_linear_velocity_gt = None
        self.latest_linear_specific_force = None  # IMU measured specific force
        self.latest_linear_acceleration = None  # specific_force + gravity
        self.latest_gravity = None  # Normalized gravity direction

        # ROS2 subscribers
        self.sub_imu = self.create_subscription(
            Vector3Stamped,
            '/go2/imu/accelerometer',
            self.imu_callback,
            10)

        self.sub_vel = self.create_subscription(
            Vector3Stamped,
            '/go2/imu/base_velocity',
            self.velocity_callback,
            10)

        self.sub_gravity = self.create_subscription(
            Vector3Stamped,
            '/go2/imu/projected_gravity',
            self.gravity_callback,
            10)

        self.get_logger().info('IMU Plotter Node initialized')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - /go2/imu/accelerometer (linear specific force)')
        self.get_logger().info('  - /go2/imu/base_velocity (linear velocity)')
        self.get_logger().info('  - /go2/imu/projected_gravity (for computing acceleration)')

    def imu_callback(self, msg: Vector3Stamped):
        """Store latest IMU specific force and compute linear acceleration"""
        linear_specific_force = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=np.float32)
        self.latest_linear_specific_force = linear_specific_force

        # Compute linear acceleration: a = f + g
        if self.latest_gravity is not None:
            gravity_vector = self.latest_gravity * 9.81  # Convert to m/s²
            self.latest_linear_acceleration = linear_specific_force + gravity_vector
        else:
            self.latest_linear_acceleration = linear_specific_force  # Fallback

        self._process_data()

    def gravity_callback(self, msg: Vector3Stamped):
        """Store latest projected gravity (normalized direction in body frame)"""
        self.latest_gravity = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=np.float32)

    def velocity_callback(self, msg: Vector3Stamped):
        """Store SportModeState velocity and integrate IMU acceleration"""
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        sportmode_velocity = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=np.float32)

        self.latest_linear_velocity_gt = sportmode_velocity

        # Integrate IMU acceleration
        if self.latest_linear_acceleration is not None:
            if self.prev_acceleration is not None and self.prev_acceleration_time is not None:
                dt = current_time - self.prev_acceleration_time
                if 0 < dt < 0.5:
                    self.integrated_velocity = self._integrate_rk4(
                        self.integrated_velocity,
                        self.prev_acceleration,
                        self.latest_linear_acceleration,
                        dt
                    )

            self.prev_acceleration = self.latest_linear_acceleration.copy()
            self.prev_acceleration_time = current_time

        self.latest_linear_velocity_computed = self.integrated_velocity.copy()

        self._process_data()

    def _integrate_rk4(self, v0: np.ndarray, a0: np.ndarray, a1: np.ndarray, dt: float) -> np.ndarray:
        """RK4 integration: v = v0 + dt/6 * (k1 + 2*k2 + 2*k3 + k4)"""
        k1 = a0
        k2 = (a0 + a1) / 2
        k3 = k2
        k4 = a1
        return v0 + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

    def _process_data(self):
        """Process and store data when all required data is available"""
        # Check if we have all required data
        if (self.latest_linear_velocity_computed is None or
            self.latest_linear_velocity_gt is None or
            self.latest_linear_specific_force is None or
            self.latest_linear_acceleration is None):
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()

        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Add data to buffers
        self.timestamps.append(current_time)

        # Linear Velocity (computed vs GT)
        self.lin_vel_computed_x.append(self.latest_linear_velocity_computed[0])
        self.lin_vel_computed_y.append(self.latest_linear_velocity_computed[1])
        self.lin_vel_computed_z.append(self.latest_linear_velocity_computed[2])
        self.lin_vel_gt_x.append(self.latest_linear_velocity_gt[0])
        self.lin_vel_gt_y.append(self.latest_linear_velocity_gt[1])
        self.lin_vel_gt_z.append(self.latest_linear_velocity_gt[2])

        # Linear Acceleration (sensor only)
        self.lin_acc_x.append(self.latest_linear_acceleration[0])
        self.lin_acc_y.append(self.latest_linear_acceleration[1])
        self.lin_acc_z.append(self.latest_linear_acceleration[2])

        # Linear Specific Force (sensor only)
        self.lin_spec_force_x.append(self.latest_linear_specific_force[0])
        self.lin_spec_force_y.append(self.latest_linear_specific_force[1])
        self.lin_spec_force_z.append(self.latest_linear_specific_force[2])

        # Log current data to terminal
        print(
            f"t={current_time:.3f}s | "
            f"ComputedVel=({self.latest_linear_velocity_computed[0]: .3f}, "
            f"{self.latest_linear_velocity_computed[1]: .3f}, "
            f"{self.latest_linear_velocity_computed[2]: .3f}) | "
            f"LinAcc=({self.latest_linear_acceleration[0]: .3f}, "
            f"{self.latest_linear_acceleration[1]: .3f}, "
            f"{self.latest_linear_acceleration[2]: .3f}) | "
            f"LinForce=({self.latest_linear_specific_force[0]: .3f}, "
            f"{self.latest_linear_specific_force[1]: .3f}, "
            f"{self.latest_linear_specific_force[2]: .3f})"
        )

    def get_plot_data(self):
        """Get current plot data for all 9 plots"""
        if len(self.timestamps) < 2:
            return None

        return {
            't': np.array(self.timestamps),
            # Linear Velocity (computed vs GT)
            'lin_vel_computed_x': np.array(self.lin_vel_computed_x),
            'lin_vel_computed_y': np.array(self.lin_vel_computed_y),
            'lin_vel_computed_z': np.array(self.lin_vel_computed_z),
            'lin_vel_gt_x': np.array(self.lin_vel_gt_x),
            'lin_vel_gt_y': np.array(self.lin_vel_gt_y),
            'lin_vel_gt_z': np.array(self.lin_vel_gt_z),
            # Linear Acceleration (sensor)
            'lin_acc_x': np.array(self.lin_acc_x),
            'lin_acc_y': np.array(self.lin_acc_y),
            'lin_acc_z': np.array(self.lin_acc_z),
            # Linear Specific Force (sensor)
            'lin_force_x': np.array(self.lin_spec_force_x),
            'lin_force_y': np.array(self.lin_spec_force_y),
            'lin_force_z': np.array(self.lin_spec_force_z),
        }


def main():
    """Main function"""
    print("=" * 70)
    print("ROS2 IMU Data Plotter (Standalone)")
    print("=" * 70)
    print("\nThis plotter runs separately from the control loop.")
    print("Start policy_runner.py with --enable-plot first!")
    print("\nPlot Layout:")
    print("  Row 0: Linear Velocity (Computed vs GT)")
    print("  Row 1: Linear Acceleration (Sensor: specific_force + gravity)")
    print("  Row 2: Linear Specific Force (Sensor: IMU raw)")
    print()

    # Initialize ROS2
    rclpy.init()
    node = IMUPlotterNode(window_size=10)  # 10 seconds of data at ~20Hz = 200 points

    # Create Qt application
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    # Create window with 9 plots (3x3 grid)
    win = pg.GraphicsLayoutWidget(show=True)
    win.resize(1800, 1200)
    win.setWindowTitle('ROS2 IMU Data: Linear Velocity, Acceleration, and Specific Force')

    # Configure PyQtGraph
    pg.setConfigOptions(antialias=True)

    # Plot organization: 3 rows x 3 columns
    # Rows: Linear Velocity, Linear Acceleration, Linear Specific Force
    # Columns: X (Forward), Y (Left), Z (Up)

    plots = []
    curves_primary = []  # Computed for velocity, sensor for others
    curves_secondary = []  # GT for velocity, None for others

    axis_labels = ['X (Forward)', 'Y (Left)', 'Z (Up)']
    row_labels = ['Linear Velocity', 'Linear Acceleration', 'Linear Specific Force']
    colors_primary = ['r', 'g', 'b']
    colors_secondary = ['#8B0000', '#006400', '#00008B']

    for row in range(3):  # 3 rows
        for col in range(3):  # 3 columns (X, Y, Z)
            p = win.addPlot(row=row, col=col)
            p.setTitle(f'{axis_labels[col]} {row_labels[row]}')

            if row == 0:  # Velocity
                p.setLabel('left', 'Vel', units='m/s')
            else:  # Acceleration and Specific Force
                p.setLabel('left', 'Acc', units='m/s²')
            p.setLabel('bottom', 'Time', units='s')

            # Set Y range based on plot type
            if row == 0:  # Linear Velocity
                p.setYRange(-2, 2)
            elif row == 1:  # Linear Acceleration
                if col < 2:  # X, Y
                    p.setYRange(-2, 2)
                else:  # Z
                    p.setYRange(-15, 15)
            else:  # Linear Specific Force
                if col < 2:  # X, Y
                    p.setYRange(-2, 2)
                else:  # Z
                    p.setYRange(-15, 15)

            p.showGrid(x=True, y=True, alpha=0.3)
            p.addLegend()

            # For velocity: Computed vs GT
            if row == 0:
                curve_primary = p.plot(pen=pg.mkPen(colors_primary[col], width=2), name='Computed')
                curve_secondary = p.plot(pen=pg.mkPen(colors_secondary[col], width=2, style=QtCore.Qt.DashLine), name='GT')
            else:
                # For acceleration and specific force: sensor data only
                curve_primary = p.plot(pen=pg.mkPen(colors_primary[col], width=2), name='Sensor')
                curve_secondary = None

            plots.append(p)
            curves_primary.append(curve_primary)
            curves_secondary.append(curve_secondary)

    # Update function
    def update():
        # Spin ROS2 node to process callbacks
        rclpy.spin_once(node, timeout_sec=0.001)

        # Update plots
        data = node.get_plot_data()
        if data is not None:
            t = data['t']

            # Row 0: Linear Velocity (Computed vs GT)
            curves_primary[0].setData(t, data['lin_vel_computed_x'])
            curves_secondary[0].setData(t, data['lin_vel_gt_x'])
            curves_primary[1].setData(t, data['lin_vel_computed_y'])
            curves_secondary[1].setData(t, data['lin_vel_gt_y'])
            curves_primary[2].setData(t, data['lin_vel_computed_z'])
            curves_secondary[2].setData(t, data['lin_vel_gt_z'])

            # Row 1: Linear Acceleration (sensor only)
            curves_primary[3].setData(t, data['lin_acc_x'])
            curves_primary[4].setData(t, data['lin_acc_y'])
            curves_primary[5].setData(t, data['lin_acc_z'])

            # Row 2: Linear Specific Force (sensor only)
            curves_primary[6].setData(t, data['lin_force_x'])
            curves_primary[7].setData(t, data['lin_force_y'])
            curves_primary[8].setData(t, data['lin_force_z'])

    # Set up timer for updates (50ms = 20Hz)
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)

    print("✓ Plotter window opened")
    print("✓ Waiting for IMU data...\n")

    # Run Qt event loop
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("\nShutting down plotter...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
