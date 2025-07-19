"""
python3 swing_one.py --joint 9
"""

import time
import sys
import numpy as np

import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--joint', type=int, default=3, help='Joint index to swing')
parser.add_argument('--interface', type=str, default='lo', help='DDS interface (default: lo)')
parser.add_argument('--domain', type=int, default=1, help='DDS domain id (1 for mujoco, 0 for real)')
args = parser.parse_args()

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

stand_up_joint_pos = np.array([
    0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
    0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763
],
                              dtype=float)

stand_down_joint_pos = np.array([
    0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
    1.22187, -2.44375, -0.0473455, 1.22187, -2.44375
],
                                dtype=float)

actions_buf = np.zeros(12, dtype=float)
actions_buf[int(args.joint)] = -0.5

dt = 0.002
runing_time = 0.0
crc = CRC()

# Projected gravity (from quaternion)
def compute_projected_gravity(quaternion, gravity=9.81):
    w, x, y, z = quaternion
    ww, xx, yy, zz = w*w, x*x, y*y, z*z
    wx, wy, wz = w*x, w*y, w*z
    xy, xz, yz = x*y, x*z, y*z
    R = np.array([
        [ ww + xx - yy - zz,   2*(xy - wz),       2*(xz + wy)    ],
        [ 2*(xy + wz),         ww - xx + yy - zz, 2*(yz - wx)    ],
        [ 2*(xz - wy),         2*(yz + wx),       ww - xx - yy + zz ]
    ], dtype=float)
    g_world = np.array([0.0, 0.0, -gravity], dtype=float)
    return R.T.dot(g_world)

def lowstate_callback(msg):
    global qpos, qvel, imu_state
    qpos = np.array([m.q for m in msg.motor_state[:12]])
    qvel = np.array([m.dq for m in msg.motor_state[:12]])
    imu_state = msg.imu_state

input("Press enter to start")

if __name__ == '__main__':

    if args.domain == 1:
        ChannelFactoryInitialize(1, "lo")
        print("ChannelFactoryInitialize(1, \"lo\")")
    else:
        ChannelFactoryInitialize(0, args.interface)
        print(f"ChannelFactoryInitialize(0, {args.interface})")

    # Create a publisher to publish the data defined in UserData class
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init(lowstate_callback, 10)


    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0

    while True:
        step_start = time.perf_counter()

        runing_time += dt

        if (runing_time < 3.0):
            # Stand up in first 3 second
            
            # Total time for standing up or standing down is about 1.2s
            phase = np.tanh(runing_time / 1.2)
            for i in range(12):
                cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (
                    1 - phase) * stand_down_joint_pos[i]
                cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].tau = 0.0
        else:
            # swing one leg
            for i in range(12):
                cmd.motor_cmd[i].q = actions_buf[i] + stand_up_joint_pos[i]
                cmd.motor_cmd[i].kp = 50.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].tau = 0.0

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)