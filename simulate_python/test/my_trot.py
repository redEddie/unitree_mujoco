import time
import torch
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

# ─────────────────────────────────────────────────────────────────────────────
def stand_up_sequence(cmd_pub, crc, duration=(500,500,1000), gains=(60,5), dt=0.002):
    # wait for first state
    start_q = [0.0]*12
    first = True
    while latest_lowstate is None:
        time.sleep(0.005)
    if first:
        for i in range(12):
            start_q[i] = latest_lowstate.motor_state[i].q
        first = False

    # stand targets from your my_standup.py
    _p1 = [ 0.0, 1.36, -2.65,   0.0, 1.36, -2.65, -0.2, 1.36, -2.65,   0.2, 1.36, -2.65 ]
    _p2 = [ 0.0, 0.67, -1.3,    0.0, 0.67, -1.3,  0.0, 0.67, -1.3,    0.0, 0.67, -1.3  ]

    d1, d2, d3 = duration
    pct1 = pct2 = pct3 = 0.0

    cmd = unitree_go_msg_dds__LowCmd_()
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01

    # four‐phase interpolation
    while True:
        if pct1 < 1:
            pct1 = min(pct1 + 1/d1, 1)
            src, dst = start_q, _p1
            kp, kd = gains
        elif pct2 < 1:
            pct2 = min(pct2 + 1/d2, 1)
            src, dst = _p1, _p2
            kp, kd = gains
        elif pct3 < 1:
            pct3 = min(pct3 + 1/d3, 1)
            src, dst = _p2, _p2
            kp, kd = gains
        else:
            break

        # apply interpolation to each of the 12 joints
        alpha = pct1 if pct1<1 else pct2 if pct2<1 else pct3 if pct3<1 else 1
        for i in range(12):
            q = (1-alpha)*src[i] + alpha*dst[i]
            cmd.motor_cmd[i].q  = q
            cmd.motor_cmd[i].kp = kp
            cmd.motor_cmd[i].kd = kd
            cmd.motor_cmd[i].dq = 0
            cmd.motor_cmd[i].tau= 0

        cmd.crc = crc.Crc(cmd)
        cmd_pub.Write(cmd)
        time.sleep(dt)

def LowStateHandler(msg: LowState_):
    # print("IMU state: ", msg.imu_state)
    # print("motor[0] state: ", msg.motor_state[0])
    global latest_lowstate
    latest_lowstate = msg

def quaternion_to_rotation_matrix(q):
    """
    Convert a quaternion [w, x, y, z] into a 3×3 rotation matrix.
    """
    w, x, y, z = q
    # precompute products
    ww, xx, yy, zz = w*w, x*x, y*y, z*z
    wx, wy, wz = w*x, w*y, w*z
    xy, xz, yz = x*y, x*z, y*z

    R = np.array([
        [ ww + xx - yy - zz,   2*(xy - wz),       2*(xz + wy)    ],
        [ 2*(xy + wz),         ww - xx + yy - zz, 2*(yz - wx)    ],
        [ 2*(xz - wy),         2*(yz + wx),       ww - xx - yy + zz ]
    ], dtype=float)
    return R

class BaseVelocityEstimator:
    def __init__(self, dt: float, gravity: float = 9.81):
        """
        dt: control timestep in seconds (e.g. 0.002 for 500 Hz)
        gravity: magnitude of gravity to subtract (9.81 m/s²)
        """
        self.dt = dt
        self.gravity = gravity
        self.lin_vel = np.zeros(3, dtype=float)

    def update(self, imu_state) -> (np.ndarray, np.ndarray):
        """
        Call this every control tick with the latest imu_state.

        imu_state.quaternion    : list or array of [w, x, y, z]
        imu_state.accelerometer : list or array of [ax, ay, az] in body frame (m/s²)
        imu_state.gyroscope     : list or array of [gx, gy, gz] in body frame (rad/s)

        Returns:
            lin_vel (3,) : estimated base linear velocity in world frame (m/s)
            ang_vel (3,) : base angular velocity in body frame (rad/s)
        """
        # 1) unpack
        q = imu_state.quaternion
        acc_body = np.array(imu_state.accelerometer, dtype=float)
        gyro_body = np.array(imu_state.gyroscope,    dtype=float)

        # 2) rotate body‐frame accel into world frame
        R = quaternion_to_rotation_matrix(q)
        acc_world = R.dot(acc_body)

        # 3) subtract gravity (assumes world‐Z up)
        acc_lin = acc_world - np.array([0.0, 0.0, self.gravity])

        # 4) integrate to get velocity
        self.lin_vel += acc_lin * self.dt

        # 5) angular velocity is just the gyro reading
        ang_vel = gyro_body.copy()

        return self.lin_vel, ang_vel

def compute_projected_gravity(quaternion: np.ndarray, gravity: float = 9.81) -> np.ndarray:
    """
    Given a body quaternion [w, x, y, z], return the gravity vector [0,0,-gravity]
    expressed in the body's root frame (projected onto its axes).
    """
    # 1) Unpack quaternion
    w, x, y, z = quaternion

    # 2) Build rotation matrix R (body→world)
    ww, xx, yy, zz = w*w, x*x, y*y, z*z
    wx, wy, wz = w*x, w*y, w*z
    xy, xz, yz = x*y, x*z, y*z

    R = np.array([
        [ ww + xx - yy - zz,   2*(xy - wz),       2*(xz + wy)    ],
        [ 2*(xy + wz),         ww - xx + yy - zz, 2*(yz - wx)    ],
        [ 2*(xz - wy),         2*(yz + wx),       ww - xx - yy + zz ]
    ], dtype=float)

    # 3) World‐frame gravity
    g_world = np.array([0.0, 0.0, -gravity], dtype=float)

    # 4) Project into body frame: g_body = Rᵀ · g_world
    g_body = R.T.dot(g_world)

    return g_body
# ─────────────────────────────────────────────────────────────────────────────
print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
# input("Press Enter to continue...")

# 1) Load your PPO policy (TorchScript .pt)
POLICY_PATH = "./policy/policy.pt"
policy = torch.jit.load(POLICY_PATH)
policy.eval()

# 2) Initialize simulated Go2 channels (domain=1)
ChannelFactoryInitialize(1, "lo")
state_sub = ChannelSubscriber("rt/lowstate", LowState_)
state_sub.Init(LowStateHandler, 10)
cmd_pub   = ChannelPublisher ("rt/lowcmd",   LowCmd_)
cmd_pub.Init()
crc       = CRC()

# 3) Run stand up sequence
stand_up_sequence(cmd_pub, crc)

# 4) Wait for first LowState_
latest_lowstate = None
while latest_lowstate is None:
    time.sleep(0.001)
# ─────────────────────────────────────────────────────────────────────────────
# 5) Training‐time constants YOU MUST PROVIDE:
#
#    obs_scales.lin_vel   →  torch.Tensor of shape (3,)
#    obs_scales.ang_vel   →  torch.Tensor of shape (3,)
#    obs_scales.dof_pos   →  torch.Tensor of shape (12,)
#    obs_scales.dof_vel   →  torch.Tensor of shape (12,)
#    commands_scale       →  torch.Tensor of shape (3,)
#    default_dof_pos      →  torch.Tensor of shape (12,)
#
#    And your buffer‐sizes for:
#    commands  (shape [1, N_commands], N_commands ≥ 3)
#    actions   (shape [1, N_actions], N_actions ≥ 12)
#
#    (In your snippet, commands[:, :3] is used, and actions is your previous action vector.)
#
# ─────────────────────────────────────────────────────────────────────────────
# 6) Create/initialize your obs‐ & action‐buffers:
# 🟢 means "I fixed it"
# 🟠 means "You fill it" (e.g. scales : to keep all inputs in a roughly ±1 range)
lin_vel_scale = 2.0
ang_vel_scale = 0.25
dof_pos_scale = 1.0
dof_vel_scale = 0.05
commands_scale = 1.0
total_command_dims = 48
default_dof_pos = torch.tensor([0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
    0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763], dtype=torch.float32)
clip_observations = 100.
clip_actions = 100.


obs_scales = lambda: None
obs_scales.lin_vel = torch.tensor(lin_vel_scale, dtype=torch.float32)
obs_scales.ang_vel = torch.tensor(ang_vel_scale, dtype=torch.float32)
obs_scales.dof_pos = torch.tensor(dof_pos_scale, dtype=torch.float32)
obs_scales.dof_vel = torch.tensor(dof_vel_scale, dtype=torch.float32)

commands_scale   = torch.tensor(commands_scale, dtype=torch.float32)
default_dof_pos  = torch.tensor(default_dof_pos.clone().detach(), dtype=torch.float32)

# start with zeros (you can also preload with your last episode’s final commands/actions)
COMMAND_BUF = np.array([1.5, 0, 0, 0])
ACTION_SCALE = 0.5
commands_buf = torch.tensor(COMMAND_BUF, dtype=torch.float32).unsqueeze(0)
actions_buf  = torch.zeros((1, 12), dtype=torch.float32)

# 7) Control loop (e.g. 500 Hz → dt=0.002)
dt = 0.002
while True:
    # ── a) pull in the latest state
    lowstate = latest_lowstate

    # ── b) unpack your LowState_ fields:
    # 🟢 (You may need to adjust these to match the actual attributes in LowState_.)
    imu_state = lowstate.imu_state
    lin_vel, ang_vel = BaseVelocityEstimator(dt).update(imu_state)
    base_lin_vel = torch.tensor(lin_vel, dtype=torch.float32).unsqueeze(0)
    base_ang_vel = torch.tensor(ang_vel, dtype=torch.float32).unsqueeze(0)
    #    if LowState_ gives you projected_gravity directly:
    # 🟢 otherwise you’d compute it via the RPY → rotation matrix on [0,0,-9.81]
    quaternion = imu_state.quaternion
    g_projected = compute_projected_gravity(quaternion)
    projected_gravity = torch.tensor(g_projected, dtype=torch.float32).unsqueeze(0)

    dof_pos = torch.tensor(
        [ms.q  for ms in lowstate.motor_state[:12]], dtype=torch.float32
        ).unsqueeze(0)
    dof_vel = torch.tensor(
        [ms.dq for ms in lowstate.motor_state[:12]], dtype=torch.float32
        ).unsqueeze(0)

    # ── c) build your observation exactly as in training
    obs = torch.cat((
        base_lin_vel   * obs_scales.lin_vel,
        base_ang_vel   * obs_scales.ang_vel,
        projected_gravity,
        commands_buf[:, :3] * commands_scale,
        (dof_pos - default_dof_pos) * obs_scales.dof_pos,
        dof_vel        * obs_scales.dof_vel,
        actions_buf
    ), dim=-1).to(torch.float32)

    obs = torch.clamp(obs, -clip_observations, clip_observations)
    # print(obs.shape)

    # ── d) run your PPO
    # 🟢 lower the actions scale to get working results
    actions_scale = 0.8
    with torch.no_grad():
        new_action = default_dof_pos + policy(obs).cpu() * actions_scale  # shape [1,12]
    actions_buf = commands_buf.clone()
    commands_buf = new_action.clone()    # store for next obs
    commands_buf = torch.clamp(commands_buf, -clip_actions, clip_actions)
    print(f"{new_action.shape}: \n    {new_action}")

    # ── e) map policy output→LowCmd_
    # 🟢 publish: use unitree_go_msg_dds__LowCmd_() instead of LowCmd_()
    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0], cmd.head[1] = 0xFE, 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio       = 0

    for i in range(12):
        cmd.motor_cmd[i].mode = 0x01
        cmd.motor_cmd[i].q    = float(new_action[0,i])
        cmd.motor_cmd[i].kp   = 50.0    # or your trained Kp
        cmd.motor_cmd[i].kd   = 5.0     # or your trained Kd
        cmd.motor_cmd[i].dq   = 0.0
        cmd.motor_cmd[i].tau  = 0.0

    cmd.crc = crc.Crc(cmd)
    cmd_pub.Write(cmd)

    # ── f) step timing
    time.sleep(dt)
