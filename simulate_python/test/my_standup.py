# go2_stand_mujoco.py
import time
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

# 1) MuJoCo 시뮬레이션은 domain=1 사용
ChannelFactoryInitialize(1, "lo")  # :contentReference[oaicite:1]{index=1}

# 2) Publisher / Subscriber 설정
lowcmd_puber = ChannelPublisher("rt/lowcmd", LowCmd_)
lowcmd_puber.Init()

start_positions = [0.0]*12
first_run = True

def lowstate_handler(msg: LowState_):
    global start_positions, first_run
    if first_run:
        for i in range(12):
            start_positions[i] = msg.motor_state[i].q
        first_run = False

lowstate_suber = ChannelSubscriber("rt/lowstate", LowState_)
lowstate_suber.Init(lowstate_handler, 10)

# 3) 명령 메시지 기본 템플릿
cmd = unitree_go_msg_dds__LowCmd_()
crc = CRC()
cmd.head[0], cmd.head[1] = 0xFE, 0xEF
cmd.level_flag = 0xFF
cmd.gpio = 0
for i in range(20):
    cmd.motor_cmd[i].mode = 0x01
    cmd.motor_cmd[i].q = 0.0
    cmd.motor_cmd[i].kp = 0.0
    cmd.motor_cmd[i].dq = 0.0
    cmd.motor_cmd[i].kd = 0.0
    cmd.motor_cmd[i].tau = 0.0

# 4) go2_stand_example.py 에 정의된 목표 포지션 및 타이밍 :contentReference[oaicite:2]{index=2}
_targetPos_1 = [ 0.0, 1.36, -2.65,   0.0, 1.36, -2.65,
                 -0.2, 1.36, -2.65,   0.2, 1.36, -2.65 ]
_targetPos_2 = [ 0.0, 0.67, -1.3,    0.0, 0.67, -1.3,
                  0.0, 0.67, -1.3,    0.0, 0.67, -1.3 ]
_targetPos_3 = [-0.35, 1.36, -2.65,  0.35, 1.36, -2.65,
                 -0.5,  1.36, -2.65,  0.5,  1.36, -2.65 ]

duration_1, duration_2, duration_3, duration_4 = 500, 500, 1000, 900
pct1 = pct2 = pct3 = pct4 = 0.0
dt = 0.002

# 5) 시뮬레이터가 첫 상태를 보낼 때까지 대기
while first_run:
    time.sleep(0.01)

# 6) 메인 루프: 4단계 선형 보간
while True:
    # Stage 1: startPos → _targetPos_1
    if pct1 < 1.0:
        pct1 = min(pct1 + 1.0/duration_1, 1.0)
        for i in range(12):
            cmd.motor_cmd[i].q  = (1-pct1)*start_positions[i] + pct1*_targetPos_1[i]
            cmd.motor_cmd[i].kp = 60.0
            cmd.motor_cmd[i].kd =  5.0
            cmd.motor_cmd[i].dq = 0
            cmd.motor_cmd[i].tau= 0

    # Stage 2: _targetPos_1 → _targetPos_2
    elif pct2 < 1.0:
        pct2 = min(pct2 + 1.0/duration_2, 1.0)
        for i in range(12):
            cmd.motor_cmd[i].q  = (1-pct2)*_targetPos_1[i] + pct2*_targetPos_2[i]
            cmd.motor_cmd[i].kp = 60.0
            cmd.motor_cmd[i].kd =  5.0
            cmd.motor_cmd[i].dq = 0
            cmd.motor_cmd[i].tau= 0

    # Stage 3: hold at _targetPos_2
    elif pct3 < 1.0:
        pct3 = min(pct3 + 1.0/duration_3, 1.0)
        for i in range(12):
            cmd.motor_cmd[i].q  = _targetPos_2[i]
            cmd.motor_cmd[i].kp = 60.0
            cmd.motor_cmd[i].kd =  5.0
            cmd.motor_cmd[i].dq = 0
            cmd.motor_cmd[i].tau= 0

    # Stage 4: _targetPos_2 → _targetPos_3
    elif pct4 < 1.0:
        pct4 = min(pct4 + 1.0/duration_4, 1.0)
        for i in range(12):
            cmd.motor_cmd[i].q  = (1-pct4)*_targetPos_2[i] + pct4*_targetPos_3[i]
            cmd.motor_cmd[i].kp = 60.0
            cmd.motor_cmd[i].kd =  5.0
            cmd.motor_cmd[i].dq = 0
            cmd.motor_cmd[i].tau= 0

    else:
        print("Stand sequence complete.")
        break

    # CRC 계산 후 퍼블리시
    cmd.crc = crc.Crc(cmd)
    lowcmd_puber.Write(cmd)
    time.sleep(dt)
