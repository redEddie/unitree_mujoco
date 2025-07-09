import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC


def HighStateHandler(msg: SportModeState_):
    print("Position: ", msg.position)
    #print("Velocity: ", msg.velocity)


from rich.console import Console
from rich.panel import Panel
from rich.live import Live
import threading

latest_imu_state = None

def LowStateHandler(msg: LowState_):
    global latest_imu_state
    latest_imu_state = msg.imu_state
    # print("motor[0] state: ", msg.motor_state[0])


if __name__ == "__main__":
    ChannelFactoryInitialize(1, "lo")


    low_state_suber = ChannelSubscriber("rt/lowstate", LowState_)
    low_state_suber.Init(LowStateHandler, 10)

    console = Console()
    with Live(Panel("Waiting for IMU state...", title="IMU State", expand=False), console=console, refresh_per_second=10) as live:
        while True:
            panel_content = str(latest_imu_state) if latest_imu_state is not None else "Waiting for IMU state..."
            live.update(Panel(panel_content, title="IMU State", expand=False))
            time.sleep(0.05)

