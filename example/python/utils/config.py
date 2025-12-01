"""
Configuration and constants for IsaacLab policy runner
"""
import numpy as np

# =============================================================================
# Joint Index Reordering Constants
# =============================================================================

# Unitree SDK/MuJoCo -> IsaacLab 변환
# IsaacLab[i] = Unitree[UNITREE_TO_ISAACLAB[i]]
UNITREE_TO_ISAACLAB = np.array([
    3, 0, 9, 6,   # FL_hip, FR_hip, RL_hip, RR_hip
    4, 1, 10, 7,  # FL_thigh, FR_thigh, RL_thigh, RR_thigh
    5, 2, 11, 8   # FL_calf, FR_calf, RL_calf, RR_calf
], dtype=np.int32)

# IsaacLab -> Unitree SDK/MuJoCo 변환
# Unitree[i] = IsaacLab[ISAACLAB_TO_UNITREE[i]]
ISAACLAB_TO_UNITREE = np.array([
    1, 5, 9,   # FR_hip, FR_thigh, FR_calf
    0, 4, 8,   # FL_hip, FL_thigh, FL_calf
    3, 7, 11,  # RR_hip, RR_thigh, RR_calf
    2, 6, 10   # RL_hip, RL_thigh, RL_calf
], dtype=np.int32)

# =============================================================================
# Policy Configuration
# =============================================================================

class PolicyConfig:
    """Configuration parameters matching IsaacLab training setup"""

    CONTROL_DT = 0.02  # 50Hz control frequency

    NOISE_LEVELS = {
        'base_lin_vel': (-0.1, 0.1),
        'base_ang_vel': (-0.2, 0.2),
        'projected_gravity': (-0.05, 0.05),
        'joint_pos': (-0.01, 0.01),
        'joint_vel': (-1.5, 1.5),
    }

    ACTION_SCALE = 0.25

    # DEFAULT_JOINT_POS는 Unitree/MuJoCo 순서로 정의됨. IsaacLab 순서로 변환 필요.
    DEFAULT_JOINT_POS = np.array([
        -0.1, 0.8, -1.5,   # FR_hip, FR_thigh, FR_calf
        0.1, 0.8, -1.5,    # FL_hip, FL_thigh, FL_calf
        -0.1, 1.0, -1.5,   # RR_hip, RR_thigh, RR_calf
        0.1, 1.0, -1.5,    # RL_hip, RL_thigh, RL_calf
    ], dtype=np.float32)

    # Unitree/MuJoCo 순서 → IsaacLab 순서로 변환
    DEFAULT_JOINT_POS = DEFAULT_JOINT_POS[UNITREE_TO_ISAACLAB]

    KP = 25.0
    KD = 0.5

    OBS_CLIP = 100.0
    ACTION_CLIP = 23.5  # effort limit
