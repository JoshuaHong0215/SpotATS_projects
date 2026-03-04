import numpy as np
from omni.isaac.core.utils.rotations import quat_to_rot_matrix

class ObservationBuilder:
    def __init__(self, spot_view, default_pos):
        self.spot = spot_view
        self.prev_action = np.zeros(12, dtype=np.float32)
        self.default_pos = default_pos

    def build(self, cmd_vec):
        joint_pos = self.spot.get_joint_positions()[0][0:12]
        joint_vel = self.spot.get_joint_velocities()[0][0:12]
        lin_vel_I = self.spot.get_linear_velocities()[0]
        ang_vel_I = self.spot.get_angular_velocities()[0]
        _, quat_IB = self.spot.get_world_poses()
        R_IB = quat_to_rot_matrix(quat_IB[0]); R_BI = R_IB.T
        lin_vel_b = R_BI @ lin_vel_I
        ang_vel_b = R_BI @ ang_vel_I
        gravity_b = R_BI @ np.array([0.0, 0.0, -1.0], dtype=np.float32)

        obs = np.zeros(48, dtype=np.float32)
        obs[0:3]   = lin_vel_b
        obs[3:6]   = ang_vel_b
        obs[6:9]   = gravity_b
        obs[9:12]  = cmd_vec
        obs[12:24] = joint_pos - self.default_pos[0:12]
        obs[24:36] = joint_vel
        obs[36:48] = self.prev_action[0:12]
        return obs

    def update_prev_action(self, action):
        self.prev_action = action.copy()