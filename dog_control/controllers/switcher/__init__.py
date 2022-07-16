import time

import dog_control.controllers.estimator.geometry as geom
import numpy as np
from dog_control.controllers.base import BaseController
from dog_control.controllers.estimator import Estimator

NOMINAL_DZ = -0.35
NOMINAL_FOOT_POS_TARGET = np.array(
    [geom.FIRST_SITE_DX, (geom.FIRST_SITE_DY + geom.SHOULDER_DY), NOMINAL_DZ]
    + [geom.FIRST_SITE_DX, -(geom.FIRST_SITE_DY + geom.SHOULDER_DY), NOMINAL_DZ]
    + [-geom.FIRST_SITE_DX, (geom.FIRST_SITE_DY + geom.SHOULDER_DY), NOMINAL_DZ]
    + [-geom.FIRST_SITE_DX, -(geom.FIRST_SITE_DY + geom.SHOULDER_DY), NOMINAL_DZ]
)


class Controller(BaseController):
    def __init__(self):
        super().__init__()

        self.has_been_called = False
        self.dt = 1 / 30

        self.mask = np.array([0, 1, 1] * 4)
        self.foot_pos_mask = np.array([0, 0, 1] * 4)

        self.kp = 1500  # 1500
        self.kd = 0
        # self.feet_target_forces = np.zeros((4, 3))
        # self.feet_target_forces[1, 2] = 70

        target_position_degrees = np.array([0, 30, -60] * 4)
        self.target_position_radians = np.radians(target_position_degrees)
        # landed_position_degrees = np.array([0, 25, -50] * 4)
        landed_position_degrees = target_position_degrees
        self.landed_position_radians = np.radians(landed_position_degrees)

        self.is_landed = False
        self.landing_frame = 0

    def _choose_action(self) -> np.ndarray:

        total_foot_force = np.sum(
            self.estimator.all_foot_force_global.reshape(4, 3), axis=0
        )
        if (
            total_foot_force[2] > -150
            or self.cur_frame < 60
            or np.sum(self.estimator.all_force_flag) > 0
        ):
            self.is_landed = False
            print("feels up")
        else:
            if not self.is_landed:
                self.landing_frame = self.cur_frame
            self.is_landed = True
            print("feels landed")

            target_feet_force = self.calc_target_force()

            target_joint_force = np.zeros((4, 3))
            for leg_id in range(4):
                target_foot_force = target_feet_force[leg_id * 3 : leg_id * 3 + 3]

                jac = self.estimator.all_jacs[leg_id]
                target_joint_force[leg_id] = (
                    target_foot_force.reshape((1, 3)) @ jac.T
                ).flatten()

            self.landed_position_radians = (
                self.cur_motor_positions + target_joint_force.flatten() / geom.KP
            )

        if (not self.is_landed) or (self.cur_frame - self.landing_frame < 30):
            return self.target_position_radians

        else:
            return self.landed_position_radians

    def _choose_starting_action(self) -> np.ndarray:
        return self.target_position_radians

    def calc_target_force(self):
        cur_feet_force = self.estimator.all_foot_force_local

        mean_feet_pos = np.mean(
            self.estimator.all_foot_position_global.reshape(4, 3), axis=0
        )
        delta_base_force = (mean_feet_pos - np.array([0, 0, -0.36])) * 100

        force_matrix = np.concatenate([np.eye(3)] * 4, axis=1)
        # torque_matrix = geom.get_moment_matrix(self.estimator.all_foot_position) # TODO stabilize the torque
        current_base_force = force_matrix @ cur_feet_force

        target_base_force = current_base_force - delta_base_force

        target_feet_force, _, _, s = np.linalg.lstsq(force_matrix, target_base_force)

        return target_feet_force
