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

    def _choose_action(self) -> np.ndarray:

        delta_pos = (
            self.estimator.all_foot_position - NOMINAL_FOOT_POS_TARGET
        ) * self.foot_pos_mask
        target_feet_force = (
            -self.kp * delta_pos - self.kd * self.estimator.all_foot_velocity
        )

        target_joint_force = np.zeros((4, 3))
        for leg_id in range(4):
            target_foot_force = target_feet_force[leg_id * 3 : leg_id * 3 + 3]

            jac = self.estimator.all_jacs[leg_id]
            target_joint_force[leg_id] = (
                target_foot_force.reshape((1, 3)) @ jac.T
            ).flatten()

        zero_impedance_target = (
            self.cur_motor_positions  # + self.dt * self.estimator.all_motor_velocity * 2
        )

        force_delta_target = target_joint_force.flatten() / geom.KP

        target_position = zero_impedance_target + force_delta_target
        # for leg_id in range(4):
        #     # target_position[leg_id * 3 + 0] = self.cur_motor_positions[leg_id * 3 + 0]
        #     target_position[leg_id * 3 + 1] = (
        #         self.cur_motor_positions[leg_id * 3 + 1]
        #         + self.dt * self.estimator.all_motor_velocity[leg_id * 3 + 1]
        #     )
        #     target_position[leg_id * 3 + 2] = self.cur_motor_positions[leg_id * 3 + 2]

        return target_position  # * self.mask

    def _choose_starting_action(self) -> np.ndarray:
        return self.target_position_radians
