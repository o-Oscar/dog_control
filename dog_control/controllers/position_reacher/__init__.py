import time

import numpy as np
from dog_control.controllers.base import BaseController
from dog_control.controllers.jaccobian import (
    LEFT_LEG_POSITIONS,
    LEG_FIRST_SITES,
    LEG_MOTOR_AXES,
    RIGHT_LEG_POSITIONS,
    calc_jac,
)
from dog_control.controllers.velocity_estimator import VelocityEstimator
from scipy.spatial.transform import Rotation as R


class Controller(BaseController):
    def __init__(self):
        super().__init__()

        self.velocity_estimator = VelocityEstimator(self.logger)

        target_position_degrees = np.array([0, 30, -60] * 4)
        self.target_position_radians = np.radians(target_position_degrees)

    def _choose_action(self) -> np.ndarray:

        self.velocity_estimator.estimate(
            self.cur_motor_positions, self.up_vector, self.rotation_speed
        )

        return self.target_position_radians

    def _choose_starting_action(self) -> np.ndarray:
        return self.target_position_radians
