import time

import numpy as np
from dog_control.controllers.base import BaseController
from dog_control.controllers.velocity_estimator import VelocityEstimator


class Controller(BaseController):
    def __init__(self):
        super().__init__()

        self.has_been_called = False
        self.dt = 1 / 30

        self.velocity_estimator = VelocityEstimator(self.logger)

    def _choose_action(self) -> np.ndarray:
        target_position = np.zeros((12,))

        self.velocity_estimator.estimate(
            self.cur_motor_positions, self.up_vector, self.rotation_speed
        )

        for leg_id in range(4):
            # target_position[leg_id * 3 + 0] = self.cur_motor_positions[leg_id * 3 + 0]
            target_position[leg_id * 3 + 1] = self.cur_motor_positions[leg_id * 3 + 1]
            target_position[leg_id * 3 + 2] = self.cur_motor_positions[leg_id * 3 + 2]

        return target_position

    def _choose_starting_action(self) -> np.ndarray:
        return np.zeros((12,))


class SymetricFrontController(BaseController):
    def __init__(self):
        super().__init__()
        self.has_been_called = False

    def _choose_action(self) -> np.ndarray:
        target_position = np.zeros((12,))

        # # target_position[0] = -self.cur_motor_positions[3]
        # target_position[1] = self.cur_motor_positions[4]
        # target_position[2] = self.cur_motor_positions[5]

        # # target_position[3] = self.cur_motor_positions[3]
        # target_position[4] = self.cur_motor_positions[4]
        # target_position[5] = self.cur_motor_positions[5]

        # target_position[0] = -self.cur_motor_positions[3]
        target_position[1] = (
            self.cur_motor_positions[1] + self.cur_motor_positions[4]
        ) / 2
        target_position[2] = (
            self.cur_motor_positions[2] + self.cur_motor_positions[5]
        ) / 2

        # target_position[3] = self.cur_motor_positions[3]
        target_position[4] = (
            self.cur_motor_positions[1] + self.cur_motor_positions[4]
        ) / 2
        target_position[5] = (
            self.cur_motor_positions[2] + self.cur_motor_positions[5]
        ) / 2

        return target_position

    def _choose_starting_action(self) -> np.ndarray:
        return np.zeros((12,))
