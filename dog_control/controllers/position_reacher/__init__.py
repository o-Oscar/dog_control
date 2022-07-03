import time

import numpy as np
from dog_control.controllers.base import BaseController


class PositionReacherController(BaseController):
    def __init__(self):
        super().__init__()
        self.has_been_called = False

        target_position_degrees = np.array([0, 20, -40] * 4)
        self.target_position_radians = np.radians(target_position_degrees)

    def _choose_action(self) -> np.ndarray:
        return self.target_position_radians

    def _choose_starting_action(self) -> np.ndarray:
        return self.target_position_radians
