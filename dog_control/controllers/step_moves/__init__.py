import time

import numpy as np
from dog_control.controllers.base import BaseController


class Controller(BaseController):
    def __init__(self):
        super().__init__()
        self.has_been_called = False

        self.period = 2  # second
        self.amplitude = 0.3  # radian

    def _choose_action(self) -> np.ndarray:
        delta_time = self.cur_frame / 30
        phase = (int(delta_time / self.period) % 2) * 2 - 1
        target_pos = np.zeros(12)
        target_pos[5] = phase * self.amplitude

        return target_pos

    def _choose_starting_action(self) -> np.ndarray:
        target_pos = self._choose_action()
        return target_pos
