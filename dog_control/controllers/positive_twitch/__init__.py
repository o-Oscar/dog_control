import time

import numpy as np
from dog_control.controllers.base import BaseController


class PositivTwitchController(BaseController):
    def __init__(self):
        super().__init__()
        self.has_been_called = False

        self.period = 1  # second
        self.amplitude = 0.2  # radian

    def _choose_action(self) -> np.ndarray:
        if not self.has_been_called:
            self.has_been_called = True
            self.start_time = time.time()

        delta_time = time.time() - self.start_time
        motor_modulo = int(delta_time / self.period) % 3
        phase = np.pi * (delta_time / self.period - motor_modulo)
        delta_target = np.sin(phase) ** 2 * self.amplitude

        target_pos = np.zeros(12)
        for i in range(12):
            if i % 3 == motor_modulo:
                target_pos[i] = delta_target

        return target_pos

    def _choose_starting_action(self) -> np.ndarray:
        target_pos = self._choose_action()
        self.has_been_called = False
        return target_pos
