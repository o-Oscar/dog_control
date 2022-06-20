import time

import numpy as np
from dog_control.controllers.base import Action, BaseController, RobotState


class DummyController(BaseController):
    def __init__(self):
        super().__init__()
        self.has_been_called = False

        self.period = 2  # second
        self.amplitude = 0.1  # radian

    def _choose_action(self, state: RobotState) -> Action:
        if not self.has_been_called:
            self.has_been_called = True
            self.start_time = time.time()

        delta_time = time.time() - self.start_time
        delta_target = np.sin(2 * np.pi * delta_time / self.period) * self.amplitude

        target_pos = np.zeros(12)
        target_pos[1] = delta_target

        return Action(target_pos)