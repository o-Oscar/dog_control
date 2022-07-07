import time

import numpy as np
from dog_control.controllers.base import BaseController


class Controller(BaseController):
    def __init__(self):
        super().__init__()
        self.has_been_called = False

        target_position_degrees = np.array([0, 30, -60] * 4)
        self.target_position_radians = np.radians(target_position_degrees)

    def _choose_action(self) -> np.ndarray:
        # full_target_position = self.target_position_radians.copy()
        delta_target = np.zeros((12,))

        # target_position[0] = -self.cur_motor_positions[3]

        # Dans l'approximation des petits angles, la rotation et son sinus sont égales.
        # Donc on peut utiliser dirrectement le up_vector pour calculer les delta de rotation ds pattes

        for leg_id in range(4):
            delta_target[leg_id * 3 + 0] = -self.up_vector[1]
            delta_target[leg_id * 3 + 1] = +self.up_vector[0]

        return self.target_position_radians + delta_target

    def _choose_starting_action(self) -> np.ndarray:
        return self.target_position_radians
