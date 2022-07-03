import abc
import dataclasses
import time

import numpy as np
from dog_control.controllers.logger import Logger


class BaseController(abc.ABC):
    def __init__(self):
        self.logger = Logger()
        self.cur_frame = 0

    @abc.abstractmethod
    def _choose_action(self) -> np.ndarray:
        pass

    def choose_action(self, frame: int, cur_motor_positions: np.ndarray) -> np.ndarray:
        self.cur_frame = frame
        self.cur_motor_positions = cur_motor_positions

        action = self._choose_action()
        self.logger.log(action, cur_motor_positions)
        return action  # TODO : implement target position limitation here

    @abc.abstractmethod
    def _choose_starting_action(self) -> np.ndarray:
        pass

    def choose_starting_action(self) -> np.ndarray:
        return (
            self._choose_starting_action()
        )  # TODO : implement target position limitation here
