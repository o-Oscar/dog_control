import abc
import dataclasses
import time

import numpy as np
from dog_control.controllers.logger import Logger


class BaseController(abc.ABC):
    def __init__(self):
        self.logger = Logger()
        self.logger.add_entry("action", 12)
        self.logger.add_entry("cur_motor_positions", 12)
        self.logger.add_entry("up_vector", 3)
        self.logger.add_entry("rotation_speed", 3)

        self.cur_frame = 0

        SHOULDER_RANGE_DEGREES = 20
        THIGH_RANGE_DEGREES = 45
        LOWER_FOOT_RANGE_DEGREE = 90

        self.action_min_degrees = np.array(
            [
                -SHOULDER_RANGE_DEGREES,
                -THIGH_RANGE_DEGREES,
                -LOWER_FOOT_RANGE_DEGREE,
            ]
            * 4
        )
        self.action_max_degrees = np.array(
            [
                SHOULDER_RANGE_DEGREES,
                THIGH_RANGE_DEGREES,
                LOWER_FOOT_RANGE_DEGREE,
            ]
            * 4
        )

        self.action_min = np.radians(self.action_min_degrees)
        self.action_max = np.radians(self.action_max_degrees)

    @abc.abstractmethod
    def _choose_action(self) -> np.ndarray:
        pass

    def choose_action(
        self,
        frame: int,
        cur_motor_positions: np.ndarray,
        up_vector: np.ndarray,
        rotation_speed: np.ndarray,
    ) -> np.ndarray:
        self.cur_frame = frame
        self.cur_motor_positions = cur_motor_positions
        self.up_vector = up_vector
        self.rotation_speed = rotation_speed

        action = self._choose_action()
        cliped_action = np.clip(action, self.action_min, self.action_max)

        self.logger["action"] = cliped_action
        self.logger["cur_motor_positions"] = cur_motor_positions
        self.logger["up_vector"] = up_vector
        self.logger["rotation_speed"] = rotation_speed
        self.logger.step()

        return cliped_action

    @abc.abstractmethod
    def _choose_starting_action(self) -> np.ndarray:
        pass

    def choose_starting_action(self) -> np.ndarray:
        starting_action = self._choose_starting_action()
        cliped_action = np.clip(starting_action, self.action_min, self.action_max)
        return cliped_action
