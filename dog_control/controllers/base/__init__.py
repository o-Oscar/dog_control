import abc
import dataclasses
import numpy as np
import time



class BaseController(abc.ABC):
    def __init__(self):
        pass

    @abc.abstractmethod
    def _choose_action(self, state: RobotState) -> np.ndarray:
        pass

    def choose_action(self, state: RobotState) -> np.ndarray:
        return self._choose_action(state)  # TODO : implement target position limitation here
