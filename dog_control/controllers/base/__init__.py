import abc
import dataclasses
import time

import numpy as np


class BaseController(abc.ABC):
    def __init__(self):
        pass

    @abc.abstractmethod
    def _choose_action(self) -> np.ndarray:
        pass

    def choose_action(self) -> np.ndarray:
        return self._choose_action()  # TODO : implement target position limitation here
