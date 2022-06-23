import abc
import dataclasses
from pathlib import Path

import mujoco_py
import numpy as np
from dog_control.simulation.engine.create_mjcf import (
    DEFAULT_SRC_PATH,
    write_robot_to_file,
)


class BaseEngine(abc.ABC):
    def __init__(self):
        pass

    @abc.abstractmethod
    def step(self, action):
        pass


@dataclasses.dataclass
class EngineConfig:
    fix_root: bool = False


# https://github.com/openai/mujoco-py/tree/master/examples
# https://mujoco.readthedocs.io/en/latest/python.html


class Engine(BaseEngine):
    def __init__(self, config: EngineConfig):

        self.config = config

        write_robot_to_file(fix_root=self.config.fix_root)

        model = mujoco_py.load_model_from_path(str(DEFAULT_SRC_PATH))
        self.sim = mujoco_py.MjSim(model)
        self.viewer = mujoco_py.MjViewer(self.sim)
        self.viewer.render()

    def step(self, action):
        print("updating engine")

        self.sim.data.ctrl[:] = action[:]
        # self.sim.data.ctrl[4] = np.pi / 4  #  * action[1]
        print(self.sim.data.ctrl[:])

        self.sim.step()
        self.viewer.render()
