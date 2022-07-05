import abc
import dataclasses
from pathlib import Path

import mujoco_py
import numpy as np
from dog_control.simulation.engine.create_mjcf import (DEFAULT_SRC_PATH,
                                                       write_robot_to_file)


class BaseEngine(abc.ABC):
    def __init__(self):
        pass

    @abc.abstractmethod
    def step(self, action):
        pass


@dataclasses.dataclass
class EngineConfig:
    fix_root: bool = False
    use_viewer: bool = True


# https://github.com/openai/mujoco-py/tree/master/examples
# https://mujoco.readthedocs.io/en/latest/python.html


class Engine(BaseEngine):
    def __init__(self, config: EngineConfig):

        self.config = config

        self.substeps = 10

        write_robot_to_file(fix_root=self.config.fix_root, substeps=self.substeps, base_motor_kp =0, base_motor_kd=0)

        model = mujoco_py.load_model_from_path(str(DEFAULT_SRC_PATH))
        self.sim = mujoco_py.MjSim(model)

        if self.config.use_viewer:
            self.viewer = mujoco_py.MjViewer(self.sim)
            self.viewer.render()

        self.kp = 200
        self.kd = 30
        self.maximum_torque = 100

    def step(self, action):
        # print("updating engine")

        # self.sim.data.ctrl[:] = action[:]
        for i in range(10):
            leg_pd_torque = self.compute_pd_torque (action)
            self.siom.data.qfrc_applied[6:] = leg_pd_torque
            self.sim.step()

        if self.config.use_viewer:
            self.viewer.render()

    def get_motor_position(self):
        to_return = self.sim.data.qpos[7:]
        return to_return

    def compute_pd_torque (self, target_q):
        q = self.sim.data.qpos[7:]
        q_dot = self.sim.data.qvel[6:]

        torque = -(q -target_q) * self.kd - q_dot * self.kd
        torque = np.clip(torque, -self.maximum_torque, self.maximum_torque)
        return torque
