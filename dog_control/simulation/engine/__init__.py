import abc
import dataclasses
from ast import Call
from pathlib import Path
from typing import Callable

import dog_control.controllers.estimator.geometry as geom
import mujoco_py
import numpy as np
from dog_control.simulation.engine.create_mjcf import (
    DEFAULT_SRC_PATH,
    write_robot_to_file,
)
from scipy.spatial.transform import Rotation as R


class BaseEngine(abc.ABC):
    def __init__(self):
        pass

    @abc.abstractmethod
    def step(self, frame, action):
        pass

    @abc.abstractmethod
    def set_motor_position(self):
        pass

    @abc.abstractmethod
    def get_motor_position(self):
        pass

    @abc.abstractmethod
    def get_up_vector(self):
        pass

    @abc.abstractmethod
    def get_rotation_speed(self):
        pass


@dataclasses.dataclass
class EngineConfig:
    fix_root: bool = False
    fix_feets: bool = False
    use_viewer: bool = True
    force_callback: Callable = None
    mocap_activation: Callable = None
    mocap_pos_callback: Callable = None
    mocap_rot_callback: Callable = None


# https://github.com/openai/mujoco-py/tree/master/examples
# https://mujoco.readthedocs.io/en/latest/python.html


class Engine(BaseEngine):
    def __init__(self, config: EngineConfig):

        self.config = config

        self.substeps = 10

        write_robot_to_file(
            fix_root=self.config.fix_root,
            fix_feets=self.config.fix_feets,
            substeps=self.substeps,
            base_motor_kp=0,
            base_motor_kd=0,
        )

        model = mujoco_py.load_model_from_path(str(DEFAULT_SRC_PATH))
        self.sim = mujoco_py.MjSim(model)

        if self.config.use_viewer:
            self.viewer = mujoco_py.MjViewer(self.sim)
            self.viewer.render()

        self.delta_q = np.zeros((12,))
        # self.delta_q = np.array([0.05, 0, 0] * 4)

    def step(self, frame, action):

        if self.config.mocap_activation is not None:
            # self.sim.data.eq_active[:] = self.config.mocap_activation(frame)
            self.sim.model.eq_active[:] = self.config.mocap_activation(frame)

        if self.config.mocap_pos_callback is not None:
            self.sim.data.mocap_pos[:] = self.config.mocap_pos_callback(frame)

        if self.config.mocap_rot_callback is not None:
            self.sim.data.mocap_quat[:] = self.config.mocap_rot_callback(frame)

        for i in range(10):
            leg_pd_torque = self.compute_pd_torque(action)
            if self.config.force_callback is not None:
                self.sim.data.xfrc_applied[:] = self.config.force_callback(frame)
            self.sim.data.qfrc_applied[6:] = leg_pd_torque
            self.sim.step()

        if self.config.use_viewer:
            self.viewer.render()

    def set_motor_position(self, motor_positions):
        self.sim.data.qpos[7:] = motor_positions

    def get_motor_position(self):
        to_return = self.sim.data.qpos[7:]
        return to_return

    def compute_pd_torque(self, target_q):
        q = self.sim.data.qpos[7:]
        q_dot = self.sim.data.qvel[6:]

        torque = -geom.KP * (q - target_q - self.delta_q) - geom.KD * q_dot
        torque = np.clip(torque, -geom.MAX_TORQUE, geom.MAX_TORQUE)
        return torque

    def get_up_vector(self):
        body_rotation = R.from_quat(
            list(self.sim.data.qpos[4:7]) + [self.sim.data.qpos[3]]
        )
        return body_rotation.inv().apply(np.array([0, 0, 1]))

    def get_rotation_speed(self):
        body_rotation = R.from_quat(
            list(self.sim.data.qpos[4:7]) + [self.sim.data.qpos[3]]
        )
        return body_rotation.inv().apply(self.sim.data.qvel[3:6])
