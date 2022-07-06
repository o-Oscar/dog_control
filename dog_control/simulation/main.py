import re

import numpy as np
from dog_control.controllers.dummy import DummyController
from dog_control.controllers.impedance_control import ImpedanceController
from dog_control.controllers.position_reacher import PositionReacherController
from dog_control.controllers.positive_twitch import PositivTwitchController
from dog_control.controllers.step_moves import StepMoveController
from dog_control.simulation.engine import Engine, EngineConfig
from dog_control.simulation.IdefX import IdefX
from scipy.spatial.transform import Rotation as R


def constant_force():
    to_return = np.zeros((14, 6))
    to_return[7, 0] = 100
    return to_return


def rock_front_and_back(frame):
    rx, ry = 0, 0
    if frame < 500:
        ry = np.pi / 4 * np.sin(frame / 500 * np.pi) ** 2
    else:
        rx = -np.pi / 4 * np.sin(frame / 500 * np.pi) ** 2
    rot = R.from_euler("xyz", [rx, ry, 0])
    scipy_quat = rot.as_quat()
    target_rot = [scipy_quat[-1]] + list(scipy_quat[:-1])
    return np.array(target_rot)


def main():

    realtime = False

    # create the objects to control IdefX.
    engine_config = EngineConfig(
        fix_root=True,
        use_viewer=realtime,
        base_motor_kp=10,
        base_motor_kd=20,
        maximum_torque=11000000,
        force_callback=None,
        target_base_rot_callback=rock_front_and_back,
    )

    idefX = IdefX(
        controller=ImpedanceController(),
        simulation_engine=Engine(engine_config),
        realtime=realtime,
    )

    # use the controller
    idefX.live()


if __name__ == "__main__":
    main()
