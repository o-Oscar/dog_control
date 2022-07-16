import re

import numpy as np

# from dog_control.controllers.dummy import Controller
# from dog_control.controllers.impedance_control import Controller
from dog_control.controllers.switcher import Controller

# from dog_control.controllers.positive_twitch import Controller
# from dog_control.controllers.step_moves import Controller
# from dog_control.controllers.vertical_feet import Controller
from dog_control.simulation.engine import Engine, EngineConfig
from dog_control.simulation.IdefX import IdefX
from scipy.spatial.transform import Rotation as R

# from dog_control.controllers.force_control import Controller


# from dog_control.controllers.position_reacher import Controller


# from dog_control.controllers.position_reacher import Controller


def constant_force(frame):
    to_return = np.zeros((19, 6))
    # to_return[12, 0] = 10 if frame < 30 else 0
    to_return[6, 5] = 3
    return to_return


def rock_front_and_side(frame):
    rx, ry = 0, 0
    T = 300
    amplitude = np.pi / 16
    if frame < T:
        ry = amplitude * np.sin(frame / T * np.pi) ** 2
    else:
        rx = -amplitude * np.sin(frame / T * np.pi) ** 2
    rot = R.from_euler("xyz", [rx, ry, 0])
    scipy_quat = rot.as_quat()
    target_rot = [scipy_quat[-1]] + list(scipy_quat[:-1])
    return np.array(target_rot)


def bank_angle(frame):
    rot = R.from_euler("xyz", [np.pi / 8, 0, 0])
    scipy_quat = rot.as_quat()
    target_rot = [scipy_quat[-1]] + list(scipy_quat[:-1])
    return np.array(target_rot)


def sine(frame, period, amplitude):
    return amplitude * np.sin(frame / period / 30 * np.pi * 2)


def mocap_rot_callback(frame):
    to_return = np.zeros((5, 4))
    to_return[0] = rock_front_and_side(frame)
    # to_return[0] = bank_angle(frame)
    return to_return


def mocap_rot_callback(frame):
    rot = R.from_euler("xyz", [0, 0, 0])
    scipy_quat = rot.as_quat()
    target_rot = [scipy_quat[-1]] + list(scipy_quat[:-1])

    to_return = np.zeros((5, 4))
    to_return[0] = np.array(target_rot)
    # to_return[0] = bank_angle(frame)
    return to_return


def mocap_pos_callback(frame):
    to_return = np.zeros((5, 3))

    # base_pos = np.array([sine(frame, 3, 0.2), 0, 0.7])
    base_pos = np.array([0, 0, 0.4])

    foot_pos = np.array([0, 0, 0])

    to_return[0] = base_pos
    to_return[1] = foot_pos
    to_return[2] = foot_pos
    to_return[3] = foot_pos
    to_return[4] = foot_pos

    return to_return


def mocap_activation(frame):
    to_return = np.zeros((5,))
    to_return[0] = 1 if frame < 6 else 0
    return to_return


def main():

    realtime = True

    # create the objects to control IdefX.
    engine_config = EngineConfig(
        fix_root=True,
        fix_feets=True,
        use_viewer=realtime,
        force_callback=constant_force,
        # mocap_activation=None,  # for in the air tests
        # mocap_pos_callback=mocap_pos_callback,
        # mocap_rot_callback=mocap_rot_callback,
        mocap_activation=mocap_activation,  # for tests with the ground
        mocap_pos_callback=mocap_pos_callback,
        # mocap_rot_callback=mocap_rot_callback,
    )

    idefX = IdefX(
        controller=Controller(),
        simulation_engine=Engine(engine_config),
        realtime=realtime,
    )

    # use the controller
    idefX.live()


if __name__ == "__main__":
    main()
