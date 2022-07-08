import re

import numpy as np

# from dog_control.controllers.dummy import Controller
# from dog_control.controllers.impedance_control import Controller
from dog_control.controllers.position_reacher import Controller

# from dog_control.controllers.positive_twitch import Controller
# from dog_control.controllers.step_moves import Controller
# from dog_control.controllers.vertical_feet import Controller
from dog_control.simulation.engine import Engine, EngineConfig
from dog_control.simulation.IdefX import IdefX
from scipy.spatial.transform import Rotation as R

# from dog_control.controllers.position_reacher import Controller


def constant_force():
    to_return = np.zeros((14, 6))
    to_return[7, 0] = 100
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


def mocap_pos_callback(frame):
    to_return = np.zeros((5, 3))

    base_pos = np.array([sine(frame, 3, 0.2), 0, 0.7])

    foot_pos = np.array([0, 0, 0.1])

    to_return[0] = base_pos
    to_return[1] = foot_pos
    to_return[2] = foot_pos
    to_return[3] = foot_pos
    to_return[4] = foot_pos

    return to_return


def main():

    realtime = True

    # create the objects to control IdefX.
    engine_config = EngineConfig(
        fix_root=True,
        fix_feets=True,
        use_viewer=realtime,
        base_motor_kp=10,
        base_motor_kd=20,
        maximum_torque=11000000,
        force_callback=None,
        mocap_pos_callback=mocap_pos_callback,
        mocap_rot_callback=mocap_rot_callback,
    )

    idefX = IdefX(
        controller=Controller(),
        simulation_engine=Engine(engine_config),
        realtime=realtime,
    )

    # use the controller
    idefX.live()


if __name__ == "__main__":
    # main()
    # x = a sin(frame / period / 30 * np.pi * 2)
    print(0.2 * np.pi * 2 / 3)
