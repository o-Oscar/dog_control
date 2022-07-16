import numpy as np
from scipy.spatial.transform import Rotation as R

LEG_MOTOR_AXES = [
    np.array([1, 0, 0]),
    np.array([0, 1, 0]),
    np.array([0, 1, 0]),
]
FIRST_SITE_DX = 0.225
FIRST_SITE_DY = 0.0525
SHOULDER_DY = 0.06875
LEFT_LEG_POSITIONS = [
    np.array([0, SHOULDER_DY, 0]),
    np.array([0, 0, -0.2]),
    np.array([0, 0, -0.2]),
]
RIGHT_LEG_POSITIONS = [
    np.array([0, -SHOULDER_DY, 0]),
    np.array([0, 0, -0.2]),
    np.array([0, 0, -0.2]),
]
LEG_FIRST_SITES = [
    np.array([FIRST_SITE_DX, FIRST_SITE_DY, 0]),
    np.array([FIRST_SITE_DX, -FIRST_SITE_DY, 0]),
    np.array([-FIRST_SITE_DX, FIRST_SITE_DY, 0]),
    np.array([-FIRST_SITE_DX, -FIRST_SITE_DY, 0]),
]

gear_ratio = np.array([1, 1, 24 / 30] * 4)
base_kp = 100
base_kd = 10
base_max_torque = 8
KP = base_kp * gear_ratio
KD = base_kd * gear_ratio
MAX_TORQUE = base_max_torque * gear_ratio


def calc_body_rotation(up_vector: np.ndarray) -> R:
    """Computes the current simplest body rotation to align the up vector."""
    cross = np.cross(up_vector, np.array([0, 0, 1]))
    cross = cross / np.linalg.norm(cross)
    angle = np.arccos(up_vector[2])
    return R.from_rotvec(cross * angle)


def calc_all_rots(axes, rotations):
    all_rots = [R.identity()]
    for ax, rotation in zip(axes, rotations):
        all_rots.append(all_rots[-1] * R.from_rotvec(ax * rotation))
    return all_rots


def calc_all_sites(all_rots, first_site, positions):
    all_sites = [first_site]
    for rot, position in zip(all_rots[1:], positions):
        all_sites.append(all_sites[-1] + rot.apply(position))
    all_sites = np.stack(all_sites, axis=0)

    return all_sites


def calc_jac(axes, all_rots, all_deltas):
    all_jacs = []
    for rot, ax, delta in zip(all_rots[:-1], axes, all_deltas):
        all_jacs.append(np.cross(delta, rot.apply(ax)))
    all_jacs = np.stack(all_jacs, axis=0)

    return all_jacs


def skew(x):
    return np.array(
        [
            [0, -x[2], x[1]],
            [x[2], 0, -x[0]],
            [-x[1], x[0], 0],
        ]
    )


def skew_matrix(all_deltas):
    return np.sum([skew(delta) for delta in all_deltas], axis=0)


def torque_to_force(jac, leg_torques):
    x, _, _, s = np.linalg.lstsq(jac, leg_torques)  # not transpose (I am a thug)
    min_singular = np.min(np.square(s))
    return min_singular < 1e-5, x
