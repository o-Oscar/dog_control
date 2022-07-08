import numpy as np
from scipy.spatial.transform.rotation import Rotation as R

LEG_MOTOR_AXES = [
    np.array([1, 0, 0]),
    np.array([0, 1, 0]),
    np.array([0, 1, 0]),
]
# LEFT_LEG_POSITIONS = [
#     np.array([0, 0.06875, 0]),
#     np.array([0, 0, -0.2]),
#     np.array([0, 0, -0.2]),
# ]
LEFT_LEG_POSITIONS = [
    np.array([0, 0.06875, 0]),
    np.array([0, 0, -0.2]),
    np.array([0, 0, -0.2]),
]
RIGHT_LEG_POSITIONS = [
    np.array([0, -0.06875, 0]),
    np.array([0, 0, -0.2]),
    np.array([0, 0, -0.2]),
]
FIRST_SITE_DX = 0.225
FIRST_SITE_DY = 0.0525
LEG_FIRST_SITES = [
    np.array([FIRST_SITE_DX, FIRST_SITE_DY, 0]),
    np.array([FIRST_SITE_DX, -FIRST_SITE_DY, 0]),
    np.array([-FIRST_SITE_DX, FIRST_SITE_DY, 0]),
    np.array([-FIRST_SITE_DX, -FIRST_SITE_DY, 0]),
]


def calc_jac(axes, rotations, first_site, positions):
    all_rots = [R.identity()]
    for ax, rotation in zip(axes, rotations):
        all_rots.append(all_rots[-1] * R.from_rotvec(ax * rotation))

    all_sites = [first_site]
    for rot, position in zip(all_rots[1:], positions):
        all_sites.append(all_sites[-1] + rot.apply(position))
    all_sites = np.stack(all_sites, axis=0)
    all_deltas = all_sites[:-1] - all_sites[-1]

    all_jacs = []
    for rot, ax, delta in zip(all_rots[:-1], axes, all_deltas):
        all_jacs.append(np.cross(delta, rot.apply(ax)))
    all_jacs = np.stack(all_jacs, axis=0)

    return all_sites[-1], all_jacs
