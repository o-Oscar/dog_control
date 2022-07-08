import numpy as np
from dog_control.controllers.jaccobian import (
    LEFT_LEG_POSITIONS,
    LEG_FIRST_SITES,
    LEG_MOTOR_AXES,
    RIGHT_LEG_POSITIONS,
    calc_jac,
)
from dog_control.controllers.logger import Logger
from scipy.spatial.transform import Rotation as R


class VelocityEstimator:
    def __init__(self, logger: Logger):
        self.logger = logger

        self.logger.add_entry("motor_velocity", 12)
        self.logger.add_entry("foot_velocity", 12)
        self.logger.add_entry("base_velocity_local", 12)
        self.logger.add_entry("base_velocity_global", 12)
        self.logger.add_entry("foot_position", 12)
        self.dt = 1 / 30

        self.has_been_called = False

        self.previous_motor_position = np.zeros((12,))
        self.all_motor_velocity = np.zeros((12,))
        self.body_rotation = R.identity()
        self.all_foot_position = np.zeros((12,))
        self.all_foot_velocity = np.zeros((12,))
        self.all_base_velocity_local = np.zeros((12,))
        self.all_base_velocity_global = np.zeros((12,))

    def estimate(
        self,
        cur_motor_positions: np.ndarray,
        up_vector: np.ndarray,
        rotation_speed: np.ndarray,
    ) -> None:
        """
        Wrapper around update_estimation. Prevents computations using non-initialised motor previous positions.
        """
        if self.has_been_called:
            self.update_estimations(cur_motor_positions, up_vector, rotation_speed)
            self.log()

        self.previous_motor_position = cur_motor_positions.copy()
        self.has_been_called = True

    def update_estimations(
        self,
        cur_motor_positions: np.ndarray,
        up_vector: np.ndarray,
        rotation_speed: np.ndarray,
    ) -> None:
        """Carries out the velocity estimations."""

        # Computes the motor velocities using simple finite difference
        self.all_motor_velocity = (
            cur_motor_positions - self.previous_motor_position
        ) / self.dt

        # Computes the main body rotation
        self.body_rotation = self.calc_body_rotation(up_vector)

        for leg_id in range(4):
            lid, hid = leg_id * 3, leg_id * 3 + 3

            # Computes a base velocity estimate by assuming that the current point foot
            # is fixed to the ground (but can still rotate)
            motor_positions = cur_motor_positions[lid:hid]
            motor_velocity = self.all_motor_velocity[lid:hid]
            foot_position, jac = calc_jac(
                LEG_MOTOR_AXES,
                motor_positions,
                LEG_FIRST_SITES[leg_id],
                LEFT_LEG_POSITIONS if leg_id % 2 == 0 else RIGHT_LEG_POSITIONS,
            )
            foot_velocity = (motor_velocity.reshape((1, 3)) @ jac).flatten()
            base_velocity_local = (
                np.cross(foot_position, rotation_speed) - foot_velocity
            )
            base_velocity_global = self.body_rotation.apply(base_velocity_local)

            # Save the result of the calculations in object properties
            self.all_foot_position[lid:hid] = foot_position
            self.all_foot_velocity[lid:hid] = foot_velocity
            self.all_base_velocity_local[lid:hid] = base_velocity_local
            self.all_base_velocity_global[lid:hid] = base_velocity_global

    def calc_body_rotation(self, up_vector: np.ndarray) -> R:
        """Computes the current simplest body rotation to align the up vector."""
        cross = np.cross(up_vector, np.array([0, 0, 1]))
        cross = cross / np.linalg.norm(cross)
        angle = np.arccos(up_vector[2])
        return R.from_rotvec(cross * angle)

    def log(self):
        self.logger["motor_velocity"] = self.all_motor_velocity
        self.logger["foot_position"] = self.all_foot_position
        self.logger["foot_velocity"] = self.all_foot_velocity
        self.logger["base_velocity_local"] = self.all_base_velocity_local
        self.logger["base_velocity_global"] = self.all_base_velocity_global
