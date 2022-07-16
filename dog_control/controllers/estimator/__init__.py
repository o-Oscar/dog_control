import dog_control.controllers.estimator.geometry as geom
import numpy as np
from dog_control.controllers.logger import Logger
from scipy.spatial.transform import Rotation as R


class Estimator:
    def __init__(self, logger: Logger):
        self.logger = logger

        self.logger.add_entry("motor_velocity", 12)
        self.logger.add_entry("foot_velocity", 12)
        self.logger.add_entry("base_velocity_local", 12)
        self.logger.add_entry("base_velocity_global", 12)
        self.logger.add_entry("foot_position", 12)
        self.logger.add_entry("foot_position_global", 12)
        self.logger.add_entry("motor_torque", 12)
        self.logger.add_entry("force_flag", 4)
        self.logger.add_entry("foot_force_local", 12)
        self.logger.add_entry("foot_force_global", 12)
        self.dt = 1 / 30

        self.has_been_called = False

        self.previous_motor_position = np.zeros((12,))
        self.all_motor_velocity = np.zeros((12,))
        self.body_rotation = R.identity()
        self.all_foot_position = np.zeros((12,))
        self.all_foot_position_global = np.zeros((12,))
        self.all_foot_velocity = np.zeros((12,))
        self.all_base_velocity_local = np.zeros((12,))
        self.all_base_velocity_global = np.zeros((12,))
        self.all_motor_torque = np.zeros((12,))
        self.all_force_flag = np.zeros((4,))
        self.all_foot_force_local = np.zeros((12,))
        self.all_foot_force_global = np.zeros((12,))
        self.all_jacs = np.zeros((4, 3, 3))

    def estimate(
        self,
        last_target_motor_positions: np.ndarray,
        cur_motor_positions: np.ndarray,
        up_vector: np.ndarray,
        rotation_speed: np.ndarray,
    ) -> None:
        """
        Wrapper around update_estimation. Prevents computations using non-initialised motor previous positions.
        """
        if self.has_been_called:
            self.update_estimations(
                last_target_motor_positions,
                cur_motor_positions,
                up_vector,
                rotation_speed,
            )
            self.log()

        self.previous_motor_position = cur_motor_positions.copy()
        self.has_been_called = True

    def update_estimations(
        self,
        last_target_motor_positions: np.ndarray,
        cur_motor_positions: np.ndarray,
        up_vector: np.ndarray,
        rotation_speed: np.ndarray,
    ) -> None:
        """Carries out the velocity estimations."""

        # motor velocities computation using simple finite difference
        self.all_motor_velocity = (
            cur_motor_positions - self.previous_motor_position
        ) / self.dt

        # main body rotation computation
        self.body_rotation = geom.calc_body_rotation(up_vector)

        # torque computation
        torque = (
            -geom.KP * (cur_motor_positions - last_target_motor_positions)
            - geom.KD * self.all_motor_velocity
        )
        self.all_motor_torque = np.clip(torque, -geom.MAX_TORQUE, geom.MAX_TORQUE)

        for leg_id in range(4):
            lid, hid = leg_id * 3, leg_id * 3 + 3

            # leg properties gathering
            motor_positions = cur_motor_positions[lid:hid]
            motor_velocity = self.all_motor_velocity[lid:hid]
            leg_torques = self.all_motor_torque[lid:hid]
            joint_axes = geom.LEG_MOTOR_AXES
            first_site = geom.LEG_FIRST_SITES[leg_id]
            leg_positions = (
                geom.LEFT_LEG_POSITIONS if leg_id % 2 == 0 else geom.RIGHT_LEG_POSITIONS
            )

            # forward kinematics
            all_rots = geom.calc_all_rots(joint_axes, motor_positions)
            all_sites = geom.calc_all_sites(all_rots, first_site, leg_positions)
            foot_position = all_sites[-1]
            foot_position_global = self.body_rotation.apply(foot_position)
            all_deltas = all_sites[:-1] - foot_position

            # velocity computation
            jac = geom.calc_jac(joint_axes, all_rots, all_deltas)
            foot_velocity = (motor_velocity.reshape((1, 3)) @ jac).flatten()
            base_velocity_local = (
                np.cross(foot_position, rotation_speed) - foot_velocity
            )
            base_velocity_global = self.body_rotation.apply(base_velocity_local)

            # force applied by foot computation
            flag, foot_force_local = geom.torque_to_force(jac, leg_torques)
            foot_force_global = self.body_rotation.apply(foot_force_local)
            # foot_force_local = (
            #     leg_torques.reshape((1, 3)) @ np.linalg.inv(jac.T)
            # ).flatten()
            # flag = 1
            # foot_force_global = self.body_rotation.apply(foot_force_local)

            # Save the results
            self.all_foot_position[lid:hid] = foot_position
            self.all_foot_position_global[lid:hid] = foot_position_global
            self.all_foot_velocity[lid:hid] = foot_velocity
            self.all_base_velocity_local[lid:hid] = base_velocity_local
            self.all_base_velocity_global[lid:hid] = base_velocity_global
            self.all_force_flag[leg_id] = flag
            self.all_foot_force_local[lid:hid] = foot_force_local
            self.all_foot_force_global[lid:hid] = foot_force_global
            self.all_jacs[leg_id] = jac

    def log(self):
        self.logger["motor_velocity"] = self.all_motor_velocity
        self.logger["foot_position"] = self.all_foot_position
        self.logger["foot_velocity"] = self.all_foot_velocity
        self.logger["base_velocity_local"] = self.all_base_velocity_local
        self.logger["base_velocity_global"] = self.all_base_velocity_global
        self.logger["motor_torque"] = self.all_motor_torque
        self.logger["force_flag"] = self.all_force_flag
        self.logger["foot_force_local"] = self.all_foot_force_local
        self.logger["foot_force_global"] = self.all_foot_force_global
