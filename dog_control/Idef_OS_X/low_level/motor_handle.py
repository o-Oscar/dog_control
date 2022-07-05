import dataclasses
import json
import pickle
from pathlib import Path
from typing import Any

import numpy as np
from dog_control.Idef_OS_X.low_level.bus import CANBus

MOTOR_OFFSET_PATH = Path("/home/pi/workspace/dog_control/data/motor_offset.json")
MOTOR_IDS = [i + 1 for i in range(12)]

GEAR_FACTOR = np.ones((12,))
GEAR_FACTOR[1 - 1] = -1
GEAR_FACTOR[2 - 1] = -1
GEAR_FACTOR[4 - 1] = -1
GEAR_FACTOR[6 - 1] = -1
GEAR_FACTOR[8 - 1] = -1
GEAR_FACTOR[12 - 1] = -1

KNEE_GEAR_RATIO = 24 / 30
GEAR_FACTOR[3 - 1] *= KNEE_GEAR_RATIO
GEAR_FACTOR[6 - 1] *= KNEE_GEAR_RATIO
GEAR_FACTOR[9 - 1] *= KNEE_GEAR_RATIO
GEAR_FACTOR[12 - 1] *= KNEE_GEAR_RATIO


ACTUATOR_GEAR_RATIO = 6
ACTUATOR_POS_MODULO = 2 * np.pi / ACTUATOR_GEAR_RATIO

# motor dirrection to change :
# 1 2 4 6 8 12


@dataclasses.dataclass
class MotorOffsets:
    rest_offset: Any
    zero_offset: Any


class MotorHandle:
    def __init__(self, bus: CANBus):
        self.bus = bus

        self.load_motor_offset()
        # self.load_default_motor_offset() # There in case no offset are already created

    def check_offset_list(self, to_check):
        if type(to_check) != dict:
            raise TypeError(
                "MotorOffset member is of type {}. It is supposed to be of type list.".format(
                    type(to_check)
                )
            )
        if set(to_check.keys()) != set(MOTOR_IDS):
            raise TypeError(
                "MotorOffset member contains {}. It is supposed to contain {}.".format(
                    to_check, MOTOR_IDS
                )
            )

    def load_motor_offset(self) -> MotorOffsets:
        if not MOTOR_OFFSET_PATH.is_file():
            raise FileExistsError(
                "No motor_offset file found at : {}".format(MOTOR_OFFSET_PATH)
            )
        with open(MOTOR_OFFSET_PATH, "rb") as f:
            self.motor_offset = pickle.load(f)

        self.check_offset_list(self.motor_offset.rest_offset)
        self.check_offset_list(self.motor_offset.zero_offset)

    def save_motor_offset(self):
        with open(MOTOR_OFFSET_PATH, "wb") as f:
            pickle.dump(self.motor_offset, f)

    def load_default_motor_offset(self):
        rest_offset = {motor_id: 0 for motor_id in MOTOR_IDS}
        zero_offset = {motor_id: 0 for motor_id in MOTOR_IDS}
        self.motor_offset = MotorOffsets(rest_offset, zero_offset)

    def check_startable(self, motors_ids_to_check=None):
        if motors_ids_to_check is None:
            motors_ids_to_check = MOTOR_IDS

        for motor_id in motors_ids_to_check:
            start_position = self.bus.get_actuator_pos(motor_id)
            delta = 0
            while (
                self.motor_offset.rest_offset[motor_id] + delta
                < start_position - ACTUATOR_POS_MODULO / 2
            ):
                delta += ACTUATOR_POS_MODULO
            while (
                self.motor_offset.rest_offset[motor_id] + delta
                > start_position + ACTUATOR_POS_MODULO / 2
            ):
                delta -= ACTUATOR_POS_MODULO

            self.motor_offset.rest_offset[motor_id] += delta
            self.motor_offset.zero_offset[motor_id] += delta

            if abs(
                start_position - self.motor_offset.rest_offset[motor_id]
            ) > np.radians(10):
                print("Info at startup of motor", motor_id, ":")
                print(
                    "delta : {:.2f} rad or {:.2f} deg".format(delta, np.degrees(delta))
                )
                print(
                    "start position : {:.2f} rad or {:.2f} deg".format(
                        start_position, np.degrees(start_position)
                    )
                )
                print(
                    "rest position : {:.2f} rad or {:.2f} deg".format(
                        self.motor_offset.rest_offset[motor_id],
                        np.degrees(self.motor_offset.rest_offset[motor_id]),
                    )
                )
                print(
                    "difference : {:.2f} rad or {:.2f} deg".format(
                        start_position - self.motor_offset.rest_offset[motor_id],
                        np.degrees(
                            start_position - self.motor_offset.rest_offset[motor_id]
                        ),
                    )
                )
                raise NameError(
                    "Motor {} too far from its default position. Please put the entire dog in its rest position.".format(
                        motor_id
                    )
                )

    def disengage(self):
        for motor_id in MOTOR_IDS:
            self.bus.turn_motor_off(motor_id)

    def set_lite_pid(self):
        for motor_id in MOTOR_IDS:
            # self.bus.set_pid(motor_id, 15, 0, 35, 0, 50, 50) # kinda strong though
            self.bus.set_pid(motor_id, 10, 0, 20, 0, 50, 50) # makeing this one work would be great

    def set_strong_pid(self):
        for motor_id in MOTOR_IDS:
            self.bus.set_pid(motor_id, 4 * 2, 0, 50, 40, 50, 50)

    def go_to_zero(self, dt=1):
        for motor_id in MOTOR_IDS:
            cur_pos = self.bus.get_actuator_pos(motor_id)
            target_pos = self.motor_offset.zero_offset[motor_id]
            speed = abs(cur_pos - target_pos) / dt
            self.bus.position_control_at_speed(motor_id, target_pos, speed)

    def go_to_rest(self, dt=1):
        for motor_id in MOTOR_IDS:
            cur_pos = self.bus.get_actuator_pos(motor_id)
            target_pos = self.motor_offset.rest_offset[motor_id]
            speed = abs(cur_pos - target_pos) / dt
            self.bus.position_control_at_speed(motor_id, target_pos, speed)

    def go_to_slow_raw(self, all_target_pos, dt=1):
        for motor_id, target_pos in zip(MOTOR_IDS, all_target_pos):
            cur_pos = self.bus.get_actuator_pos(motor_id)
            speed = abs(cur_pos - target_pos) / dt
            self.bus.position_control_at_speed(motor_id, target_pos, speed)

    def go_to_raw(self, all_target_pos):
        for motor_id, target_pos in zip(MOTOR_IDS, all_target_pos):
            self.bus.position_control(motor_id, target_pos)

    def go_to_slow(self, all_target_pos, dt=1):
        offsets = np.array([self.motor_offset.zero_offset[id] for id in MOTOR_IDS])
        raw_target_pos = all_target_pos * GEAR_FACTOR + offsets
        self.go_to_slow_raw(raw_target_pos, dt)

    def go_to(self, all_target_pos):
        offsets = np.array([self.motor_offset.zero_offset[id] for id in MOTOR_IDS])
        raw_target_pos = all_target_pos * GEAR_FACTOR + offsets
        self.go_to_raw(raw_target_pos)

    def get_raw_motor_pos(self):
        to_return = []
        for motor_id in MOTOR_IDS:
            to_return.append(self.bus.get_actuator_pos(motor_id))
        return np.array(to_return)

    def get_motor_pos(self):
        offsets = np.array([self.motor_offset.zero_offset[id] for id in MOTOR_IDS])
        return (self.get_raw_motor_pos() - offsets) / GEAR_FACTOR
