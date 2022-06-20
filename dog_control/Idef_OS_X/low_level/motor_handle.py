from bus import CANBus
import dataclasses
import pickle
import json
from pathlib import Path


MOTOR_OFFSET_PATH = Path("/home/pi/workspace/dog_control/data/motor_offset.json")
MOTOR_IDS = [i + 1 for i in range(12)]


@dataclasses.dataclass
class MotorOffsets:
    rest_offset: dict[int, float]
    zero_offset: dict[int, float]


class MotorHandle:
    def __init__(self, bus: CANBus):
        self.bus = bus
        self.motor_offset = self.load_motor_offset()

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
        with open(MOTOR_OFFSET_PATH) as f:
            self.motor_offset = pickle.load(f)

        self.check_offset_list(self.motor_offset.rest_offset)
        self.check_offset_list(self.motor_offset.zero_offset)

    def save_motor_offset(self):
        with open(MOTOR_OFFSET_PATH) as f:
            pickle.dump(self.motor_offset, f)

    def load_default_motor_offset(self):
        zero_offset = {motor_id: 0 for motor_id in MOTOR_IDS}
        self.motor_offset = MotorOffsets(zero_offset, zero_offset)

    def disengage(self):
        for motor_id in MOTOR_IDS:
            self.bus.turn_motor_off(motor_id)

    def set_lite_pid(self):
        for motor_id in MOTOR_IDS:
            self.bus.set_pid(motor_id, 15, 0, 35, 0, 50, 50)

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

    def go_to(self, all_target_pos):
        for motor_id, target_pos in zip(MOTOR_IDS, all_target_pos):
            self.bus.position_control(motor_id, target_pos)