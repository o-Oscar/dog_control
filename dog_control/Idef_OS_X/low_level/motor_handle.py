from bus import CANBus
import dataclasses
import pickle
import json
from pathlib import Path


MOTOR_OFFSET_PATH = Path("/home/pi/workspace/dog_control/data/motor_offset.json")
MOTOR_IDS = [i+1 for i in range(12)]


@dataclasses.dataclass
class MotorOffsets:
    rest_offset: list
    zero_offset: list

class MotorHandle:
    def __init__(self, bus: CANBus):
        self.bus = bus
        self.motor_offset = self.load_motor_offset()

    def check_offset_list(self, to_check):
        if type(to_check) != list:
            raise TypeError("MotorOffset member is of type {}. It is supposed to be of type list.".format(type(to_check)))
        if len(to_check) != MOTOR_NB:
            raise TypeError("MotorOffset member is of length {}. It is supposed to be of length {}.".format(len(to_check), MOTOR_NB))
    

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
        self.motor_offset = MotorOffsets([0]*MOTOR_NB, [0]*MOTOR_NB)
