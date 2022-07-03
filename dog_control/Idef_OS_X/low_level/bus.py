import dataclasses
import struct

import can
import numpy as np

CHANNEL = "can0"
# BUSTYPE = "socketcan_native"
BUSTYPE = "socketcan"

DEFAULT_MOTOR_SPEED = 10 * 360 * 6


@dataclasses.dataclass
class ActuatorInfos:
    temperature: float
    speed: float
    torque: float
    encoder: float


class CANBus:
    def __init__(self):
        self.bus = None

    def connect(self):
        if self.bus is not None:
            raise NameError("Bus already connected. Can not recreate a connection.")
        print("Creating can bus image.")
        self.bus = can.interface.Bus(channel=CHANNEL, bustype=BUSTYPE)

    def send(self, motor_id, command):
        if self.bus is None:
            raise NameError("Bus is not connected.")

        if not isinstance(motor_id, int):
            raise NameError(
                "motor_id is {}. Supposed to be of type int.".format(str(command))
            )
        if not isinstance(command, list):
            raise NameError(
                "command is {}. Supposed to be of type list.".format(str(command))
            )

        msg = can.Message(
            arbitration_id=0x140 + motor_id, data=command, is_extended_id=False
        )

        self.bus.send(msg)
        ret = self.bus.recv()

        return ret.data

    def set_pid(
        self, motor_id, angle_kp, angle_ki, speed_kp, speed_ki, current_kp, current_ki
    ):
        self.send(
            motor_id,
            [0x31, 0, angle_kp, angle_ki, speed_kp, speed_ki, current_kp, current_ki],
        )

    def position_control(self, motor_id, rad_pos):
        """
        rad_pos in radian.
        """
        motor_pos = int(np.degrees(rad_pos) * 100 * 6)
        motor_speed = int(DEFAULT_MOTOR_SPEED)

        bytes_pos = motor_pos.to_bytes(4, byteorder="little", signed=True)
        bytes_speed = motor_speed.to_bytes(2, byteorder="little", signed=True)

        data = self.send(
            motor_id,
            [
                0xA4,
                0,
                bytes_speed[0],
                bytes_speed[1],
                bytes_pos[0],
                bytes_pos[1],
                bytes_pos[2],
                bytes_pos[3],
            ],
        )

    def position_control_at_speed(self, motor_id, rad_pos, rad_speed):
        """
        rad_pos in radian.
        rad_speed in radian per second
        """
        motor_pos = int(np.degrees(rad_pos) * 100 * 6)
        motor_speed = min(1000, int(np.degrees(rad_speed) * 6))

        bytes_pos = motor_pos.to_bytes(4, byteorder="little", signed=True)
        bytes_speed = motor_speed.to_bytes(2, byteorder="little", signed=True)

        data = self.send(
            motor_id,
            [
                0xA4,
                0,
                bytes_speed[0],
                bytes_speed[1],
                bytes_pos[0],
                bytes_pos[1],
                bytes_pos[2],
                bytes_pos[3],
            ],
        )

    def get_actuator_pos(self, motor_id):
        """
        Returns the actuator position in radian.
        """
        data = self.send(motor_id, [0x92, 0, 0, 0, 0, 0, 0, 0])
        data[0] = 0
        return np.radians(struct.unpack("<q", data)[0] / 2**8 / 100 / 6)

    def get_actuator_info(self, motor_id) -> ActuatorInfos:
        """
        Returns actuator informations.
        """
        data = self.send(motor_id, [0x9C, 0, 0, 0, 0, 0, 0, 0])
        temp = struct.unpack("<b", data[1:2])[0]
        torque = struct.unpack("<h", data[2:4])[0]
        speed = np.radians(struct.unpack("<h", data[4:6])[0] / 6)
        encoder = struct.unpack("<h", data[6:8])[0]
        return ActuatorInfos(temp, speed, torque, encoder)

    def turn_motor_off(self, motor_id):
        """Turns the motor off (CONNECTS the motor phases)"""
        data = self.send(motor_id, [0x80, 0, 0, 0, 0, 0, 0, 0])

    def set_zero_torque(self, motor_id):
        """set torque to zero (UNPLUGS the motor phases)"""
        data = self.send(motor_id, [0xA1, 0, 0, 0, 0, 0, 0, 0])
