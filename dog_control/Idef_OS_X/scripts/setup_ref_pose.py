"""
sudo ip link set can0 up type can bitrate 1000000
cd /home/pi/psc/IDEFX/software
python3 init.py set

sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000

ip -details -statistics link show can0
"""

import json
import sys
import time

import numpy as np
from dog_control.Idef_OS_X.low_level import bus, motor_handle

if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise NameError("This programm needs at leat one arguments")

    motor_ids = [int(x) for x in sys.argv[1:]]
    if np.any(np.array(motor_ids) <= 0) or np.any(np.array(motor_ids) > 12):
        raise NameError(
            "Motors ids must lie within [1, 12]. Received motor_ids : {}".format(
                motor_ids
            )
        )

    bus.connect()

    # choosing the rest pose
    for motor_id in motor_ids:
        print(motor_id)
        bus.set_zero_torque(motor_id)

    input(
        "Put motors {} in their rest position and then press enter.".format(motor_ids)
    )
    for motor_id in motor_ids:
        current_pos = bus.get_actuator_pos(motor_id)
        motor_handle.motor_offset.rest_offset[motor_id] = current_pos
        print("Rest position of motor", motor_id, ":", current_pos)

    input(
        "Put motors {} in their zero position and then press enter.".format(motor_ids)
    )
    for motor_id in motor_ids:
        current_pos = bus.get_actuator_pos(motor_id)
        motor_handle.motor_offset.zero_offset[motor_id] = current_pos
        print("Zero position of motor", motor_id, ":", current_pos)

    motor_handle.save_motor_offset()

    for motor_id in motor_ids:
        bus.position_control_at_speed(
            motor_id, motor_handle.motor_offset.zero_offset[motor_id], 0.5
        )
        print(motor_handle.motor_offset.zero_offset[motor_id])

    input("Get your hands away from the robot and then press enter.")

    for motor_id in motor_ids:
        bus.position_control_at_speed(
            motor_id, motor_handle.motor_offset.rest_offset[motor_id], 0.5
        )
        print(motor_handle.motor_offset.rest_offset[motor_id])

    input("Press enter to turn off the motors.")

    for motor_id in motor_ids:
        bus.turn_motor_off(motor_id)
