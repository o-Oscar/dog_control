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
    if len(sys.argv) < 3:
        raise NameError("This programm needs at leat one arguments")

    motor_ids = [int(x) for x in sys.argv[2:]]
    if np.any(motor_ids <= 0) or np.any(motor_ids > 12):
        raise NameError(
            "Motors ids must lie within [1, 12]. Received motor_ids : {}".format(
                motor_ids
            )
        )

    bus.connect()

    # choosing the rest pose
    for motor_id in motor_ids:
        print(motor_id, bus.get_actuator_pos(motor_id))
