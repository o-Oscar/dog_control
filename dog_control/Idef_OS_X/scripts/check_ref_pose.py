"""
sudo ip link set can0 up type can bitrate 1000000
cd /home/pi/psc/IDEFX/software
python3 init.py set

sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000

ip -details -statistics link show can0
"""

from dog_control.Idef_OS_X.low_level import bus, motor_handle
import sys
import json
import time
import numpy as np

if __name__ == "__main__":
	if len(sys.argv) < 3:
		raise NameError("This programm needs at leat one arguments")
	
	motor_ids = [int(x) for x in sys.argv[2:]]
	if np.any(motor_ids <= 0) or np.any(motor_ids > 12):
		raise NameError("Motors ids must lie within [1, 12]. Received motor_ids : {}".format(motor_ids))


	bus.connect ()
	
	print("raw motor pose info : ", motor_handle.motor_offset)
	current_motor_position = motor_handle.check_startable()
	print("refined motor pose info : ", current_motor_position)
	
	input("Enter to go to zero pose.")
	for motor_id in motor_ids:
		bus.position_control_at_speed (motor_id, motor_handle.motor_offset.zero_offset[motor_id], 0.5)
	
	input("Enter to go to rest pose.")
	for motor_id in motor_ids:
		bus.position_control_at_speed (motor_id, motor_handle.motor_offset.rest_offset[motor_id], 0.5)
	
	input("Enter to turn the motors off.")
	for motor_id in motor_ids:
		bus.turn_off(motor_id)
	