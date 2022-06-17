"""
sudo ip link set can0 up type can bitrate 1000000
cd /home/pi/psc/IDEFX/software
python3 init.py set

sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000

ip -details -statistics link show can0
"""

import idefX.backend as b
import idefX.motors as motors
import sys
import json
import time
import numpy as np

def load_motor_pose ():
	with open("motor_pose.txt") as f:
		pose = json.loads(''.join(f.readlines()))
	return pose

def save_motor_pose (pose):
	with open("motor_pose.txt", "w") as f:
		f.write(json.dumps(pose))


if __name__ == "__main__":
	if len(sys.argv) < 3:
		raise NameError("This programm needs at leat one arguments")
	
	motor_ids = [int(x) for x in sys.argv[2:]]
	if np.any(motor_ids <= 0) or np.any(motor_ids > 12):
		raise NameError("Motors ids must lie within [1, 12]. Received motor_ids : {}".format(motor_ids))


	b.init_bus ()
	
	all_motor_id = [int(x) for x in sys.argv[2:]]
	motor_pose = load_motor_pose()
	
	# choosing the rest pose
	for motor_id in all_motor_id:
		print(motor_id)
		b.set_zero_torque(motor_id)
	
	
	input("Put motors {} at their rest position.".format(all_motor_id))
	for motor_id in all_motor_id:
		motor_pose["rest"][str(motor_id)] = b.actuator_pos(motor_id)
		print(b.actuator_pos(motor_id))
	
	input("Put motors {} at their zero position.".format(all_motor_id))
	for motor_id in all_motor_id:
		motor_pose["zero"][str(motor_id)] = b.actuator_pos(motor_id)
		
	save_motor_pose(motor_pose)
	
	for motor_id in all_motor_id:
		b.position_control_at_speed (motor_id, motor_pose["zero"][str(motor_id)], 0.5)
	
	input("Get your hands away from the robot.")
	
	for motor_id in all_motor_id:
		b.position_control_at_speed (motor_id, motor_pose["rest"][str(motor_id)], 0.5)
	
	input("Enter to go back to rest.")
	
	for motor_id in all_motor_id:
		b.turn_off(motor_id)
		
	
	elif sys.argv[1] == "check":
		b.init_bus ()
		
		all_motor_id = [int(x) for x in sys.argv[2:]]
		motor_pose = load_motor_pose()
		print("raw motor pose info : ", motor_pose)
		motor_pose = motors.check_start(all_motor_id, motor_pose)
		print("refined motor pose info : ", motor_pose)
		
		input("Enter to go to zero pose.")
		for motor_id in all_motor_id:
			b.position_control_at_speed (motor_id, motor_pose["zero"][str(motor_id)], 0.5)
		
		input("Enter to go to rest pose.")
		for motor_id in all_motor_id:
			b.position_control_at_speed (motor_id, motor_pose["rest"][str(motor_id)], 0.5)
		
		input("Enter to turn the motors off.")
		for motor_id in all_motor_id:
			b.turn_off(motor_id)
	
	elif sys.argv[1] == "kill":
		b.init_bus ()
		motors.disengage()
	
	else:
		raise NameError("{} is not a valid argument.".format(sys.argv[1]))
