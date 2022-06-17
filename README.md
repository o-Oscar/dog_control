

# Dog Control

All the stuff you need to control a robot dog.

## TODO

Goal : having a simple controller that flings the motors around in sim and in real life.

TODO :
-> fix motor_handle.py so that the motor ids used can be arbitrary ones -> the motor offsets must be dictionnary with (hopefully) motor ids (int) as keys
-> fix shutdown so that it will do its best to kill all motors.
-> create an abstract base controller.

## Status

### mmjcf creation

A first version has been coded. Still needs to match the real robot.

mujoco xml reference : https://mujoco.readthedocs.io/en/latest/XMLreference.html#mjcf-reference