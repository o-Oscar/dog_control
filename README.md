

# Dog Control

All the stuff you need to control a robot dog.

## TODO

Goal : having a controller with low PD gains that keeps the robot upright both in simulation and in real life.

- read the imu data
- write a controller that keeps the feets under the dog.

Then

- compute a "velocity" of the actuators
- write a base velocity estimator (one for each foot when we suppose it is fixed to the ground)

Then

- solve a least-square problem to compute the forces at the feets needed to put the dog back into balance (QP problem, use quadprog)
- add the constraints that the forces must be over the ground
- add the constraint that the torques at each motor must be smaller than some threshold

## Notes

### mujoco

mujoco xml reference : https://mujoco.readthedocs.io/en/latest/XMLreference.html#xml-reference

And the other very good one : https://mujoco.readthedocs.io/en/latest/APIreference.html#mjdata

### raspberry

To setup a network on the raspberry :

- sudo raspi-config
- System option
- Wireless LAN
- Enter Box name and password