

# Dog Control

All the stuff you need to control a robot dog.

## TODO

Goal : having a controller with low PD gains that keeps the robot upright both in simulation and in real life.

- compute a "velocity" of the actuators (finite difference)
- write it to the log (add procedure to add as many log entry as we wish)
- write a base velocity estimator (one for each foot when we suppose it is fixed to the ground)
- test it in simulation (new mocap body fixed to every feet ?)
- test it on the real dog (have the actuators put zero force) -> zero out the pid ?

Then

- solve a least-square problem to compute the forces at the feets needed to put the dog back into balance (QP problem, use quadprog) (need to think about what we want as force / torque applied on the main body)
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