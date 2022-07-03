

# Dog Control

All the stuff you need to control a robot dog.

## TODO

Goal : having a simple controller that flings the motors around in sim and in real life.

TODO :
-> implement the control (PID + limit) of the actuator in simulation "by hand"
-> test it against the logs
-> find the best matching parameters

-> read the imu data
-> write a controller that keeps the feets under the dog.

-> compute a "velocity" of the actuators
-> write a base velocity estimator

## Status

### mxml creation

A first version has been coded. Still needs to match the real robot.

mujoco xml reference : https://mujoco.readthedocs.io/en/latest/XMLreference.html#xml-reference

## Notes

To scann the network for the raspberry :
nmap -sn 192.168.1.0/24 | grep raspberrypi
nmap -sn 192.168.1.0/24 | sed -r -n -e 's/^.+raspberry.+\((.+)\)$/\1/p'
ssh pi@192.168.1.43
To setup a network on the raspberry :
sudo raspi-config
System option
Wireless LAN
Enter Box name and password