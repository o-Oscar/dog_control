

# Dog Control

All the stuff you need to control a robot dog.

## TODO

Goal : having a simple controller that flings the motors around in sim and in real life.

TODO :
-> Understand the motor commands and implement torque limits on the motors
-> Remove some lines to have a zero in the xml that is actually zero 
-> Remove limits to the motors in the xml
-> create a controller that makes little impulses toward the positiv direction of the motor to figure out the direction of each motor.
-> implement an inverting procedure to map actions to the right actuator direction
-> start the dog and figure out the "right actuator direction" for each actuator

## Status

### mxml creation

A first version has been coded. Still needs to match the real robot.

mujoco xml reference : https://mujoco.readthedocs.io/en/latest/XMLreference.html#xml-reference

## Notes

To scann the network for the raspberry :
nmap -sn 192.168.1.0/24 | grep raspberrypi
ssh pi@192.168.1.43
To setup a network on the raspberry :
sudo raspi-config
System option
Wireless LAN
Enter Box name and password