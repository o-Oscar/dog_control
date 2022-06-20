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
	bus.connect ()
	motor_handle.disangage()