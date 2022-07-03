from dog_control.Idef_OS_X.low_level.bus import CANBus
from dog_control.Idef_OS_X.low_level.motor_handle import MotorHandle

bus = CANBus()
motor_handle = MotorHandle(bus)
# motor_handle = MotorHandle(bus, True)
