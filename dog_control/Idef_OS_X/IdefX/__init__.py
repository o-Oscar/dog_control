import time
from msilib.schema import Error

from dog_control.controllers.base import BaseController
from dog_control.Idef_OS_X.low_level import bus, motor_handle
from dog_control.Idef_OS_X.sensors import SensorReader


class IdefX:
    def __init__(self, controller: BaseController, enable_motors):
        self.controller = controller
        self.enable_motors = enable_motors
        self.motors_enabled = False

        self.sensor_reader = SensorReader()
        # self.controller.set_sensor_reader (self.sensor_reader)

        self.target_loop_time = 1 / 30  # in s

    def live(self):
        self.startup()

        try:
            self.main_loop()
        except BaseException as error:
            print("An exception occurred in the main loop :")
            print(error)

        self.shutdown()

    def main_loop(self):
        last_loop_timestamp = time.time()

        while True:
            self.sensor_reader.update_sensors()
            target_pos = self.controller.choose_action()

            if self.motors_enabled:
                motor_handle.go_to(target_pos)

            used_loop_time = (time.time() - last_loop_timestamp) / self.target_loop_time
            print("used loop time : {}%".format(100 * used_loop_time))
            while time.time() - last_loop_timestamp < self.target_loop_time:
                pass
            last_loop_timestamp = time.time()

    def startup(self):
        if self.enable_motors:
            self.motors_enabled = True

            self.sensor_reader.init_sensors()

            # TODO : define and go to the starting pose that the controller expects

    def shutdown(self):
        if self.motors_enabled:
            print("Stopping IdefX")
            self.stop()

            print("Going to rest pose")
            self.go_to_rest()

            print("Shutting down.")
            self.disengage()

    def stop(self):
        motor_handle.set_lite_pid()
        time.sleep(1)

    def go_to_rest(self):
        motor_handle.set_strong_pid()
        motor_handle.go_to_zero()
        time.sleep(1)
        motor_handle.go_to_rest()
        time.sleep(1)

    def disengage(self):
        motor_handle.disengage()
