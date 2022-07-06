import time
import traceback

import numpy as np
from dog_control.controllers.async_input import AsyncInput
from dog_control.controllers.base import BaseController
from dog_control.Idef_OS_X.low_level import bus, motor_handle
from dog_control.Idef_OS_X.sensors.imu import IMU


class IdefX:
    def __init__(self, controller: BaseController, enable_motors):
        self.controller = controller
        self.enable_motors = enable_motors
        self.motors_enabled = False

        self.imu = IMU()
        # self.controller.set_sensor_reader (self.sensor_reader)

        self.target_loop_time = 1 / 30  # in s

        self.async_input = AsyncInput()

    def live(self):
        self.startup()

        try:
            self.main_loop()
        except BaseException as error:
            print("An exception occurred in the main loop :")
            print(error)
            print(traceback.format_exc())

        self.shutdown()

    def main_loop(self):
        last_loop_timestamp = time.time()

        frame = 0

        while True:
            self.imu.update()

            if self.motors_enabled:
                cur_motor_pos = motor_handle.get_motor_pos()
            else:
                cur_motor_pos = np.zeros((12,))

            target_pos = self.controller.choose_action(
                frame,
                cur_motor_pos,
                self.imu.up_vect,
                self.imu.v_rot,
            )

            if self.motors_enabled:
                motor_handle.go_to(target_pos)

            used_loop_time = (time.time() - last_loop_timestamp) / self.target_loop_time
            print("used loop time : {}%".format(100 * used_loop_time))
            while time.time() - last_loop_timestamp < self.target_loop_time:
                pass
            last_loop_timestamp = time.time()

            if self.async_input.check_for_msg() == "q":
                break

            frame += 1

    def startup(self):
        if self.enable_motors:
            self.motors_enabled = True

            bus.connect()
            motor_handle.check_startable()

            target_pos = self.controller.choose_starting_action()
            motor_handle.set_strong_pid()
            motor_handle.go_to_slow(target_pos)
            time.sleep(1)
            motor_handle.set_lite_pid()

    def shutdown(self):
        if self.motors_enabled:
            print("Stopping IdefX")
            self.stop()

            print("Going to rest pose")
            self.go_to_rest()

            print("Shutting down.")
            self.disengage()

    def stop(self):
        motor_handle.set_strong_pid()
        motor_handle.go_to_zero()
        time.sleep(1)

    def go_to_rest(self):
        motor_handle.go_to_rest()
        time.sleep(1)

    def disengage(self):
        motor_handle.disengage()
