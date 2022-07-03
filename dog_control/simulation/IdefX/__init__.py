import time

from dog_control.controllers.async_input import AsyncInput
from dog_control.controllers.base import BaseController
from dog_control.simulation.engine import BaseEngine


class IdefX:
    def __init__(
        self, controller: BaseController, simulation_engine: BaseEngine, realtime=True
    ):
        self.controller = controller
        self.simulation_engine = simulation_engine
        self.realtime = realtime

        self.target_loop_time = 1 / 30

        self.max_iteration = 1000
        # self.max_iteration = 100000

        self.async_input = AsyncInput()

    def live(self):

        self.startup()

        self.main_loop()

        self.shutdown()

    def startup(self):
        # setup the dog in the simulation to be in the right orientation/position for the current test
        pass

    def main_loop(self):

        last_loop_timestamp = time.time()
        for frame in range(self.max_iteration):
            action = self.controller.choose_action(
                frame,
                self.simulation_engine.get_motor_position(),
            )
            self.simulation_engine.step(action)

            used_loop_time = (time.time() - last_loop_timestamp) / self.target_loop_time
            print("used loop time : {:.2f}%".format(100 * used_loop_time))
            while (
                time.time() - last_loop_timestamp < self.target_loop_time
                and self.realtime
            ):
                pass
            last_loop_timestamp = time.time()

            if self.async_input.check_for_msg() == "q":
                break

    def shutdown(self):
        pass
