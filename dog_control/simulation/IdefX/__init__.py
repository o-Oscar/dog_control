import time
from logging import shutdown

from dog_control.controllers.base import BaseController
from dog_control.simulation.engine import BaseEngine


class IdefX:
    def __init__(self, controller: BaseController, simulation_engine: BaseEngine):
        self.controller = controller

        self.simulation_engine = simulation_engine

        self.target_loop_time = 1 / 30

        self.max_iteration = 10
        self.max_iteration = 100000

    def live(self):

        self.startup()

        self.main_loop()

        self.shutdown()

    def startup(self):
        # setup the dog in the simulation to be in the right orientation/position for the current test
        pass

    def main_loop(self):

        last_loop_timestamp = time.time()
        for i in range(self.max_iteration):
            action = self.controller.choose_action()
            self.simulation_engine.step(action)

            used_loop_time = (time.time() - last_loop_timestamp) / self.target_loop_time
            print("used loop time : {:.2f}%".format(100 * used_loop_time))
            while time.time() - last_loop_timestamp < self.target_loop_time:
                pass
            last_loop_timestamp = time.time()

    def shutdown(self):
        pass
