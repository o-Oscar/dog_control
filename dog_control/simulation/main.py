import re

from dog_control.controllers.dummy import DummyController
from dog_control.controllers.position_reacher import PositionReacherController
from dog_control.controllers.positive_twitch import PositivTwitchController
from dog_control.controllers.step_moves import StepMoveController
from dog_control.simulation.engine import Engine, EngineConfig
from dog_control.simulation.IdefX import IdefX


def main():

    realtime = False

    # create the objects to control IdefX.
    engine_config = EngineConfig(
        fix_root=True,
        use_viewer=realtime,
        base_motor_kp=10,
        base_motor_kd=20,
        maximum_torque=11000000,
    )

    # idefX = IdefX(
    #     controller=DummyController(),
    #     simulation_engine=Engine(engine_config),
    # )
    idefX = IdefX(
        controller=StepMoveController(),
        simulation_engine=Engine(engine_config),
        realtime=realtime,
    )

    # use the controller
    idefX.live()


if __name__ == "__main__":
    main()
