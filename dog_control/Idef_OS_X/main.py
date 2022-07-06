from dog_control.controllers.dummy import DummyController
from dog_control.controllers.impedance_control import ImpedanceController
from dog_control.controllers.position_reacher import PositionReacherController
from dog_control.controllers.positive_twitch import PositivTwitchController
from dog_control.controllers.step_moves import StepMoveController
from dog_control.Idef_OS_X.IdefX import IdefX


def main():
    # create the objects to control IdefX.
    idefX = IdefX(
        controller=ImpedanceController(),
        enable_motors=True,
    )

    # use the controller
    idefX.live()


if __name__ == "__main__":
    main()
