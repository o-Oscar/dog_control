# from dog_control.controllers.dummy import Controller
# from dog_control.controllers.impedance_control import Controller
# from dog_control.controllers.position_reacher import Controller

# from dog_control.controllers.positive_twitch import Controller
# from dog_control.controllers.step_moves import Controller
from dog_control.controllers.vertical_feet import Controller
from dog_control.Idef_OS_X.IdefX import IdefX


def main():
    # create the objects to control IdefX.
    idefX = IdefX(
        controller=Controller(),
        enable_motors=True,
    )

    # use the controller
    idefX.live()


if __name__ == "__main__":
    main()
