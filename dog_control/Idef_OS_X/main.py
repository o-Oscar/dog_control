from dog_control.Idef_OS_X import observer
from dog_control.Idef_OS_X.IdefX import IdefX
from dog_control.controllers.dummy import DummyController
from dog_control.Idef_OS_X.observer.base import DummyObserver


def main():
    # create the objects to control IdefX.
    idefX = IdefX(
        controller=DummyController(),
        observer=DummyObserver(),
        enable_motors=True,
    )

    # use the controller
    idefX.live()


if __name__ == "__main__":
    main()
