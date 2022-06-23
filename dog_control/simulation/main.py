from dog_control.controllers.dummy import DummyController
from dog_control.simulation.engine import Engine, EngineConfig
from dog_control.simulation.IdefX import IdefX


def main():
    # create the objects to control IdefX.
    engine_config = EngineConfig(fix_root=True)

    idefX = IdefX(controller=DummyController(), simulation_engine=Engine(engine_config))

    # use the controller
    idefX.live()


if __name__ == "__main__":
    main()
