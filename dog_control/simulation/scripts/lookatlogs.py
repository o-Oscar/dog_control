import matplotlib.pyplot as plt
import numpy as np
from dog_control.controllers.logger import Logger, get_all_logs

"""

python dog_control/simulation/main.py && python dog_control/simulation/scripts/lookatlogs.py


"""

if False:
    log_path = "logs/local/log1"
    local_log = get_all_logs(log_path)

    log_path = "logs/dog/log1"
    dog_log = get_all_logs(log_path)

    # print(np.sum(log.motor_positions))
    plt.plot(local_log.motor_targets[:, 5])
    plt.plot(local_log.motor_positions[:, 5])
    plt.plot(dog_log.motor_targets[:, 5])
    plt.plot(dog_log.motor_positions[:, 5])

    plt.show()

if True:
    log_path = "logs/local/log1"
    local_log = get_all_logs(log_path)
    print(local_log.keys())

    log_path = "logs/dog/log1"
    dog_log = get_all_logs(log_path)

    # print(np.sum(log.motor_positions))
    # plt.figure()
    # plt.plot(local_log["estimated_motor_velocity"])
    plt.figure()
    # plt.plot(local_log["estimated_base_velocity"][:, :3])
    plt.plot(dog_log["base_velocity_global"])
    # plt.plot(local_log["estimated_foot_pos"][:, :3])
    # plt.plot(
    #     local_log["estimated_base_velocity"][:, 2]
    #     / local_log["estimated_base_velocity"][:, 1]
    # )
    # plt.plot(dog_log["up_vectors"])
    # plt.figure()
    # plt.plot(local_log["rotation_speed"])
    # plt.plot(dog_log["rotation_speeds"])

    plt.show()
