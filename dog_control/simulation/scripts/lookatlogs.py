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

    plt.figure()
    s = np.sum(local_log["foot_force_local"].reshape((-1, 4, 3)), axis=1)
    print(s[-1, 2] / 9.81)
    plt.plot(s)

    plt.figure()
    s = np.sum(local_log["foot_force_global"].reshape((-1, 4, 3)), axis=1)
    print(s[-1, 2] / 9.81)
    plt.plot(s)

    # plt.figure()
    # # delta = local_log["motor_torque"]
    # plt.plot(local_log["foot_force_global"][:, 3:6])

    # plt.figure()
    # # delta = local_log["motor_torque"]
    # plt.plot(local_log["foot_force_global"][:, :3])

    plt.show()
