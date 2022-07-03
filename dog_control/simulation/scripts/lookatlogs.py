import matplotlib.pyplot as plt
import numpy as np
from dog_control.controllers.logger import Logger, get_all_logs

"""

python dog_control/simulation/main.py && python dog_control/simulation/scripts/lookatlogs.py


"""

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
