import dataclasses
import pickle
import shutil
from pathlib import Path

import numpy as np


@dataclasses.dataclass
class Log:
    size: int
    motor_targets: np.ndarray
    motor_positions: np.ndarray
    up_vectors: np.ndarray
    rotation_speeds: np.ndarray


LOG_FILES_PATH = Path("logs/local/log1")
LOG_FILE_MAX_SIZE = 30


class Logger:
    def __init__(self):
        self.log_count = 0
        self.reset_log()

        shutil.rmtree(LOG_FILES_PATH, ignore_errors=True)
        LOG_FILES_PATH.mkdir(parents=True, exist_ok=True)

    def reset_log(self):
        self.current_log = Log(
            0,
            np.zeros((LOG_FILE_MAX_SIZE, 12)),
            np.zeros((LOG_FILE_MAX_SIZE, 12)),
            np.zeros((LOG_FILE_MAX_SIZE, 3)),
            np.zeros((LOG_FILE_MAX_SIZE, 3)),
        )

    def save_log(self):
        cur_log_path = LOG_FILES_PATH / "{:04d}.pkl".format(self.log_count)
        with open(cur_log_path, "wb") as f:
            pickle.dump(self.current_log, f)
        self.log_count += 1

    def log(self, motor_targets, motor_positions, up_vector, rotation_speed):

        self.current_log.motor_targets[self.current_log.size] = motor_targets
        self.current_log.motor_positions[self.current_log.size] = motor_positions
        self.current_log.up_vectors[self.current_log.size] = up_vector
        self.current_log.rotation_speeds[self.current_log.size] = rotation_speed

        self.current_log.size = self.current_log.size + 1

        if self.current_log.size == LOG_FILE_MAX_SIZE:
            self.save_log()
            self.reset_log()


def get_all_logs(path):
    path = Path(path)

    full_size = 0
    all_motor_targets = []
    all_motor_positions = []
    all_up_vectors = []
    all_rotation_speeds = []
    for log_file_path in sorted(path.glob("*")):
        with open(log_file_path, "rb") as f:
            cur_log = pickle.load(f)

            full_size += cur_log.size
            all_motor_targets.append(cur_log.motor_targets)
            all_motor_positions.append(cur_log.motor_positions)
            all_up_vectors.append(cur_log.up_vectors)
            all_rotation_speeds.append(cur_log.rotation_speeds)

    full_log = Log(
        full_size,
        np.concatenate(all_motor_targets, axis=0),
        np.concatenate(all_motor_positions, axis=0),
        np.concatenate(all_up_vectors, axis=0),
        np.concatenate(all_rotation_speeds, axis=0),
    )

    return full_log
