import dataclasses
import pickle
import shutil
from pathlib import Path

import numpy as np

LOG_FILES_PATH = Path("logs/local/log1")
LOG_FILE_MAX_SIZE = 30


class Logger:
    def __init__(self):
        self.log_count = 0
        self.cur_log_size = 0
        self.all_entries = {}
        self.reset_log()

        shutil.rmtree(LOG_FILES_PATH, ignore_errors=True)
        LOG_FILES_PATH.mkdir(parents=True, exist_ok=True)

    def add_entry(self, name, n_channels):
        self.all_entries[name] = n_channels
        self.reset_log()

    def reset_log(self):
        self.current_log = {
            name: np.zeros((LOG_FILE_MAX_SIZE, n_channels))
            for name, n_channels in self.all_entries.items()
        }
        self.cur_log_size = 0

    def save_log(self):
        cur_log_path = LOG_FILES_PATH / "{:04d}.pkl".format(self.log_count)
        with open(cur_log_path, "wb") as f:
            pickle.dump(self.current_log, f)

    def __setitem__(self, key, value):
        self.current_log[key][self.cur_log_size] = value

    def step(self):
        self.cur_log_size += 1
        if self.cur_log_size == LOG_FILE_MAX_SIZE:
            self.save_log()
            self.reset_log()
            self.log_count += 1


def get_all_logs(path):
    path = Path(path)

    full_log = {}

    for log_file_path in sorted(path.glob("*")):
        with open(log_file_path, "rb") as f:
            cur_log = pickle.load(f)

            for key, value in cur_log.items():
                if key not in full_log:
                    full_log[key] = []
                full_log[key].append(value)

    full_log = {key: np.concatenate(value, axis=0) for key, value in full_log.items()}

    return full_log
