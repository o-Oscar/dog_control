import struct

import numpy as np
import serial


class IMU:
    def __init__(self):

        self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=3)

        self.skipped_nb = 0
        self.dumped_nb = 0

        self.v_rot = np.asarray([0, 0, 0])
        self.up_vect = np.asarray([0, 0, 1])

    def read_float(self):
        s = self.ser.read(4)
        return struct.unpack("f", s)[0]

    def update(self, debug=False):
        waiting_byte = self.ser.inWaiting()
        self.skipped_nb = waiting_byte - waiting_byte % (4 * 6 + 1)
        # ser.read(skipped_nb)
        n_read = 0
        while self.ser.inWaiting() > 4 * 6 + 1:
            n_read += 1
            x = self.ser.read()
            self.dumped_nb = 0
            while not x == b"f":
                x = self.ser.read()
                self.dumped_nb += 1

            raw_v_rot = np.asarray([self.read_float() for i in range(3)])
            new_v_rot = np.asarray([-raw_v_rot[1], raw_v_rot[0], raw_v_rot[2]])
            # v_rot_stupid = np.asarray([read_float() for i in range(3)])

            raw_up_vect = [self.read_float() for i in range(3)]
            new_up_vect = np.asarray([-raw_up_vect[1], raw_up_vect[0], raw_up_vect[2]])
            # new_up_vect = new_up_vect - np.asarray([0.023, 0.023, 0])

            lamb_up = 1  # 0.1/(1.3)**5
            lamb_v = lamb_up
            self.up_vect = self.up_vect * (1 - lamb_up) + new_up_vect * lamb_up
            self.v_rot = self.v_rot * (1 - lamb_v) + new_v_rot * lamb_v

        # print("read {} from imu".format(n_read))
