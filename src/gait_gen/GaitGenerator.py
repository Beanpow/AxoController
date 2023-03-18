import math

import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt

from export_gait import get_original_gait


class GaitGenerator:
    def __init__(self, gait_path: str = "./gait1.csv", cycle_time: float = 4) -> None:
        self.cycle_time = cycle_time
        self.gait_path = gait_path

        self.origianl_gait = self.get_original_gait()
        self.tck_list = self.generate_tck()

    def get_original_gait(self):
        total_gait = []
        with open(self.gait_path, "r") as f:
            indx = 0
            for line in f:
                if indx == 0:
                    indx += 1
                    continue
                line = line.split(',')

                gait = [float(line[i]) if i in [9, 4] else float(line[i]) * 180 / math.pi for i in [9, 12, 4, 7]]
                total_gait.append(gait)
        return total_gait

    def generate_tck(self):
        total_gait = get_original_gait()
        total_gait = np.array(total_gait) * 0.7  # reduce the amplitude to 70% for collision avoidance

        t = np.arange(len(total_gait)).reshape(-1, 1) / len(total_gait) * self.cycle_time
        tck_list = []

        for i in range(4):
            tck = interpolate.splrep(t, total_gait[:, i], s=40, per=True)
            tck_list.append(tck)

        return tck_list

    def query(self, time: float):
        time = time % self.cycle_time

        pos_list = []
        vel_list = []

        for tck in self.tck_list:
            pos = interpolate.splev(time, tck, der=0)
            vel = interpolate.splev(time, tck, der=1)
            pos_list.append(pos)
            vel_list.append(vel)

        return np.array(pos_list), np.array(vel_list)


if __name__ == "__main__":
    gaitGenerator = GaitGenerator()
    pos_list = []
    vel_list = []

    for i in range(100):
        pos, vel = gaitGenerator.query(i / 100 * 10)
        print(pos)
        pos_list.append(pos)
        vel_list.append(vel)

    pos_list = np.array(pos_list)
    vel_list = np.array(vel_list)

    pos_list = vel_list
    plt.plot(pos_list[:, 0])
    plt.plot(pos_list[:, 1])
    plt.plot(pos_list[:, 2])
    plt.plot(pos_list[:, 3])
    plt.show()
