# -*- encoding: utf-8 -*-
'''
@File    :   export_gait.py
@Time    :   2022/10/19 16:36:57
@Author  :   Beanpow
@Version :   1.0
@Contact :   beanpow@gmail.com
@Desc    :   None
'''
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy import interpolate


def get_original_gait():
    total_gait = []
    with open("./gait1.csv", "r") as f:
        indx = 0
        for line in f:
            if indx == 0:
                indx += 1
                continue
            line = line.split(',')

            gait = [float(line[i]) if i in [9, 4] else float(line[i]) * 180 / math.pi for i in [9, 12, 4, 7]]
            total_gait.append(gait)
    return total_gait


def main():
    total_gait = get_original_gait()
    total_gait = np.array(total_gait) * 0.7
    cycle_time = 4

    t = np.arange(len(total_gait)).reshape(-1, 1) / len(total_gait) * cycle_time

    final_shape = (3 * (len(t) - 1), 4)  # control rate is 50 / 4 * 3 H
    total_y_new = np.empty(final_shape)
    total_y_hat_new = np.empty(final_shape)

    x_new = np.arange(0, cycle_time + 0.000001, cycle_time / (len(t) - 1) / 3)
    for i in range(4):
        tck = interpolate.splrep(t, total_gait[:, i], s=40, per=True)

        y_new = interpolate.splev(x_new, tck, der=0)[:-1]
        y_hat_new = interpolate.splev(x_new, tck, der=1)[:-1]

        total_y_new[:, i] = y_new
        total_y_hat_new[:, i] = y_hat_new

    total_gait = np.vstack((total_y_new, total_y_hat_new))

    plt.plot(x_new[:-1], total_y_hat_new[:, 0])
    plt.plot(x_new[:-1], total_y_new[:, 0])
    plt.legend(["vel", "pos"])
    plt.show()

    with open("./final_gait.csv", "w") as f:
        for gait in total_gait:
            f.writelines(','.join([str(i) for i in gait]) + '\n')


if __name__ == "__main__":
    main()
