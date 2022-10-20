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


def main():
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

    total_gait = np.array(total_gait)
    t = np.arange(len(total_gait)).reshape(-1, 1) / len(total_gait)

    total_y_new = np.empty((len(total_gait) - 1, 4))
    total_y_hat_new = np.empty((len(total_gait) - 1, 4))
    for i in range(4):
        tck = interpolate.splrep(t, total_gait[:, i], s=40, per=True)

        x_new = np.arange(0, 1.02, 0.02)
        y_new = interpolate.splev(x_new, tck, der=0)[:-1]
        y_hat_new = interpolate.splev(x_new, tck, der=1)[:-1]
        total_y_new[:, i] = y_new
        total_y_hat_new[:, i] = y_hat_new

    print(total_y_hat_new.shape, total_y_new.shape)
    total_gait = np.vstack((total_y_new, total_y_hat_new))
    print(total_gait.shape)

    with open("./final_gait.csv", "w") as f:
        for gait in total_gait:
            f.writelines(','.join([str(i) for i in gait]) + '\n')


if __name__ == "__main__":
    main()
