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

    with open("./final_gait.csv", "w") as f:
        for gait in total_gait:
            f.writelines(','.join([str(i) for i in gait]) + '\n')


if __name__ == "__main__":
    main()
