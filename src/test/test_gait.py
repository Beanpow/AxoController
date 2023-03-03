import sys

import numpy as np
import matplotlib.pyplot as plt

sys.path.append("..")
from utils import load_trj  # noqa: E402


def main():
    trj = load_trj("../gait_gen/final_gait.csv")
    pos, vel = trj

    pos = np.array(pos)
    vel = np.array(vel)

    print(pos.shape)
    print(vel.shape)

    plt.plot(pos[:, 0], label='hl')
    plt.plot(pos[:, 1], label='kl')
    plt.plot(pos[:, 2], label='hr')
    plt.plot(pos[:, 3], label='kr')

    plt.show()

    plt.plot(vel[:, 0], label='hl')
    plt.plot(vel[:, 1], label='kl')
    plt.plot(vel[:, 2], label='hr')
    plt.plot(vel[:, 3], label='kr')

    plt.show()


if __name__ == "__main__":
    main()
