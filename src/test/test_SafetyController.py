import sys

import numpy as np

sys.path.append("..")
from SafetyController import SafetyController  # noqa: E402
from utils import load_trj  # noqa: E402


def main():
    traj = load_trj("../gait_gen/final_gait.csv")
    pos = traj[0]
    vel = traj[1]

    pos = np.array(pos)
    vel = np.array(vel)
    # pos[:, 0] *= 0
    # pos[:, 1] *= 0
    # pos[:, 3] *= 0

    # vel[:, 0] *= 0
    # vel[:, 1] *= 0
    # vel[:, 3] *= 0

    traj = (pos.tolist(), vel.tolist())
    # print(traj)

    controller = SafetyController(moment_port="com5", axo_port="com3", trajectory=traj, isPlot=True, isStimulate=False)
    controller.run_with_detection(10)


if __name__ == "__main__":
    main()
