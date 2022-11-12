# -*- encoding: utf-8 -*-
'''
@File    :   main.py
@Time    :   2022/10/16 21:45:25
@Author  :   Beanpow
@Version :   1.0
@Contact :   beanpow@gmail.com
@Desc    :   None
'''

import time

from SafetyController import SafetyController
from utils import load_trj


def main():
    traj = load_trj("./gait_gen/final_gait.csv")
    safety_controller = SafetyController(moment_port="com5", axo_port="com3", trajectory=traj, isPlot=True, isStimulate=True)
    try:
        safety_controller.record_safe_info(5)
        time.sleep(5)
        safety_controller.run_with_detection(10)
    except KeyboardInterrupt:
        print("[info]: Keyboard interrupt")
    finally:
        safety_controller.close()


if __name__ == "__main__":
    main()
