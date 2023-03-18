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
import argparse

from SafetyController import SafetyController
from utils import load_trj


def main():
    args = parser_args()
    traj = load_trj(args.traj_path)
    safety_controller = SafetyController(moment_port=args.moment_port, axo_port=args.axo_port, trajectory=traj, isPlot=args.isPlot, isStimulate=args.isStimulate)
    try:
        input("Press Enter to start...")
        safety_controller.record_safe_info(5)
        time.sleep(10)
        safety_controller.run_with_detection(15 * 10)
    except KeyboardInterrupt:
        print("[info]: Keyboard interrupt")
    except Exception as e:
        print("[error]: ", e)
    finally:
        safety_controller.close()


def parser_args():
    parser = argparse.ArgumentParser(description="Safety Controller")
    parser.add_argument("--moment_port", type=str, default="com5", help="The port of moment sensor")
    parser.add_argument("--axo_port", type=str, default="com3", help="The port of axo")
    parser.add_argument("--traj_path", type=str, default="./gait_gen/final_gait.csv", help="The path of trajectory")
    parser.add_argument("--isPlot", type=bool, default=True, help="Whether to plot the data")
    parser.add_argument("--isStimulate", type=bool, default=False, help="Whether to stimulate the user")
    args = parser.parse_args()
    return args


if __name__ == "__main__":
    main()
