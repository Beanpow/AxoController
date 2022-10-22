# -*- encoding: utf-8 -*-
'''
@File    :   SafetyController.py
@Time    :   2022/10/16 22:22:39
@Author  :   Beanpow
@Version :   1.0
@Contact :   beanpow@gmail.com
@Desc    :   None
'''

import time
from tqdm import tqdm
import numpy as np
from typing import Tuple
from InfoPlottor import InfoPlottor
from AxoController import AxoController
from utils import load_trj
import matplotlib.pyplot as plt


class SafetyController:
    def __init__(self, port: str = "com3", isPlot: bool = True) -> None:
        self.axo_controller = AxoController(port=port)
        self.isPlot = isPlot
        if self.isPlot:
            self.info_plottor = InfoPlottor(axo_controller=self.axo_controller)
            self.info_plottor.open_plot_info()

    def run_one_cycle(self, trajectory: Tuple[list[list[float]], list[list[float]]], one_cycle_duration: float):
        traj_pos, traj_vel = trajectory
        one_step_time = one_cycle_duration / len(traj_pos)
        for target_pos, target_vel in zip(traj_pos, traj_vel):
            start_time = time.time()
            if self.isPlot:
                self.info_plottor.set_target_pos(target_pos)

            self.axo_controller.set_all_motors_pos_vel_based(target_pos, target_vel)

            assert time.time() - start_time <= one_step_time, "one step time is too short"
            print(one_step_time - (time.time() - start_time))
            time.sleep(one_step_time - (time.time() - start_time))

    def run(self, trajectory: Tuple[list[list[float]], list[list[float]]], duration: int):
        self.axo_controller.enter_control_mode()
        time.sleep(4)  # wait for the robot reset to the initial position

        self.axo_controller.exit_control_mode()
        self.axo_controller.change_control_mode("velocity")
        self.axo_controller.set_all_motors_vel([0, 0, 0, 0])
        self.axo_controller.enter_control_mode(isChangeMode=False)
        self.axo_controller.set_all_motors_pos_vel_based_sync(trajectory[0][0])

        start_time = time.time()
        last_time = start_time
        with tqdm(total=duration) as pbar:
            while True:
                self.run_one_cycle(trajectory, one_cycle_duration=4)

                now_time = time.time()
                if now_time - start_time > duration:
                    break
                pbar.update(now_time - last_time)
                last_time = now_time

        self.axo_controller.exit_control_mode()
        self.axo_controller.change_control_mode("position")

    def close(self):
        pid_res = self.axo_controller.pid_res
        vel_his = self.axo_controller.pid_list[0].vel_history
        lvel_his = self.axo_controller.pid_list[0].lvel_history
        pos_his = self.axo_controller.pid_list[0].pos_histroy
        if self.isPlot:
            self.info_plottor.close_plot_info()

        self.axo_controller.close_controller()
        print("pid_res:", pid_res)

        plt.plot(pid_res)
        plt.legend(["p", "d", "current_vel", "target_vel"])
        plt.show()

        np.save("pid_res.npy", np.array(pid_res))

        plt.plot(vel_his)
        plt.plot(lvel_his)
        plt.show()

        plt.plot(np.array(pos_his)[:, 0])
        plt.plot(np.array(pos_his)[:, 1])
        plt.legend(["target_pos", "current_pos"])
        plt.show()


if __name__ == "__main__":
    safety_controller = SafetyController()
    try:
        # traj = [[math.sin(i / 10 + math.pi / 2) * 15 + 10, -(math.sin(i / 10 + math.pi / 2) * 15 + 15), math.sin(i / 10) * 15 + 10, -(math.sin(i / 10) * 15 + 15)] for i in range(1000)]
        traj_pos, traj_vel = load_trj("./gait_gen/final_gait.csv")
        safety_controller.run(trajectory=(traj_pos, traj_vel), duration=1800)
    except KeyboardInterrupt:
        print("[info] Keyboard interrupt")
    finally:
        safety_controller.close()
