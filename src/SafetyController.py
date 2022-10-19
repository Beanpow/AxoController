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

from InfoPlottor import InfoPlottor
from AxoController import AxoController
from utils import load_trj


class SafetyController:
    def __init__(self, port: str = "com3", isPlot: bool = True) -> None:
        self.axo_controller = AxoController(port=port)
        self.isPlot = isPlot
        if self.isPlot:
            self.info_plottor = InfoPlottor(axo_controller=self.axo_controller)
            self.info_plottor.open_plot_info()

    def run(self, trajectory: list[list[float]], duration: int):
        self.axo_controller.enter_control_mode()
        self.axo_controller.change_control_mode("velocity")
        self.axo_controller.set_all_motors_pos_vel_based_sync(trajectory[0])

        start_time = time.time()
        last_time = start_time
        idx = 0
        with tqdm(total=duration) as pbar:
            while True:
                target_pos = trajectory[idx % len(trajectory)]
                if self.isPlot:
                    self.info_plottor.set_target_pos(target_pos)

                self.axo_controller.set_all_motors_pos_vel_based(target_pos)

                time.sleep(0.1)
                idx += 1

                now_time = time.time()
                if now_time - start_time > duration:
                    break
                pbar.update(now_time - last_time)
                last_time = now_time

        self.axo_controller.exit_control_mode()
        self.axo_controller.change_control_mode("position")

    def close(self):
        if self.isPlot:
            self.info_plottor.close_plot_info()

        self.axo_controller.close_controller()


if __name__ == "__main__":
    safety_controller = SafetyController()
    try:
        # traj = [[math.sin(i / 10 + math.pi / 2) * 15 + 10, -(math.sin(i / 10 + math.pi / 2) * 15 + 15), math.sin(i / 10) * 15 + 10, -(math.sin(i / 10) * 15 + 15)] for i in range(1000)]
        traj = load_trj("./gait_gen/final_gait.csv")
        traj = traj * 10
        safety_controller.run(trajectory=traj, duration=20)
    except KeyboardInterrupt:
        print("[info] Keyboard interrupt")
    finally:
        safety_controller.close()
