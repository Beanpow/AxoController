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
import math

from InfoPlottor import InfoPlottor
from AxoController import AxoController


class SafetyController:
    def __init__(self, port: str = "com3", isPlot: bool = True) -> None:
        self.axo_controller = AxoController(port=port)
        self.isPlot = isPlot
        if self.isPlot:
            self.info_plottor = InfoPlottor(axo_controller=self.axo_controller)
            self.info_plottor.open_plot_info()

    def run(self, trajectory: list[list[float]]):
        self.axo_controller.enter_control_mode()
        self.axo_controller.change_control_mode("velocity")
        self.axo_controller.set_all_motors_pos_vel_based_sync(trajectory[0])

        for target_pos in trajectory:
            if self.isPlot:
                self.info_plottor.set_target_pos(target_pos)

            self.axo_controller.set_all_motors_pos_vel_based(target_pos)
            time.sleep(0.1)

        self.axo_controller.exit_control_mode()
        self.axo_controller.change_control_mode("position")

    def close(self):
        if self.isPlot:
            self.info_plottor.close_plot_info()

        self.axo_controller.close_controller()


if __name__ == "__main__":
    safety_controller = SafetyController()
    try:
        traj = [[math.sin(i / 10 + math.pi / 2) * 15 + 10, -(math.sin(i / 10 + math.pi / 2) * 15 + 15), math.sin(i / 10) * 15 + 10, -(math.sin(i / 10) * 15 + 15)] for i in range(1000)]
        safety_controller.run(trajectory=traj)
    except KeyboardInterrupt:
        print("[info] Keyboard interrupt")
    finally:
        safety_controller.close()
