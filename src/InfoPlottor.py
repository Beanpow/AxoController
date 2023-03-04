# -*- encoding: utf-8 -*-
'''
@File    :   InfoPlottor.py
@Time    :   2022/10/16 21:13:49
@Author  :   Beanpow
@Version :   1.0
@Contact :   beanpow@gmail.com
@Desc    :   None
'''
import time
import numpy as np

from MutliprocessPlot import MutliprocessPlot
import threading
from AxoController import AxoController


class InfoPlottor:
    def __init__(self, axo_controller: AxoController) -> None:
        self.mutliprocess_plot = MutliprocessPlot(drawSize=200, bound=[(-80, 60), (-1800, 1700), (-12, 12)])
        self.axo_controller = axo_controller

        # Variable for plot_info
        self.is_plot_info = False
        self.target_pos = [0, 0, 0, 0]

        self.data = []

    def open_plot_info(self) -> None:
        self.is_plot_info = True
        self.mutliprocess_plot.start()

        self.plot_info_thread = threading.Thread(target=self._plot_info)
        self.plot_info_thread.setDaemon(True)
        self.plot_info_thread.start()

    def close_plot_info(self) -> None:
        self.is_plot_info = False
        self.mutliprocess_plot.stop()

        self.data = np.array(self.data)
        np.save(f"logs/axo_info_{time.strftime('%Y-%m-%d_%H-%M-%S', time.localtime())}.npy", self.data)

        time.sleep(0.3)
        assert self.plot_info_thread.is_alive() is False

    def set_target_pos(self, target_pos: list[float]) -> None:
        self.target_pos = target_pos

    def _plot_info(self):
        while self.is_plot_info:
            start_time = time.time()
            leg_info = self.axo_controller.get_leg_info()

            leg_pos = [leg_info["left"]["hip_pos"], leg_info["left"]["knee_pos"], leg_info["right"]["hip_pos"], leg_info["right"]["knee_pos"]]
            leg_vel = [leg_info["left"]["hip_vel"], leg_info["left"]["knee_vel"], leg_info["right"]["hip_vel"], leg_info["right"]["knee_vel"]]
            leg_current = [leg_info["left"]["hip_current"], leg_info["left"]["knee_current"], leg_info["right"]["hip_current"], leg_info["right"]["knee_current"]]

            self.data.append([start_time, *leg_pos, *leg_vel, *leg_current, *self.target_pos])
            self.mutliprocess_plot.main_conn.send([*leg_pos, *self.axo_controller.get_leg_vel(), *self.axo_controller.get_leg_current(), *self.target_pos])
            time.sleep(0.03)
