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
from MutliprocessPlot import MutliprocessPlot
import threading
from AxoController import AxoController


class InfoPlottor:
    def __init__(self, axo_controller: AxoController) -> None:
        self.mutliprocess_plot = MutliprocessPlot(drawSize=200, bound=[(-50, 100), (-1000, 1000), (-10, 10)])
        self.axo_controller = axo_controller

        # Variable for plot_info
        self.is_plot_info = False
        self.target_pos = [0, 0, 0, 0]

    def open_plot_info(self) -> None:
        self.is_plot_info = True
        self.mutliprocess_plot.start()

        self.plot_info_thread = threading.Thread(target=self._plot_info)
        self.plot_info_thread.setDaemon(True)
        self.plot_info_thread.start()

    def close_plot_info(self) -> None:
        self.is_plot_info = False
        self.mutliprocess_plot.stop()

        time.sleep(0.3)
        assert self.plot_info_thread.is_alive() is False

    def set_target_pos(self, target_pos: list[float]) -> None:
        self.target_pos = target_pos

    def _plot_info(self):
        while self.is_plot_info:
            self.mutliprocess_plot.main_conn.send([*self.axo_controller.get_leg_pos(), *self.axo_controller.get_leg_vel(), *self.axo_controller.get_leg_current(), *self.target_pos])
            time.sleep(0.01)
