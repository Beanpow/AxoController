# -*- encoding: utf-8 -*-
'''
@File    :   MutliProcessPlot.py
@Time    :   2022/10/16 21:14:52
@Author  :   Beanpow
@Version :   1.0
@Contact :   beanpow@gmail.com
@Desc    :   None
'''

import matplotlib.pyplot as plt
import numpy as np
import time
from multiprocessing import Pipe, Process, Value
import matplotlib

matplotlib.use('Qt5Agg')


class MutliprocessPlot:
    def __init__(self, drawSize=300, bound=[(0, 1)]) -> None:
        self.main_conn, self.plot_conn = Pipe()

        self.drawSize = drawSize
        self.bound = bound
        self.isDraw = Value('i', 0)

        self.p = Process(target=self.DrawPic, args=(self.isDraw,))
        self.p.daemon = True

    def initFig(self):
        self.fig = plt.figure(figsize=(10, 5))
        self.ax1 = self.fig.add_subplot(111)

        self.ax1.set_ylim(self.bound[0])
        self.ax1.set_xlim(0, self.drawSize)

        plt.ioff()

    def _set_status(self, status: int):
        self.isDraw.value = status  # type: ignore

    def start(self):
        self._set_status(1)
        self.p.start()

    def stop(self):
        self._set_status(0)
        time.sleep(0.1)
        indx = 0
        while self.p.is_alive():
            indx += 1
            self._set_status(0)
            if indx > 10:
                print("[Error] stop failed")
                break

    def DrawPic(self, isDraw):
        self.initFig()
        data = np.array([]).reshape(0, 1)

        line1 = self.ax1.plot(data[:])

        self.ax1.set_title("Stimulate Signal")

        self.ax1.set_ylabel("Amplitude")
        self.fig.canvas.draw()

        axbackground1 = self.fig.canvas.copy_from_bbox(self.ax1.bbox)  # type: ignore

        plt.show(block=False)

        while isDraw.value:
            temp = self.plot_conn.recv()
            data = np.vstack((data, temp))

            for i in range(4):
                line1[0].set_data(range(len(data[-self.drawSize :])), data[-self.drawSize :])

            self.fig.canvas.restore_region(axbackground1)  # type: ignore

            self.ax1.draw_artist(line1[0])

            self.fig.canvas.blit(self.ax1.bbox)  # type: ignore

            self.fig.canvas.flush_events()
            # plt.pause(0.0000000000001)
            # plt.draw()
