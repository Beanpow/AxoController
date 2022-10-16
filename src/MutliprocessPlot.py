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
import random
from multiprocessing import Pipe, Process, Value
import matplotlib

matplotlib.use('Qt5Agg')


class MutliprocessPlot:
    def __init__(self, drawSize=100, bound=[(-50, 100), (-1000, 1000), (-10, 10)]) -> None:
        self.main_conn, self.plot_conn = Pipe()

        self.drawSize = drawSize
        self.bound = bound
        self.isDraw = Value('i', 0)

        self.p = Process(target=self.DrawPic, args=(self.isDraw,))
        self.p.daemon = True

    def initFig(self):
        self.fig = plt.figure(figsize=(18, 9))
        self.ax1 = self.fig.add_subplot(131)
        self.ax2 = self.fig.add_subplot(132)
        self.ax3 = self.fig.add_subplot(133)

        self.ax1.set_ylim(self.bound[0])
        self.ax1.set_xlim(0, self.drawSize)
        self.ax2.set_ylim(self.bound[1])
        self.ax2.set_xlim(0, self.drawSize)
        self.ax3.set_ylim(self.bound[2])
        self.ax3.set_xlim(0, self.drawSize)

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
        data = np.array([]).reshape(0, 16)

        legend = ["hip_l", "knee_l", "hip_r", "knee_r"]
        line1 = [self.ax1.plot(data[:, i], label=legend[i] + "_real") for i in range(4)]
        line2 = [self.ax2.plot(data[:, i + 4], label=legend[i]) for i in range(4)]
        line3 = [self.ax3.plot(data[:, i + 8], label=legend[i]) for i in range(4)]
        line4 = [self.ax1.plot(data[:, i + 12], label=legend[i] + "_target") for i in range(4)]

        self.ax1.legend()
        self.ax2.legend()
        self.ax3.legend()

        self.ax1.set_title("Position")
        self.ax2.set_title("Velocity")
        self.ax3.set_title("Current")

        self.ax1.set_ylabel("Angle / deg")
        self.ax2.set_ylabel("Velocity / rpm")
        self.ax3.set_ylabel("Current / A")

        self.fig.canvas.draw()

        axbackground1 = self.fig.canvas.copy_from_bbox(self.ax1.bbox)  # type: ignore
        axbackground2 = self.fig.canvas.copy_from_bbox(self.ax2.bbox)  # type: ignore
        axbackground3 = self.fig.canvas.copy_from_bbox(self.ax3.bbox)  # type: ignore

        plt.show(block=False)

        while isDraw.value:
            temp = self.plot_conn.recv()
            data = np.vstack((data, temp))

            for i in range(4):
                line1[i][0].set_data(range(len(data[-self.drawSize :, i])), data[-self.drawSize :, i])
                line2[i][0].set_data(range(len(data[-self.drawSize :, i + 4])), data[-self.drawSize :, i + 4])
                line3[i][0].set_data(range(len(data[-self.drawSize :, i + 8])), data[-self.drawSize :, i + 8])
                line4[i][0].set_data(range(len(data[-self.drawSize :, i + 12])), data[-self.drawSize :, i + 12])

            self.fig.canvas.restore_region(axbackground1)  # type: ignore
            self.fig.canvas.restore_region(axbackground2)  # type: ignore
            self.fig.canvas.restore_region(axbackground3)  # type: ignore

            for i in range(4):
                self.ax1.draw_artist(line1[i][0])
                self.ax2.draw_artist(line2[i][0])
                self.ax3.draw_artist(line3[i][0])
                self.ax1.draw_artist(line4[i][0])

            self.fig.canvas.blit(self.ax1.bbox)  # type: ignore
            self.fig.canvas.blit(self.ax2.bbox)  # type: ignore
            self.fig.canvas.blit(self.ax3.bbox)  # type: ignore

            self.fig.canvas.flush_events()
            # plt.pause(0.0000000000001)
            # plt.draw()


def Update(conn):
    std = random.randint(50, 60)
    mean = random.randint(0, 100)
    conn.send([mean - std, mean, mean + std])


def main():
    t = MutliprocessPlot()
    t.start()

    try:
        indx = 0
        while indx < 500:
            indx += 1
            Update(t.main_conn)

        t.stop()
    except KeyboardInterrupt:
        t.stop()
        t.p.join()
        plt.close()
        print('close')


if __name__ == "__main__":
    main()
