# -*- encoding: utf-8 -*-
'''
@File    :   plt_without_control.py
@Time    :   2022/11/02 04:58:02
@Author  :   Beanpow
@Version :   1.0
@Contact :   beanpow@gmail.com
@Desc    :   None
'''

import time

from InfoPlottor import InfoPlottor
from AxoController import AxoController


def main():
    axo = AxoController(port="com3")
    info_plot = InfoPlottor(axo_controller=axo)
    info_plot.open_plot_info()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
