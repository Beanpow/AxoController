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

from InfoPlottor import InfoPlottor
from AxoController import AxoController


def main():
    port = "COM3"
    axo_controller = AxoController(port=port)
    info_plottor = InfoPlottor(axo_controller=axo_controller)
    info_plottor.open_plot_info()
    time.sleep(100)


if __name__ == "__main__":
    main()
