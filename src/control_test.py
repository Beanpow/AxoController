# -*- encoding: utf-8 -*-
"""
@File    :   control_test.py
@Time    :   2022/10/18 21:12:39
@Author  :   allen9629
@Version :   1.0
@Contact :   allen9629@qq.com
@Desc    :   None
"""

import csv
from logging import raiseExceptions
import time
from AxoController import AxoController
from InfoPlottor import InfoPlottor
import numpy as np
import matplotlib.pyplot as plt

class testControl():

    def __init__(self,port:str="com3") -> None:
        
        print("Start Reading Gait Data")
        self.ankle_data=[]
        self.hip_data=[]

    def read_gait_data(self,file_path,isNormal: bool = True):
        csv_data_file_path = file_path
        with open(csv_data_file_path, newline='') as gait_data:
            gait_reader = csv.reader(gait_data,delimiter=',')
            for row in gait_reader:
                self.ankle_data.append(row[0])
                self.hip_data.append(row[1])
            #print(self.ankle_data[0])
            #print(self.hip_data[0])
            self.ankle_array = np.array(self.ankle_data)
            self.hip_array = np.array(self.hip_data)
    
    def plot_data(self, tgt, isPlot: bool=True):
        
        if isPlot:
            self.info_plottor = InfoPlottor(axo_controller=self.axo_controller)
            self.info_plottor.open_plot_info()
            self.info_plottor.set_target_pos(target_pos=tgt)
            plt.plot(self.ankle_array,label='al')
            plt.plot(self.hip_array,label='hl')
            plt.legend()
            plt.show()
        
        else:
            raiseExceptions('Plot option closed')

    def run(self):

        self.axo_controller.enter_control_mode()
        self.axo_controller.change_control_mode("velocity")
        self.axo_controller.set_all_motors_pos_vel_based_sync(self.ankle_array[0])

        for target_pos in self.ankle_array:
            self.plot_data(isPlot=True)
            self.axo_controller.set_all_motors_pos_vel_based_sync(target_pos)
        
        self.axo_controller.exit_control_mode()
        self.axo_controller.change_control_mode("position")
    
    def close(self):

        self.info_plottor.close_plot_info()
        self.axo_controller.close_controller()


if __name__ == "__main__":

    tc= testControl()
    tc.read_gait_data("/Users/allenlim/Documents/AxoController/src/gait_gen/gait.csv")
    
    try:
        tc.run()
    except KeyboardInterrupt:
        print("[INFO] Program Interrupted by Keyboard Commands")
    finally:
        tc.close()
    




