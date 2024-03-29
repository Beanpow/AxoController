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
import os
from tqdm import tqdm
from typing import Callable
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

from InfoPlottor import InfoPlottor
from AxoController import AxoController
from MomentManager import MomentManager
from StimulateController import StimulateController
from utils import accurate_delay


class SafetyController:
    def __init__(self, axo_port: str, moment_port: str, trajectory: tuple[list[list[float]], list[list[float]]], isPlot: bool = True, isStimulate: bool = False) -> None:
        self.trajectory = trajectory
        self.one_cycle_time = 4  # unit: second
        self.factor = 8

        self.user_name = input("Please input user name: ")
        self.log_path = f"logs/{self.user_name}_{time.strftime('%Y-%m-%d_%H-%M-%S', time.localtime())}"
        os.mkdir(self.log_path)

        self.moment_menager = MomentManager(port=moment_port)
        self.axo_controller = AxoController(port=axo_port)
        self.isPlot = isPlot
        if self.isPlot:
            self.info_plottor = InfoPlottor(axo_controller=self.axo_controller, log_path=self.log_path)
            self.info_plottor.open_plot_info()

        self.safe_cur_mom_raw = []
        self.safe_cur_mom = None

        self.runtime_record = []

        self.isStimulate = isStimulate
        if isStimulate:
            self.stimulator = StimulateController()

    def record_safe_info(self, cycle_num: int):
        self.axo_controller.enter_control_mode()
        time.sleep(4)
        self.axo_controller.exit_control_mode()

        self.axo_controller.change_control_mode("velocity")
        self.axo_controller.set_all_motors_vel([0, 0, 0, 0])
        self.axo_controller.enter_control_mode(isChangeMode=False)
        self.axo_controller.set_all_motors_pos_vel_based_sync(self.trajectory[0][0])

        print(f"[info]: start all control, time is {time.time()}")

        if self.isStimulate:
            self.stimulator.start_stimulate()

        with tqdm(total=cycle_num) as pbar:
            for _ in range(cycle_num):
                start_time = time.time()
                self.run_one_cycle(callback_func=self.record_callback)
                pbar.update(1)
                print(f"cycle time: {(time.time() - start_time)}")

        if self.isStimulate:
            self.stimulator.stop_stimulate()

        self.process_raw_data()

        if self.axo_controller.in_control_mode:
            self.axo_controller.exit_control_mode()
        self.axo_controller.change_control_mode("position")

    def record_callback(self, indx: int) -> bool:
        current = self.axo_controller.get_leg_current()
        moments = self.moment_menager.get_all_moments()[0]
        safe_cur_moment = np.concatenate((np.array([indx]), np.array(current), np.array(moments)))

        assert isinstance(self.safe_cur_mom_raw, list)
        self.safe_cur_mom_raw.append(safe_cur_moment)

        return True

    def process_raw_data(self):
        self.safe_cur_mom_raw = np.array(self.safe_cur_mom_raw)
        assert self.safe_cur_mom_raw.shape[0] > 0
        np.save(f"{self.log_path}/safe_cur_mom_raw.npy", self.safe_cur_mom_raw)

        max_indx = int(max(self.safe_cur_mom_raw[:, 0]) + 1)
        indx_set = [[] for i in range(max_indx)]
        for i in range(self.safe_cur_mom_raw.shape[0]):
            indx_set[int(self.safe_cur_mom_raw[i, 0])].append(i)

        cur_len = (self.safe_cur_mom_raw.shape[1] - 1) // 2
        safe_cur_mom = []
        for indx in indx_set:
            if len(indx) == 0:
                print("[error]: exist empty moment in some indx")

            raw_cur_mom = self.safe_cur_mom_raw[indx, 1:]  # type: ignore
            mean = np.mean(raw_cur_mom, axis=0)
            std = np.std(raw_cur_mom, axis=0)
            # std = np.min(std, )
            std[:cur_len] = np.clip(std[:cur_len], 0.3, None)  # clip current std
            std[cur_len:] = np.clip(std[cur_len:], 1, None)  # clip moment std

            safe_cur_mom.append(np.hstack((mean, std)))

        safe_cur_mom = np.array(safe_cur_mom)
        for i in range(safe_cur_mom.shape[1]):
            safe_cur_mom[:, i] = signal.savgol_filter(safe_cur_mom[:, i], 51, 3)

        self.safe_cur_mom = safe_cur_mom

        np.save(f"{self.log_path}/safe_cur_mom.npy", self.safe_cur_mom)

    def run_one_cycle(self, callback_func: Callable):
        traj_pos, traj_vel = self.trajectory
        one_step_time = self.one_cycle_time / (len(traj_pos) - 1)
        for idx, (target_pos, target_vel) in enumerate(zip(traj_pos, traj_vel)):
            start_time = time.time()
            if not self.axo_controller.in_control_mode:
                break

            if self.isPlot:
                self.info_plottor.set_target_pos(target_pos)

            if not callback_func(idx):
                break

            self.axo_controller.set_all_motors_pos_vel_based(target_pos, target_vel)

            # assert time.time() - start_time <= one_step_time, f"one step time is too short, {time.time() - start_time} > {one_step_time}"
            if time.time() - start_time < one_step_time:
                time.sleep(one_step_time - (time.time() - start_time))

    def run_one_cycle_new(self, callback_func: Callable):
        cycle_start_time = time.time()
        traj_pos, traj_vel = self.trajectory
        one_step_time = self.one_cycle_time / (len(traj_pos) - 1)
        next_step_time = cycle_start_time + one_step_time

        # Axo currently on the first step, so we start from the second step
        for idx, (target_pos, target_vel) in enumerate(zip(traj_pos[1:], traj_vel[1:])):
            if not self.axo_controller.in_control_mode:
                # TODO: should we break here? or raise an exception?
                break

            if self.isPlot:
                self.info_plottor.set_target_pos(target_pos)

            if not callback_func(idx):
                # TODO: the same as above
                break

            self.axo_controller.set_all_motors_pos_vel_based(target_pos, target_vel)

            assert time.time() < next_step_time, f"one step time is too short, {time.time()} > {next_step_time}"
            # accurate_delay((next_step_time - time.time()) * 1000)
            # print(f"step time: {(time.time() - cycle_start_time)}")
            time.sleep(next_step_time - time.time())
            next_step_time += one_step_time

    def empty_callback(self, indx: int) -> bool:
        return True

    def detect_callback(self, indx: int) -> bool:
        moments = self.moment_menager.get_all_moments()[0]
        current = self.axo_controller.get_leg_current()
        robot_info = np.hstack((np.array([indx]), current, moments))
        self.runtime_record.append(robot_info)  # type: ignore
        robot_info = robot_info[1:]
        assert self.safe_cur_mom is not None

        factor = self.factor
        mean_nums = self.safe_cur_mom.shape[1] // 2

        low_limitation = self.safe_cur_mom[indx, :mean_nums] - factor * self.safe_cur_mom[indx, mean_nums:]
        high_limitation = self.safe_cur_mom[indx, :mean_nums] + factor * self.safe_cur_mom[indx, mean_nums:]

        if np.any(robot_info < low_limitation) or np.any(robot_info > high_limitation):
            which_motor = np.where((robot_info < low_limitation) | (robot_info > high_limitation))
            which_motor = which_motor[0].tolist()
            print(f"[error]: moment or current is unsafe, pos_idx: {which_motor}")
            print(f"         moments or current: {robot_info[which_motor]}, low_limitation: {low_limitation[which_motor]}, high_limitation: {high_limitation[which_motor]}")
            self.axo_controller.exit_control_mode()
            return False

        return True

    def run_with_detection(self, cycle_num: int):
        self.axo_controller.enter_control_mode()
        time.sleep(4)  # wait for the robot reset to the initial position

        self.axo_controller.exit_control_mode()
        self.axo_controller.change_control_mode("velocity")
        self.axo_controller.set_all_motors_vel([0, 0, 0, 0])
        self.axo_controller.enter_control_mode(isChangeMode=False)
        self.axo_controller.set_all_motors_pos_vel_based_sync(self.trajectory[0][0])

        if self.isStimulate:
            self.stimulator.start_stimulate()

        with tqdm(total=cycle_num) as pbar:
            for _ in range(cycle_num):
                if not self.axo_controller.in_control_mode:
                    break

                self.run_one_cycle(self.detect_callback)
                pbar.update(1)

        if self.isStimulate:
            self.stimulator.stop_stimulate()

        if self.axo_controller.in_control_mode:
            self.axo_controller.exit_control_mode()
        self.axo_controller.change_control_mode("position")

    def plot_safe_cur_mom(self):
        assert self.safe_cur_mom is not None
        mean_nums = self.safe_cur_mom.shape[1] // 2
        motor_name = ["Left Hip", "Left Knee", "Right Hip", "Right Knee"]

        self.runtime_record = np.array(self.runtime_record)

        np.save(f"{self.log_path}/detected_info.npy", self.runtime_record)

        if self.runtime_record.shape[0] == 0:
            print("[error]: no data in runtime_record")
            return

        plot_data = self.runtime_record[-100:, :]

        mean_std = []
        for i in range(plot_data.shape[0]):
            mean_std.append(self.safe_cur_mom[int(plot_data[i, 0]), :])
        mean_std = np.array(mean_std)
        print(mean_std.shape)

        for i in range(mean_nums):
            plt.subplot(241 + i)
            plt.plot(mean_std[:, i], "--")
            plt.fill_between(range(mean_std.shape[0]), mean_std[:, i] - self.factor * mean_std[:, i + mean_nums], mean_std[:, i] + self.factor * mean_std[:, i + mean_nums], alpha=0.2)  # type: ignore
            plt.plot(plot_data[:, i + 1])

            plt.title(motor_name[i % 4])
            if i == 4:
                plt.ylabel("Moment")

            if i == 0:
                plt.ylabel("Current")

        plt.show()

    def close(self):
        if self.isPlot:
            self.info_plottor.close_plot_info()

        self.axo_controller.close_controller()
        self.plot_safe_cur_mom()
