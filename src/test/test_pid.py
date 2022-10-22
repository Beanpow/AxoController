# -*- coding: utf-8 -*-
# -*- coding: utf-8 -*-
import numpy as np
from scipy.integrate import ode
import matplotlib.pyplot as plt
import sys
import time

sys.path.append("..")
from utils import PID


# -*- coding: utf-8 -*-
# 二阶弹簧阻尼系统
class MassSpringDamper(object):  # 系统的输出是当前位置, 输入是外部力
    def __init__(self, m=1.0, b=10.0, k=20.0, f=0.0):
        self.m, self.b, self.k, self.f = m, b, k, f  # 元组赋值

    def function(self, t, x_state):  # 注意，使用ode顺序不同， 且必须返回列表或者数组，不能是元组
        x = x_state[0]
        v = x_state[1]

        dx = v
        dv = (self.f - self.k * x - self.b * v) / self.m
        return [dx, dv]  # 注意，使用ode必须返回列表或者数组，不能是元组


# 整个系统模拟函数
def pid_control_system(kp, ki, kd, dt=0.01, end_time=3.0, target=3.0):  # 目标位置
    # 初始状态
    x0_state = np.array([0.0, 0.0])
    t0 = 0.0

    # 二阶系统
    sys = MassSpringDamper()  # 形参参见system类

    # 控制器
    pid = PID(kp, ki, kd, dt)

    # 创建ode对象
    r = ode(sys.function)

    # 设置积分器相关参数
    r.set_integrator('vode', method='bdf')

    # 设定系统初始状态和时间
    r.set_initial_value(x0_state, t0)  # t0默认0.0

    # 列表用于保存数据
    t = [0]
    result = [x0_state]
    F_arr = [0]

    # 开始
    while r.successful() and r.t + dt < end_time:
        # 积分计算
        r.integrate(r.t + dt)  # 返回状态向量(列表) r.y = [x, v]， system类中的函数返回值是列表
        time.sleep(dt)

        # 控制器输出
        pid.setpoint = target
        F = pid(r.y[0])

        # 更新系统的外控制力
        sys.f = F

        # 列表用于保存数据
        print('时间: %s, 状态: %s, 外控制力: %s' % (r.t, r.y[0], F))
        t.append(r.t)  # 一维列表
        result.append(r.y)  # 二维列表
        F_arr.append(F)  # 一维列表

    # 再转回numpy数组中, 方便可视化
    result_np = np.array(result)  # 二维数组
    t_np = np.array(t)  # 一维数组
    F_arr_np = np.array(F_arr)  # 一维数组

    return [t_np, F_arr_np, result_np]  # 用列表返回多个值，元组也可，习惯列表


# 绘图可视化
def PlotFigure(t, F, x_state):
    plt.figure(figsize=(12, 8))
    plt.plot(t, x_state[:, 0], color='green', label='position', linewidth=2)
    plt.plot(t, x_state[:, 1], color='red', label='velocity', linewidth=2)
    # plt.plot(t, F, color='red', label='Force', linewidth=2)
    plt.title('PID_Control_State or F')
    plt.xlabel('time/s')
    plt.ylabel('x and v or F')
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == '__main__':

    # 运行主程序
    [t, F_arr, result] = pid_control_system(200.0, 0, 20, 0.02)  # PID控制器参数

    # 提示
    print('_______________________________________')
    print('程序自身在运行')
    print('控制力的终值：', F_arr[-1])
    print(result.shape)
    print('_______________________________________')

    # 绘图
    PlotFigure(t, F_arr, result)

else:
    print('我是被引入')
