import sys
import math
import time

from simple_pid.PID import PID

sys.path.append("..")
from AxoController import AxoController  # noqa: E402


def main():
    port = "com3"
    axo = AxoController(port=port, angle_telorance=[10, 50, 65, 3])
    axo.enter_control_mode()
    # axo.change_control_mode("position")
    # axo.set_all_motors_pos_sync([0, 0, 0, 0])
    axo.change_control_mode("velocity")

    try:
        print("Start")
        for i in range(2000):
            axo.set_all_motors_pos_vel_based([20, 0, 0, 0], [0, 0, 0, 0])

            time.sleep(0.05)
    finally:
        axo.exit_control_mode()
        axo.close_controller()


if __name__ == "__main__":
    main()
