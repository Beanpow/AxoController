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
    axo.change_control_mode("position")
    axo.set_all_motors_pos_sync([0, 0, 0, 0])
    axo.change_control_mode("velocity")

    max_vel = 500

    pid = PID(60, 0, 1, 0)

    try:
        print("Start")
        for i in range(2000):
            current_value = axo.get_leg_pos()[2]
            pid.setpoint = math.sin(i / 20) * 20 + 15
            axo.target_pos = [0, 0, pid.setpoint, 0]
            output = pid(current_value)
            assert output is not None
            output = min(max(output, -max_vel), max_vel)
            print(f"control value is :{output}, current value is {current_value}, setpoint is {pid.setpoint}")

            axo.set_all_motors_vel([0, 0, output, 0])
            time.sleep(0.01)
    finally:
        axo.exit_control_mode()
        axo.close_controller()


if __name__ == "__main__":
    main()
