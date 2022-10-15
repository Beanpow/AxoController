import sys
import time

from simple_pid.PID import PID

sys.path.append("..")
from AxoController import AxoController  # noqa: E402


def main():
    port = "com3"
    axo = AxoController(port=port)
    axo.enter_control_mode()
    axo.change_control_mode("velocity")

    pid = PID(0, 0, 0, 0)

    for i in range(100):
        current_value = axo.get_leg_pos()[0]
        output = pid(current_value)
        assert output is not None
        output = max(min(output, -100), 100)

        axo.set_all_motors_vel([output, 0, 0, 0])
        time.sleep(0.1)

    axo.exit_control_mode()
    axo.close_controller()


if __name__ == "__main__":
    main()
