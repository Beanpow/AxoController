import sys
import time

sys.path.append("..")
from AxoController import AxoController  # noqa: E402


def main():
    port = "COM3"
    axo_ctrl = AxoController(port=port, angle_telorance=[20, 60, 80, 5])
    # axo_ctrl.enter_control_mode()

    # axo_ctrl.set_all_motors_pos_sync([10, 0, 10, 0])

    # axo_ctrl.set_all_motors_pos_sync([40, -30, 40, -30])

    # axo_ctrl.exit_control_mode()
    time.sleep(10)
    axo_ctrl.close_controller()


if __name__ == "__main__":
    main()
