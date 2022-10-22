from AxoController import AxoController


def main():
    axo = AxoController(port="com3", open_detection=False)
    axo.enter_control_mode()
    axo.change_control_mode("position")
    axo.set_all_motors_pos_sync([0, 0, 0, 0])
    axo.exit_control_mode()


if __name__ == "__main__":
    main()
