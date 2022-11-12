from SocketServer import SocketServer


class StimulateController:
    def __init__(self) -> None:
        self.socket = SocketServer(host="127.0.0.1", port=9999)

    def start_stimulate(self):
        self.socket.send_data("start")

    def stop_stimulate(self):
        self.socket.send_data("stop")
