import socket
import time
import sys
import numpy as np

sys.path.append("..")
from MutliprocessPlot import MutliprocessPlot
from utils import accurate_delay


class Listener:
    def __init__(self, addr: str, port: int) -> None:
        self.addr = addr
        self.port = port
        self.client_socket = socket.socket()  # instantiate
        self.client_socket.connect((addr, port))  # connect to the server
        self.data = []

    def __del__(self):
        self.client_socket.close()

    def listen(self, mutliprocess_plot: MutliprocessPlot) -> None:
        try:
            while True:
                start = time.time()
                self.client_socket.send("MEASUrement:IMMed:VALue?\n".encode())
                data = self.client_socket.recv(1024).decode()  # rece
                mutliprocess_plot.main_conn.send(float(data))
                self.data.append([start, float(data)])
                accurate_delay((0.01 - (time.time() - start)) * 1000)
        except KeyboardInterrupt:
            pass
        finally:
            self.data = np.array(self.data)
            np.save("stimulate" + str(time.time()) + ".npy", self.data)


def main():
    mp = MutliprocessPlot()
    mp.start()
    listener = Listener(addr="192.168.137.28", port=7818)
    listener.listen(mp)


if __name__ == "__main__":
    main()
