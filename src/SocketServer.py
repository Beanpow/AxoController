import socket


class SocketServer:
    def __init__(self, host: str = "127.0.0.1", port: int = 8888):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.create_socket()
        except KeyboardInterrupt:
            self.close_socket()

    def create_socket(self):
        self.sock.bind((self.host, self.port))
        self.sock.listen(5)
        print('[info]: Waiting for connection...')
        self.conn, self.addr = self.sock.accept()
        print(f"[info]: Connected by {self.addr}.")

    def send_data(self, data):
        self.conn.send(data.encode())

    def receive_data(self):
        data = self.conn.recv(1024)
        return data.decode()

    def close_socket(self):
        self.conn.close()
        self.sock.close()
