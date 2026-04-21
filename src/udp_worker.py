import socket
import select
import threading

class UDPWorker(threading.Thread):
    def __init__(self, remote_ip, remote_port, msg_handler, logging_callback=None):
        super().__init__(daemon=True)
        self.msg_handler = msg_handler
        self.logger = logging_callback
        
        self.remote_peer = (remote_ip, remote_port)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop_event = threading.Event()

    def run(self):
        try:
            while not self._stop_event.is_set():
                readable, _, _ = select.select([self.sock], [], [], 0.005)

                if readable:
                    try:
                        data, addr = self.sock.recvfrom(2048)
                        if addr == self.remote_peer:
                            self.msg_handler.handle_raw_packet(data)
                        else:
                            self._log(f"Ignored packet from unknown source: {addr}")
                    except Exception as e:
                        self._log(f"Recv error: {e}")

                self._process_outgoing()
        finally:
            self.sock.close()
            self._log("Socket closed. Thread terminated safely.")

    def _process_outgoing(self):
        for _ in range(10):
            msg = self.msg_handler.get_next_outgoing_msg()
            if msg:
                try:
                    self.sock.sendto(bytes(msg), self.remote_peer)
                except Exception as e:
                    self._log(f"Send error: {e}")
            else:
                break

    def _log(self, message: str):
        """Hilfsfunktion für internes Logging."""
        print(f"[UDP-Worker] {message}")
        
        if self.logger:
            self.logger(message)