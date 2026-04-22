import socket
import select
import threading
import time

class UDPWorker(threading.Thread):
    def __init__(self, remote_ip, remote_port, msg_handler, logging_callback=None):
        super().__init__(daemon=True)
        self.msg_handler = msg_handler
        self.logger = logging_callback
        
        self.remote_peer = (remote_ip, remote_port)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", remote_port))
        self._stop_event = threading.Event()

    def run(self):
        self._log(f"Worker gestartet, Ziel: {self.remote_peer}")
        try:
            while not self._stop_event.is_set():
                # 1. SELECT: Prüfen, ob überhaupt etwas da ist (kurzer Timeout)
                readable, _, _ = select.select([self.sock], [], [], 0.001)

                if readable:
                    # 2. BATCH-READ: Alle verfügbaren Pakete sofort abholen
                    for _ in range(1,20):
                        try:
                            # recvfrom mit MSG_DONTWAIT (oder setblocking(0))
                            # Damit lesen wir so lange, bis keine Daten mehr im Puffer sind
                            self.sock.setblocking(False) 
                            data, addr = self.sock.recvfrom(2048)
                            
                            if addr[0] == self.remote_peer[0]:
                                self.msg_handler.handle_raw_packet(data)
                            
                        except (BlockingIOError, socket.error):
                            # Keine weiteren Daten im Puffer vorhanden
                            break
                        except Exception as e:
                            self._log(f"Recv error: {e}")
                            break
                    
                    # Zurück in den blockierenden Modus für select (sauberer)
                    self.sock.setblocking(True)

                # 3. Danach ausgehende Nachrichten senden
                self._process_outgoing()

                # 4. Kurzes Sleep, um den GIL freizugeben
                # WICHTIG: 0.01 ist okay, aber 0.001 wäre für 1kHz Steuerung besser
                time.sleep(0.001) 
        finally:
            self.sock.close()

    def _process_outgoing(self):
        for _ in range(10):
            raw_msg = self.msg_handler.get_next_outgoing_msg()
            if raw_msg:
                try:
                    self.sock.sendto(raw_msg, self.remote_peer)
                except Exception as e:
                    self._log(f"Send error: {e}")
            else:
                break

    def _log(self, message: str):
        """Hilfsfunktion für internes Logging."""
        print(f"[UDP-Worker] {message}")
        
        if self.logger:
            self.logger(message)