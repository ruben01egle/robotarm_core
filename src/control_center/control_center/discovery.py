import socket
import time

PORT = 6666 
SIGNATURE = b"ROS2"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

while True:
    sock.sendto(SIGNATURE, ('<broadcast>', PORT))
    print(f"Beacon gesendet an Port {PORT}...")
    time.sleep(1)