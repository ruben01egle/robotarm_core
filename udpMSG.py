import socket
import struct
import time
from enum import IntEnum


class MessageType(IntEnum):
    HEARTBEAT      = 0x01
    SYSTEM_COMMAND = 0x02
    SYSTEM_LOG     = 0x03
    MSG_STATUS     = 0x04
    TRAJECTORY     = 0x05
    TELEMETRY      = 0x06
    CANFD_TUNNEL   = 0x07

class SystemMode(IntEnum):
    IDLE           = 0x01
    MOTION         = 0x02
    CONFIG         = 0x03
    ERROR          = 0x66
    EMERGENCY_HALT = 0xFF

class SystemCMD(IntEnum):
    NO_ACTION      = 0x01
    START_MOTION   = 0x02
    STOP_MOTION    = 0x03
    START_CONFIG   = 0x04
    STOP_CONFIG    = 0x05
    REBOOT         = 0x06
    EMERGENCY_HALT = 0xFF

UDP_MAGIC_BYTE = 0x112233FF

class RobotUDPClient:
    def __init__(self, ip, port):
        self.target_ip = ip
        self.target_port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _pack_header(self, msg_type, mode):
        """Packt den CPacketHeader (4 * uint32_t)"""
        timestamp = int(int(time.time()) & 0xFFFFFFFF)
        # Format: I=uint32 (4 Byte), insgesamt 16 Bytes
        return struct.pack("<IIII", UDP_MAGIC_BYTE, msg_type, timestamp, mode)

    def send_heartbeat(self, mode=SystemMode.IDLE):
        """Sendet Heartbeat (Header + TrajectoryData Platzhalter)"""
        header = self._pack_header(MessageType.HEARTBEAT, mode)
        self.sock.sendto(header, (self.target_ip, self.target_port))
        print(f"Sent HEARTBEAT in mode {mode.name}")

    def send_command(self, cmd: SystemCMD, msg_id: int, mode=SystemMode.IDLE):
        """Sendet CSystemCMDMsg (Header + uint32 + uint32)"""
        header = self._pack_header(MessageType.SYSTEM_COMMAND, mode)
        payload = struct.pack("<II", msg_id, cmd)
        self.sock.sendto(header + payload, (self.target_ip, self.target_port))
        print(f"Sent COMMAND {cmd.name} (ID: {msg_id})")

    def send_canfd_msg(self, msg_id: int, data: bytes, mode=SystemMode.IDLE):
        """Sendet CCANFDMsg (Header + uint32 + uint32 + 64*uint8)"""
        if len(data) > 64:
            raise ValueError("CANFD data too long (max 64 bytes)")

        header = self._pack_header(MessageType.CANFD_TUNNEL, mode)
        payload_meta = struct.pack("<II", msg_id, len(data))
        payload_data = data.ljust(64, b'\x00')
        
        self.sock.sendto(header + payload_meta + payload_data, (self.target_ip, self.target_port))
        print(f"Sent CANFD Tunnel (ID: {hex(msg_id)}, Len: {len(data)})")

# --- Beispiel Anwendung ---

if __name__ == "__main__":
    # Konfiguration
    ROBOT_IP = "192.168.0.200"
    PORT = 1234
    
    client = RobotUDPClient(ROBOT_IP, PORT)

    # 1. Heartbeat senden
    client.send_heartbeat(SystemMode.IDLE)

    # 2. System Command senden (z.B. Reboot)
    client.send_command(SystemCMD.REBOOT, msg_id=1)
    client.send_command(SystemCMD.REBOOT, msg_id=2)

    # 3. CAN-FD Nachricht tunneln
    can_payload = bytes([0xDE, 0xAD, 0xBE, 0xEF])
    client.send_canfd_msg(msg_id=0x123, data=can_payload)