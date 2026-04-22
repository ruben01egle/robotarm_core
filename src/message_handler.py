import ctypes
from robot_protocol.udp_protocol import (
    MessageType, UDP_MAGIC_BYTE, CPacketHeader,
    CRobotHeartbeatMsg, CSystemCMDMsg, CMsgStatusMsg,
    CSystemLogMsg, CTrajectoryMsg, CTelemetryMsg, CCANFDMsg
)

class MessageHandler:
    def __init__(self, data_link, logging_callback=None):
        """
        Gegenstück zu CMsgHandler.cpp.
        :param data_link: Instanz von DataLink (Shared Memory Äquivalent)
        :param logging_callback: Funktion für Log-Ausgaben (analog zur LogFunc im C++)
        """
        self.data_link = data_link
        self.logger = logging_callback
        
        # Mapping von MessageType auf die entsprechende ctypes-Klasse
        self._msg_type_map = {
            MessageType.HEARTBEAT: CRobotHeartbeatMsg,
            MessageType.SYSTEM_COMMAND: CSystemCMDMsg,
            MessageType.MSG_STATUS: CMsgStatusMsg,
            MessageType.SYSTEM_LOG: CSystemLogMsg,
            MessageType.TRAJECTORY: CTrajectoryMsg,
            MessageType.TELEMETRY: CTelemetryMsg,
            MessageType.CANFD_TUNNEL: CCANFDMsg,
        }

    def _log(self, message: str):
        if self.logger:
            self.logger(message)
        else:
            print(f"[MessageHandler] {message}")

    def handle_raw_packet(self, data: bytes) -> bool:
        """
        Analoge Funktion zu CMsgHandler::handleMsg.
        Verarbeitet Rohdaten vom UDP-Worker.
        """
        # Validierung der Mindestgröße (Header)
        if len(data) < ctypes.sizeof(CPacketHeader):
            self._log("UDP packet too short for header")
            return False

        # Header aus den ersten Bytes extrahieren
        header = CPacketHeader.from_buffer_copy(data[:ctypes.sizeof(CPacketHeader)])

        # Magic Byte Check
        if header.mMagicByte != UDP_MAGIC_BYTE:
            self._log(f"UDP header invalid magic byte: {hex(header.mMagicByte)}")
            return False

        # Die richtige Nachrichtenklasse finden
        msg_class = self._msg_type_map.get(header.mMsgType)
        if not msg_class:
            self._log(f"UDP received unexpected msg type: {header.mMsgType}")
            return False

        # Längenvalidierung für den spezifischen Typ
        if len(data) != ctypes.sizeof(msg_class):
            self._log(f"UDP msg length mismatch for type {header.mMsgType}")
            return False

        # Nachricht instanziieren (entspricht processTypedMsg im C++)
        msg_inst = msg_class.from_buffer_copy(data)
        self._process_msg(msg_inst)
        return True

    def _process_msg(self, msg):
        """
        Sortiert die Nachrichten in die entsprechenden Queues des DataLink ein.
        Entspricht den überladenen processMsg-Funktionen.
        """
        m_type = msg.mHeader.mMsgType

        try:
            if m_type == MessageType.HEARTBEAT:
                self.data_link.heartbeat_in.put_nowait(msg)
            
            elif m_type == MessageType.MSG_STATUS:
                self.data_link.msg_status_in.put_nowait(msg)
            
            elif m_type == MessageType.SYSTEM_LOG:
                # Logs können wir hier direkt verarbeiten oder in die Queue schieben
                info = msg.mInfo.decode('utf-8', 'ignore').strip('\x00')
                self._log(f"STM32 Log: {info}")
                self.data_link.log_in.put_nowait(msg)
            
            elif m_type == MessageType.CANFD_TUNNEL:
                self.data_link.can_fd_in.put_nowait(msg)
            
            elif m_type == MessageType.TELEMETRY:
                self.data_link.telemetry_in.put_nowait(msg)
            #@print("pushed msg to queue")

        except Exception as e:
            self._log(f"Error pushing msg {m_type} to DataLink: {e}")

    def get_next_outgoing_msg(self):
        """
        Gegenstück zu CMsgHandler::getNextOutgoingMsg.
        Gibt bytes zurück oder None, wenn keine Daten da sind.
        """
        msg_obj = None

        # Priorisierung analog zu CMsgHandler.cpp
        if not self.data_link.heartbeat_out.empty():
            msg_obj = self.data_link.heartbeat_out.get_nowait()
        
        elif not self.data_link.msg_status_in.empty(): # Mapping auf entsprechende Queue
            msg_obj = self.data_link.msg_status_in.get_nowait()
            
        elif not self.data_link.log_in.empty():
            msg_obj = self.data_link.log_in.get_nowait()

        elif not self.data_link.can_fd_out.empty():
            msg_obj = self.data_link.can_fd_out.get_nowait()

        elif not self.data_link.telemetry_in.empty():
            msg_obj = self.data_link.telemetry_in.get_nowait()

        # Konvertierung in bytes nur, wenn ein Objekt gefunden wurde
        if msg_obj is not None:
            return bytes(msg_obj)
        
        return None