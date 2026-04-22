import sys
from PyQt6.QtWidgets import QApplication

from qt_gui.robot_main_widget import RobotMainWindow
from qt_gui.telemetry_mockdata_datastore import SimulatorWorker
from qt_gui.data_store import GuiDataStore

# Neue Importe
from src.data_link import DataLink
from src.message_handler import MessageHandler
from src.udp_worker import UDPWorker
from src.robot_controller import RobotController # Pfad anpassen
from src.hardware_simulator import HardwareSimulator

class RobotApplication:
    def __init__(self):
        # 1. Daten-Infrastruktur
        self.gui_data_store = GuiDataStore()
        self.data_link = DataLink()
        
        # 2. Kommunikation
        self.msg_handler = MessageHandler(self.data_link)
        self.udp_worker = UDPWorker(
            remote_ip="127.0.0.1", # IP deines Roboters/STM32
            remote_port=12345, 
            msg_handler=self.msg_handler
        )

        # 3. Logik-Zentrale (State Machine)
        self.controller = RobotController(self.data_link, self.gui_data_store)

        self.hardware_sim = HardwareSimulator("127.0.0.1", 12345)

    def start(self):
        # Alle Hintergrund-Prozesse starten
        self.udp_worker.start()
        self.controller.start()
        self.hardware_sim.start()
        
    def run_gui(self):
        app = QApplication(sys.argv)
        self.window = RobotMainWindow(self.gui_data_store)
        self.window.show()
        
        exit_code = app.exec()
        self.shutdown()
        sys.exit(exit_code)

    def shutdown(self):
        print("Fahre System sauber runter...")
        self.controller.stop()
        # UDP Worker sollte ein stop() haben oder daemon=True sein
        # self.udp_worker.stop()

if __name__ == "__main__":
    robot_app = RobotApplication()
    robot_app.start()
    robot_app.run_gui()