import sys
from PyQt6.QtWidgets import QApplication

from qt_gui.robot_main_widget import RobotMainWindow
from qt_gui.telemtry_mockdata import SimulatorWorker
from qt_gui.data_store import GuiDataStore

class RobotApplication:
    def __init__(self):
        # 1. Core-Komponenten (laufen auch ohne GUI)
        self.gui_data_store = GuiDataStore()
        self.worker = SimulatorWorker(self.gui_data_store)

        #self.udp_worker = UDPWorker(self.data_store)
        #self.state_machine = RobotStateMachine(self.udp_worker)
        
    def start(self):
        # Alle Hintergrund-Prozesse starten
        self.worker.start()
        #self.udp_worker.start()
        #self.state_machine.run_async()
        
    def run_gui(self):
        # Erst hier wird die GUI erzeugt und gestartet
        app = QApplication(sys.argv)
        self.window = RobotMainWindow(self.gui_data_store)
        self.window.show()
        
        # Die GUI-Schleife blockiert hier, bis das Fenster geschlossen wird
        exit_code = app.exec()
        
        # NACHDEM die GUI geschlossen wurde, entscheidest du, was passiert:
        self.shutdown()
        sys.exit(exit_code)

    def shutdown(self):
        print("GUI geschlossen, fahre System sauber runter...")
        self.worker.stop()
        #self.state_machine.stop()
        #self.udp_worker.stop()

if __name__ == "__main__":
    robot_app = RobotApplication()
    robot_app.start()
    robot_app.run_gui()