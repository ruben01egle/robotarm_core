import sys
from PyQt6.QtWidgets import QApplication

from qt_gui.control_center import RobotControlCenter

if __name__ == "__main__":
    app = QApplication(sys.argv)
    

    window = RobotControlCenter()
    window.show()
    
    sys.exit(app.exec())