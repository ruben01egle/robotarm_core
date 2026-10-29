from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QPushButton, QButtonGroup
from PyQt6.QtCore import pyqtSignal

from robotarm_interface.msg import RobotarmMode

# Execution: position controlled directly on the hardware, or compliant through the impedance controller
POSITION = "position"
COMPLIANT = "compliant"

# The firmware mode manager switches the controllers; the GUI only requests modes.
JOINT_STREAM_MODES = {
    POSITION: RobotarmMode.JOINT_STREAM,
    COMPLIANT: RobotarmMode.IMPEDANCE_JOINT_STREAM,
}
CARTESIAN_JOG_MODES = {
    POSITION: RobotarmMode.CARTESIAN_JOG,
    COMPLIANT: RobotarmMode.IMPEDANCE_JOG,
}
# Modes in which the cartesian_impedance_controller runs (its stiffness applies)
IMPEDANCE_MODES = {RobotarmMode.IMPEDANCE, RobotarmMode.IMPEDANCE_JOINT_STREAM,
                   RobotarmMode.IMPEDANCE_JOG}

# Command inputs of the modes: topics and services are relative to these names
IMPEDANCE_CONTROLLER = "cartesian_impedance_controller"
JOINT_STREAM_CONTROLLERS = {
    POSITION: "joint_stream_controller",
    COMPLIANT: "joint_stream_controller_impedance",
}
CARTESIAN_JOG_CONTROLLERS = {
    POSITION: "cartesian_jog_controller",
    COMPLIANT: "cartesian_jog_controller_impedance",
}

MODE_NAMES = {
    getattr(RobotarmMode, name): name
    for name in dir(RobotarmMode)
    if name.isupper() and isinstance(getattr(RobotarmMode, name), int)
}


def mode_name(mode) -> str:
    return MODE_NAMES.get(mode, f"UNKNOWN ({mode})")


def mode_available(mode, available) -> bool:
    """available = None means the firmware has not reported its state yet."""
    return available is not None and mode in available


class ExecutionToggle(QWidget):
    """Position | Compliant toggle. Locked while motion is enabled."""
    mode_changed = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.locked = False
        self.available = {POSITION: True, COMPLIANT: True}

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        label = QLabel("Execution:")
        label.setStyleSheet("color: #95a5a6;")
        layout.addWidget(label)

        self.buttons = {
            POSITION: QPushButton("Position"),
            COMPLIANT: QPushButton("Compliant"),
        }
        self.tooltips = {
            POSITION: "Position controlled, stiff",
            COMPLIANT: "Executed by the cartesian impedance controller:\n"
                       "the TCP follows compliantly, stiffness below",
        }

        self.group = QButtonGroup(self)
        self.group.setExclusive(True)
        for mode, btn in self.buttons.items():
            btn.setCheckable(True)
            btn.setFixedWidth(85)
            btn.setStyleSheet("""
                QPushButton:checked { background-color: #2980b9; color: white; font-weight: bold; }
                QPushButton:disabled { color: #7f8c8d; }
                QPushButton:checked:disabled { background-color: #34607f; color: #dfe6e9; }
            """)
            btn.toggled.connect(lambda checked, m=mode: checked and self.mode_changed.emit(m))
            self.group.addButton(btn)
            layout.addWidget(btn)
        layout.addStretch()

        self.buttons[POSITION].setChecked(True)
        self.refresh()

    def mode(self) -> str:
        return POSITION if self.buttons[POSITION].isChecked() else COMPLIANT

    def set_locked(self, locked: bool):
        if self.locked != locked:
            self.locked = locked
            self.refresh()

    def set_available(self, position: bool, compliant: bool):
        available = {POSITION: position, COMPLIANT: compliant}
        if available != self.available:
            self.available = available
            self.refresh()

    def refresh(self):
        for mode, btn in self.buttons.items():
            btn.setEnabled(not self.locked and self.available[mode])
            btn.setToolTip(self.tooltips[mode] +
                           ("" if self.available[mode] else "\n(not available on the firmware)"))
        # Unavailable mode selected (e.g. simulation without position controller): switch
        if not self.locked and not self.available[self.mode()]:
            other = COMPLIANT if self.mode() == POSITION else POSITION
            if self.available[other]:
                self.buttons[other].setChecked(True)
