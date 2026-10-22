from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QPushButton, QButtonGroup
from PyQt6.QtCore import pyqtSignal

# Ausführung: positionsgeregelt direkt auf der Hardware oder nachgiebig über den Impedanz-Controller
POSITION = "position"
COMPLIANT = "compliant"

IMPEDANCE_CONTROLLER = "cartesian_impedance_controller"
GC_CONTROLLER = "gc_controller"

# Gleiche Controller-Klasse, *_impedance ist auf den cartesian_impedance_controller gechained.
# Topics/Services liegen relativ zum Controller-Namen.
JOINT_STREAM_CONTROLLERS = {
    POSITION: "joint_stream_controller",
    COMPLIANT: "joint_stream_controller_impedance",
}
CARTESIAN_JOG_CONTROLLERS = {
    POSITION: "cartesian_jog_controller",
    COMPLIANT: "cartesian_jog_controller_impedance",
}


def controller_set(instances: dict, mode: str) -> list:
    """Controller, die für den Modus gemeinsam (in einem Switch-Request) aktiviert werden."""
    if mode == COMPLIANT:
        return [IMPEDANCE_CONTROLLER, instances[COMPLIANT]]
    return [instances[POSITION]]


def set_available(names: list, loaded) -> bool:
    """loaded = None heißt: controller_manager noch nicht abgefragt -> nichts ausgrauen."""
    return loaded is None or all(name in loaded for name in names)


class ExecutionToggle(QWidget):
    """Umschalter Position | Compliant. Gesperrt, solange die Bewegung freigegeben ist."""
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
                           ("" if self.available[mode] else "\n(not loaded in the controller_manager)"))
        # Nicht verfügbarer Modus gewählt (z.B. Simulation ohne Positions-Controller) -> wechseln
        if not self.locked and not self.available[self.mode()]:
            other = COMPLIANT if self.mode() == POSITION else POSITION
            if self.available[other]:
                self.buttons[other].setChecked(True)
