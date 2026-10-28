from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QScrollArea, QFrame, QSizePolicy,
                             QHBoxLayout, QPushButton, QStackedWidget, QTextEdit)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QTextCursor

# Deine Widgets importieren
from .telemetry_widget import TelemetryDashboard
from .joint_jog_widget import JointJogWidget
from .cartesian_jog_widget import CartesianJogWidget
from .compliance_widget import ComplianceWidget
from .motion_widget import MotionWidget
from .stiffness_widget import StiffnessWidget
from .controller_modes import COMPLIANT
from .control_header_widget import ControlHeader

class RobotMainWindow(QMainWindow):
    def __init__(self, guiDataStore, node):
        super().__init__()
        self.setWindowTitle("Robot Arm Control Center")
        self.data_store = guiDataStore
        self.node = node

        # --- HAUPT LAYOUT ---
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        # 1. OBEN: Navigation (Haupt-Modi)
        self.control_header = ControlHeader(self.data_store, self.node)

        # 2. MITTE: Side-by-Side Content
        self.content_layout = QHBoxLayout()
        
        # LINKS: Der Control Stack
        self.init_nav_bar()
        self.left_column_container = QWidget()
        self.left_column_container.setFixedWidth(400) # Die Breite gilt nun für die ganze Spalte
        self.left_column_layout = QVBoxLayout(self.left_column_container)
        self.left_column_layout.setContentsMargins(0, 0, 0, 0)

        self.control_stack = QStackedWidget()
        
        self.joint_page = JointJogWidget(self.data_store, self.node)
        self.cartesian_page = CartesianJogWidget(self.data_store, self.node)
        self.compliance_page = ComplianceWidget(self.data_store, self.node)
        self.motion_page = MotionWidget(self.data_store, self.node)
        self.pages = [self.joint_page, self.cartesian_page, self.compliance_page, self.motion_page]
        
        for page in self.pages:
            self.control_stack.addWidget(page)

        # Steifigkeit: ein gemeinsames Panel unter dem Stack, da die Navigation während einer Mission gesperrt ist
        self.stiffness_panel = StiffnessWidget(self.data_store, self.node)

        # Seite + Panel scrollen, falls die Höhe nicht reicht (Joint-Seite + ausgeklapptes Panel)
        scroll_content = QWidget()
        # Nie breiter als der Viewport (kein horizontales Scrollen), lange Inhalte werden umbrochen/abgeschnitten
        scroll_content.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.addWidget(self.control_stack)
        scroll_layout.addWidget(self.stiffness_panel)
        scroll_layout.addStretch()

        self.left_scroll = QScrollArea()
        self.left_scroll.setWidgetResizable(True)
        self.left_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.left_scroll.setWidget(scroll_content)

        self.left_column_layout.addWidget(self.nav_container) 
        self.left_column_layout.addWidget(self.left_scroll)
        self.fit_stack_to_page()
        
        # RECHTS: Das neue, selbstverwaltete Dashboard
        self.dashboard = TelemetryDashboard(self.data_store)

        # Zusammenfügen der Mitte
        self.content_layout.addWidget(self.left_column_container)
        self.content_layout.addWidget(self.dashboard, stretch=1)

        self.main_layout.addWidget(self.control_header, stretch=1)
        self.main_layout.addLayout(self.content_layout, stretch=4)

        # 3. UNTEN: Log-Konsole
        self.init_log_console()

        self.resize(1400, 1000)
    
        # Timer (ca. 30 FPS für flüssige Plots)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.global_update)
        self.update_timer.start(33)

        self.log_message("System initialized. Dashboard and Controls ready.")

    def init_nav_bar(self):
        """Erstellt die obere Leiste zum Umschalten der linken Control-Seite."""
        self.nav_container = QWidget()
        # Nebeneinander statt untereinander, spart Höhe für Seite + Steifigkeits-Panel
        self.nav_layout = QHBoxLayout(self.nav_container)
        self.nav_layout.setContentsMargins(0, 0, 0, 4)
        
        self.btn_joint_jog = QPushButton("JOINT")
        self.btn_cartesian_jog = QPushButton("CARTESIAN")
        self.btn_compliance = QPushButton("COMPLIANCE")
        self.btn_moveit = QPushButton("MOVEIT")
        self.nav_buttons = [self.btn_joint_jog, self.btn_cartesian_jog, self.btn_compliance, self.btn_moveit]
        
        for idx, btn in enumerate(self.nav_buttons):
            btn.setCheckable(True)
            btn.setAutoExclusive(True)
            btn.setMinimumHeight(40)
            btn.setStyleSheet("""
                    QPushButton { 
                        background-color: #3d3d3d; color: white; 
                    }
                    QPushButton:checked { 
                        background-color: #27ae60;  /* Ein schönes Grün für den aktiven Modus */
                        border: 2px solid #2ecc71;
                    }
                    QPushButton:disabled { color: #7f8c8d; }
                    QPushButton:checked:disabled { color: #dfe6e9; }
                """)
            # Lambda nutzt hier den Default-Parameter i=idx, um den Scope zu fixieren
            btn.clicked.connect(lambda checked, i=idx: self.switch_gui_mode(i))
            self.nav_layout.addWidget(btn)
        self.btn_joint_jog.setChecked(True)
        self.main_layout.addWidget(self.nav_container)

    def init_log_console(self):
        """Erstellt die Konsole am unteren Rand."""
        self.log_console = QTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setMaximumHeight(120) 
        self.log_console.setMinimumHeight(60)
        self.log_console.setStyleSheet("""
            background-color: #1e1e1e; 
            color: #d4d4d4; 
            font-family: 'Consolas', monospace;
            font-size: 11px;
            border-top: 2px solid #3d3d3d;
        """)
        self.main_layout.addWidget(self.log_console, stretch=0)

    def switch_gui_mode(self, index):
        """Schaltet nur den linken Control-Stack um."""
        self.control_stack.setCurrentIndex(index)
        self.fit_stack_to_page()
        # Auf der Compliance-Seite ist das Steifigkeits-Panel der Hauptinhalt
        self.stiffness_panel.set_expanded(self.control_stack.currentWidget() is self.compliance_page)
        modes = ["JOINT", "CARTESIAN", "COMPLIANCE", "MOVEIT"]
        self.log_message(f"Gui Mode changed to: {modes[index]}")

    def fit_stack_to_page(self):
        """QStackedWidget nimmt sonst die Höhe der größten Seite an, das Panel stünde weit unten."""
        for page in self.pages:
            policy = QSizePolicy.Policy.Preferred if page is self.control_stack.currentWidget() else QSizePolicy.Policy.Ignored
            page.setSizePolicy(QSizePolicy.Policy.Preferred, policy)
            page.adjustSize()

    def update_stiffness_visibility(self):
        """Panel nur auf der Compliance-Seite und bei gewähltem Modus Compliant."""
        page = self.control_stack.currentWidget()
        visible = page is self.compliance_page or page.execution_mode() == COMPLIANT
        if visible == self.stiffness_panel.isHidden():
            self.stiffness_panel.setVisible(visible)

    def global_update(self):
        """Zentraler Timer-Aufruf. Das Dashboard aktualisiert alle seine Tiles selbst."""
        self.dashboard.update_all()
        self.joint_page.update_widget()
        self.cartesian_page.update_widget()
        self.compliance_page.update_widget()
        self.motion_page.update_widget()
        self.stiffness_panel.update_widget()
        self.update_stiffness_visibility()
        self.control_header.update_status()
        self.update_nav_lock()
        self.log_update()

    def update_nav_lock(self):
        """Solange eine Mission aktiv ist, darf die Control-Seite nicht gewechselt werden."""
        mission_active = bool(self.data_store.get_status()["active_node"])
        for btn in self.nav_buttons:
            btn.setEnabled(not mission_active)

    def closeEvent(self, event):
        # Sicherheit: Null-Twist senden, da der Cartesian-Controller (noch) keinen Timeout hat
        self.cartesian_page.shutdown()
        super().closeEvent(event)

    def log_update(self):
        new_logs = self.data_store.get_new_logs()
        for log_entry in new_logs:
            self.log_console.append(log_entry)
            self.log_console.moveCursor(QTextCursor.MoveOperation.End)

    # --- Event Handler für Logging ---
    def log_message(self, message):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_console.append(f"[{ts}] {message}")
        self.log_console.moveCursor(QTextCursor.MoveOperation.End)
