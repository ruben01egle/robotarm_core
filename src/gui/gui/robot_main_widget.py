from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QScrollArea, QFrame, QSizePolicy,
                             QHBoxLayout, QLabel, QMessageBox, QPushButton, QStackedWidget, QTextEdit)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QKeySequence, QShortcut, QTextCursor

from .telemetry_widget import TelemetryDashboard
from .joint_jog_widget import JointJogWidget
from .cartesian_jog_widget import CartesianJogWidget
from .compliance_widget import ComplianceWidget
from .program_widget import MOTION_SERVER, ProgramWidget
from .pose_bar_widget import PoseBar
from .stiffness_widget import StiffnessWidget
from .controller_modes import COMPLIANT
from .control_header_widget import ControlHeader

NAV_BUTTON_STYLE = """
    QPushButton {
        background-color: #3d3d3d; color: white;
    }
    QPushButton:checked {
        background-color: #27ae60;  /* green for the active mode */
        border: 2px solid #2ecc71;
    }
    QPushButton:disabled { color: #7f8c8d; }
    QPushButton:checked:disabled { color: #dfe6e9; }
"""

class RobotMainWindow(QMainWindow):
    def __init__(self, guiDataStore, node):
        super().__init__()
        self.setWindowTitle("Robot Arm Control Center")
        self.data_store = guiDataStore
        self.node = node

        # --- MAIN LAYOUT ---
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        # 1. TOP: header and the always visible position bar
        self.control_header = ControlHeader(self.data_store, self.node)
        self.pose_bar = PoseBar(self.data_store)

        # 2. MIDDLE: content side by side
        self.content_layout = QHBoxLayout()
        
        # LEFT: manual control (the control stack)
        self.init_nav_bar()
        self.left_column_container = QWidget()
        self.left_column_container.setFixedWidth(400) # width of the whole column
        self.left_column_layout = QVBoxLayout(self.left_column_container)
        self.left_column_layout.setContentsMargins(0, 0, 0, 0)

        self.control_stack = QStackedWidget()
        
        self.joint_page = JointJogWidget(self.data_store, self.node)
        self.cartesian_page = CartesianJogWidget(self.data_store, self.node)
        self.compliance_page = ComplianceWidget(self.data_store, self.node)
        self.pages = [self.joint_page, self.cartesian_page, self.compliance_page]
        
        for page in self.pages:
            self.control_stack.addWidget(page)

        # Stiffness: one shared panel below the stack, since navigation is locked during a mission
        self.stiffness_panel = StiffnessWidget(self.data_store, self.node)

        # Scroll page + panel if the height is not enough (joint page + expanded panel)
        scroll_content = QWidget()
        # Never wider than the viewport (no horizontal scrolling); long content is wrapped or cut
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

        # Shown while a program runs: manual control is locked then
        self.program_lock_label = QLabel("Program running: manual control locked")
        self.program_lock_label.setWordWrap(True)
        self.program_lock_label.setStyleSheet("""
            background-color: #3b2f0a; color: #f1c40f; font-weight: bold;
            border: 1px solid #f1c40f; border-radius: 6px; padding: 6px;
        """)
        self.program_lock_label.hide()

        self.left_column_layout.addWidget(self.nav_container)
        self.left_column_layout.addWidget(self.program_lock_label)
        self.left_column_layout.addWidget(self.left_scroll)
        self.fit_stack_to_page()

        # RIGHT: workspace, MONITOR (telemetry) or PROGRAM (MoveIt goals and programs)
        self.dashboard = TelemetryDashboard(self.data_store)
        self.program_page = ProgramWidget(self.data_store, self.node)
        self.init_workspace()

        # Assemble the middle
        self.content_layout.addWidget(self.left_column_container)
        self.content_layout.addWidget(self.workspace_container, stretch=1)

        self.main_layout.addWidget(self.control_header, stretch=1)
        self.main_layout.addWidget(self.pose_bar)
        self.main_layout.addLayout(self.content_layout, stretch=4)

        # 3. BOTTOM: log console
        self.init_log_console()

        self.resize(1400, 1000)
    
        # Timer (~30 FPS for smooth plots)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.global_update)
        self.update_timer.start(33)

        # Space: teach in the PROGRAM tab (the widget ignores it while typing or not visible)
        self.teach_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Space), self)
        self.teach_shortcut.activated.connect(self.program_page.shortcut_teach)

        self.log_message("System initialized. Dashboard and Controls ready.")

    def init_workspace(self):
        """Right side: tab buttons MONITOR | PROGRAM above a stack with the two workspaces."""
        self.workspace_container = QWidget()
        layout = QVBoxLayout(self.workspace_container)
        layout.setContentsMargins(0, 0, 0, 0)
        tabs = QHBoxLayout()
        tabs.setContentsMargins(0, 0, 0, 4)
        self.btn_monitor = QPushButton("MONITOR")
        self.btn_program = QPushButton("PROGRAM")
        self.workspace_buttons = [self.btn_monitor, self.btn_program]
        for index, btn in enumerate(self.workspace_buttons):
            btn.setCheckable(True)
            btn.setAutoExclusive(True)
            btn.setMinimumHeight(40)
            btn.setStyleSheet(NAV_BUTTON_STYLE)
            btn.clicked.connect(lambda checked, i=index: self.switch_workspace(i))
            tabs.addWidget(btn)
        self.btn_monitor.setChecked(True)
        layout.addLayout(tabs)
        self.workspace_stack = QStackedWidget()
        self.workspace_stack.addWidget(self.dashboard)
        self.workspace_stack.addWidget(self.program_page)
        layout.addWidget(self.workspace_stack, stretch=1)

    def switch_workspace(self, index):
        self.workspace_stack.setCurrentIndex(index)
        self.log_message(f"Workspace changed to: {['MONITOR', 'PROGRAM'][index]}")

    def init_nav_bar(self):
        """Create the bar that switches the left control page."""
        self.nav_container = QWidget()
        # Side by side instead of stacked: saves height for page + stiffness panel
        self.nav_layout = QHBoxLayout(self.nav_container)
        self.nav_layout.setContentsMargins(0, 0, 0, 4)
        
        self.btn_joint_jog = QPushButton("JOINT")
        self.btn_cartesian_jog = QPushButton("CARTESIAN")
        self.btn_compliance = QPushButton("COMPLIANCE")
        self.nav_buttons = [self.btn_joint_jog, self.btn_cartesian_jog, self.btn_compliance]

        for idx, btn in enumerate(self.nav_buttons):
            btn.setCheckable(True)
            btn.setAutoExclusive(True)
            btn.setMinimumHeight(40)
            btn.setStyleSheet(NAV_BUTTON_STYLE)
            # default argument i=idx binds the index for the lambda
            btn.clicked.connect(lambda checked, i=idx: self.switch_gui_mode(i))
            self.nav_layout.addWidget(btn)
        self.btn_joint_jog.setChecked(True)
        self.main_layout.addWidget(self.nav_container)

    def init_log_console(self):
        """Create the log console at the bottom."""
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
        """Switch the left control stack only."""
        self.control_stack.setCurrentIndex(index)
        self.fit_stack_to_page()
        # On the compliance page the stiffness panel is the main content
        self.stiffness_panel.set_expanded(self.control_stack.currentWidget() is self.compliance_page)
        modes = ["JOINT", "CARTESIAN", "COMPLIANCE"]
        self.log_message(f"Gui Mode changed to: {modes[index]}")

    def fit_stack_to_page(self):
        """Otherwise QStackedWidget takes the height of the largest page and the panel ends up far below."""
        for page in self.pages:
            policy = QSizePolicy.Policy.Preferred if page is self.control_stack.currentWidget() else QSizePolicy.Policy.Ignored
            page.setSizePolicy(QSizePolicy.Policy.Preferred, policy)
            page.adjustSize()

    def update_stiffness_visibility(self):
        """Show the panel only on the compliance page and when mode Compliant is selected."""
        page = self.control_stack.currentWidget()
        visible = page is self.compliance_page or page.execution_mode() == COMPLIANT
        if visible == self.stiffness_panel.isHidden():
            self.stiffness_panel.setVisible(visible)

    def global_update(self):
        """Central timer slot. The dashboard updates all its tiles itself."""
        self.dashboard.update_all()
        self.joint_page.update_widget()
        self.cartesian_page.update_widget()
        self.compliance_page.update_widget()
        self.program_page.update_widget()
        self.pose_bar.update_widget()
        self.stiffness_panel.update_widget()
        self.update_stiffness_visibility()
        self.control_header.update_status()
        self.update_nav_lock()
        self.log_update()

    def update_nav_lock(self):
        """
        The control page cannot be changed while a mission is active; while a program runs
        (mission of the motion_server) manual control is locked completely.
        """
        owner = self.data_store.get_status()["active_node"]
        for btn in self.nav_buttons:
            btn.setEnabled(not owner)
        program_running = owner == MOTION_SERVER
        if program_running == self.left_scroll.isEnabled():
            self.left_scroll.setEnabled(not program_running)
            self.program_lock_label.setVisible(program_running)

    def closeEvent(self, event):
        if self.program_page.dirty:
            answer = QMessageBox.question(
                self, "Unsaved program",
                "The program in the PROGRAM tab has unsaved changes. Close anyway?")
            if answer != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
        # Safety: send a zero twist now instead of waiting for the controller's 250 ms watchdog
        self.cartesian_page.shutdown()
        super().closeEvent(event)

    def log_update(self):
        new_logs = self.data_store.get_new_logs()
        for log_entry in new_logs:
            self.log_console.append(log_entry)
            self.log_console.moveCursor(QTextCursor.MoveOperation.End)

    # --- Logging ---
    def log_message(self, message):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_console.append(f"[{ts}] {message}")
        self.log_console.moveCursor(QTextCursor.MoveOperation.End)
