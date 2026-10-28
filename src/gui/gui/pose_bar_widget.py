import math

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QApplication, QFrame, QHBoxLayout, QLabel

# Redraw every n-th call of update_widget (main timer ~30 Hz -> ~10 Hz, easier to read)
REDRAW_EVERY = 3
# [deg] distance to a joint limit at which the value turns amber / red
LIMIT_WARN_DEG = 5.0
LIMIT_ALARM_DEG = 0.5
# [rad/s] above this, the arm counts as moving
MOVING_SPEED = 0.01

TEXT_COLOR = "#ecf0f1"
WARN_COLOR = "#f39c12"
ALARM_COLOR = "#e74c3c"
DIM_COLOR = "#7f8c8d"


class CopyLabel(QLabel):
    """Label that copies its copy_text to the clipboard when clicked."""

    def __init__(self, text=""):
        super().__init__(text)
        self.copy_text = ""
        self.setToolTip("Click to copy")

    def mousePressEvent(self, event):
        if self.copy_text:
            QApplication.clipboard().setText(self.copy_text)
        super().mousePressEvent(event)


class PoseBar(QFrame):
    """
    Always visible current position: joints [deg] and TCP pose [mm, deg] in the base frame.

    Joint values turn amber near a limit and red at it; the dot is lit while the arm moves.
    """

    def __init__(self, store, parent=None):
        super().__init__(parent)
        self.store = store
        self.calls = 0
        self.setObjectName("PoseBar")
        self.setFixedHeight(34)
        self.setStyleSheet(f"""
            #PoseBar {{ background-color: #1f2a35; border-bottom: 1px solid #34495e; }}
            QLabel {{ color: {TEXT_COLOR}; font-family: 'DejaVu Sans Mono', 'Consolas', monospace;
                      font-size: 12px; }}
        """)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(20, 0, 20, 0)
        layout.setSpacing(6)

        self.moving_dot = QLabel("●")
        self.moving_dot.setToolTip("Lit while the arm moves")
        layout.addWidget(self.moving_dot)

        layout.addWidget(self.caption("J"))
        self.joint_labels = []
        for i in range(6):
            label = CopyLabel("—")
            label.setFixedWidth(62)
            label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            label.setToolTip(f"Axis {i + 1} [deg], click to copy all joints")
            self.joint_labels.append(label)
            layout.addWidget(label)
        layout.addWidget(self.caption("°"))

        layout.addSpacing(24)
        self.tcp_caption = self.caption("TCP")
        layout.addWidget(self.tcp_caption)
        self.xyz_label = CopyLabel("—")
        self.xyz_label.setToolTip("TCP position [mm], click to copy")
        layout.addWidget(self.xyz_label)
        layout.addWidget(self.caption("mm"))
        layout.addSpacing(12)
        self.rpy_label = CopyLabel("—")
        self.rpy_label.setToolTip("TCP orientation roll, pitch, yaw [deg], click to copy")
        layout.addWidget(self.rpy_label)
        layout.addWidget(self.caption("° rpy"))
        layout.addStretch()

    @staticmethod
    def caption(text):
        label = QLabel(text)
        label.setStyleSheet(f"color: {DIM_COLOR};")
        return label

    def update_widget(self):
        """Called periodically by the main GUI timer (~30 Hz); redraws at ~10 Hz."""
        self.calls += 1
        if self.calls % REDRAW_EVERY:
            return
        self.update_joints()
        self.update_tcp()

    def update_joints(self):
        if not self.store.has_joint_state():
            for label in self.joint_labels:
                label.setText("—")
            self.moving_dot.setStyleSheet(f"color: {DIM_COLOR};")
            return
        positions = self.store.get_current_positions()
        limits = self.store.get_joint_limits()
        degrees = [math.degrees(q) for q in positions]
        for i, label in enumerate(self.joint_labels):
            if i >= len(degrees):
                label.setText("—")
                continue
            label.setText(f"{degrees[i]:7.1f}")
            color = TEXT_COLOR
            if i < len(limits):
                lower, upper = limits[i][0], limits[i][1]
                margin = math.degrees(min(positions[i] - lower, upper - positions[i]))
                if margin < LIMIT_ALARM_DEG:
                    color = ALARM_COLOR
                elif margin < LIMIT_WARN_DEG:
                    color = WARN_COLOR
            label.setStyleSheet(f"color: {color};")
        all_joints = "[" + ", ".join(f"{d:.2f}" for d in degrees) + "]"
        for label in self.joint_labels:
            label.copy_text = all_joints

        speed = max((abs(v) for v in self.store.get_current_velocities()), default=0.0)
        moving = speed > MOVING_SPEED
        self.moving_dot.setStyleSheet(f"color: {'#2ecc71' if moving else DIM_COLOR};")
        self.moving_dot.setToolTip("Arm is moving" if moving else "Arm is still")

    def update_tcp(self):
        pose = self.store.get_current_tcp_pose()
        frame = self.store.get_base_frame()
        self.tcp_caption.setText(f"TCP ({frame})" if frame else "TCP")
        if pose is None:
            self.xyz_label.setText("—")
            self.rpy_label.setText("—")
            self.xyz_label.copy_text = self.rpy_label.copy_text = ""
            return
        xyz = [v * 1000.0 for v in pose[:3]]
        # unwrapped in the store (for the plots): shown in (-180, 180]
        rpy = [math.degrees(math.remainder(a, 2.0 * math.pi)) for a in pose[3:6]]
        self.xyz_label.setText(" ".join(f"{v:8.1f}" for v in xyz))
        self.rpy_label.setText(" ".join(f"{a:7.1f}" for a in rpy))
        self.xyz_label.copy_text = "[" + ", ".join(f"{v:.2f}" for v in xyz) + "]"
        self.rpy_label.copy_text = "[" + ", ".join(f"{a:.2f}" for a in rpy) + "]"
