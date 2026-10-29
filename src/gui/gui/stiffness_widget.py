import time

from PyQt6.QtWidgets import (QWidget, QFrame, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
                             QSlider, QDoubleSpinBox, QButtonGroup)
from PyQt6.QtCore import Qt

from robotarm_interface.msg import RobotarmMode

from .controller_modes import IMPEDANCE_CONTROLLER, IMPEDANCE_MODES

# Slider resolution: 0.1 N/m or 0.1 Nm/rad
SLIDER_SCALE = 10
# Without a response to set_parameters_atomically, release the controls again after this time
APPLY_TIMEOUT_S = 5.0

FRAMES = ("base", "tcp")
AXES = ("x", "y", "z")


class StiffnessWidget(QFrame):
    """
    Stiffness of the cartesian_impedance_controller. Two sets of values:
    committed = what the controller uses (data store), staged = what the user is editing.
    Only "Apply now" sends anything (frame and all six values in one set_parameters_atomically).
    """
    def __init__(self, store, node, parent=None):
        super().__init__(parent)
        self.store = store
        self.node = node

        self.committed = None
        self.staged = None
        self.seen_version = -1
        self.apply_started = None
        self.changed_underneath = False

        # group ('linear' / 'angular') -> list per axis
        self.sliders = {"linear": [], "angular": []}
        self.spins = {"linear": [], "angular": []}
        self.dots = {"linear": [], "angular": []}
        self.range_labels = {}

        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setStyleSheet("StiffnessWidget { border: 1px solid #3d3d3d; border-radius: 6px; }")
        self.init_ui()
        self.set_expanded(False)

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 6)
        layout.setSpacing(4)

        # --- HEADER ROW (always visible) ---
        head = QHBoxLayout()
        self.btn_expand = QPushButton()
        self.btn_expand.setCheckable(True)
        self.btn_expand.setFlat(True)
        self.btn_expand.setStyleSheet("font-weight: bold; text-align: left;")
        self.btn_expand.toggled.connect(self.set_expanded)
        head.addWidget(self.btn_expand)

        self.summary_label = QLabel("")
        self.summary_label.setStyleSheet("font-family: monospace; color: #95a5a6; font-size: 11px;")
        self.summary_label.setWordWrap(True)
        head.addWidget(self.summary_label, stretch=1)

        self.pending_label = QLabel("● pending")
        self.pending_label.setStyleSheet("color: #f1c40f; font-weight: bold;")
        self.pending_label.hide()
        head.addWidget(self.pending_label)
        layout.addLayout(head)

        # --- CONTENT (collapsible) ---
        self.body = QWidget()
        body = QVBoxLayout(self.body)
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(4)

        self.info_label = QLabel("")
        self.info_label.setWordWrap(True)
        self.info_label.setStyleSheet("color: #3498db; font-size: 11px;")
        self.info_label.hide()
        body.addWidget(self.info_label)

        # Frame
        frame_row = QHBoxLayout()
        frame_label = QLabel("Frame:")
        frame_label.setStyleSheet("color: #95a5a6;")
        frame_row.addWidget(frame_label)
        self.frame_buttons = {}
        self.frame_group = QButtonGroup(self)
        self.frame_group.setExclusive(True)
        for frame in FRAMES:
            btn = QPushButton(frame.capitalize())
            btn.setCheckable(True)
            btn.setFixedWidth(60)
            btn.setStyleSheet("QPushButton:checked { background-color: #2980b9; color: white; font-weight: bold; }")
            btn.setToolTip("Axes fixed to the robot base" if frame == "base" else "Axes move with the tool")
            btn.toggled.connect(lambda checked, f=frame: checked and self.on_frame_selected(f))
            self.frame_group.addButton(btn)
            self.frame_buttons[frame] = btn
            frame_row.addWidget(btn)
        frame_row.addStretch()
        body.addLayout(frame_row)

        self.frame_hint = QLabel("Note: values are reinterpreted in the new frame, not converted.")
        self.frame_hint.setWordWrap(True)
        self.frame_hint.setStyleSheet("color: #f1c40f; font-size: 11px;")
        self.frame_hint.hide()
        body.addWidget(self.frame_hint)

        # Axes
        for group, title, prefix in (("linear", "Linear [N/m]", ""), ("angular", "Angular [Nm/rad]", "r")):
            group_row = QHBoxLayout()
            group_label = QLabel(f"<b>{title}</b>")
            group_row.addWidget(group_label)
            group_row.addStretch()
            range_label = QLabel("")
            range_label.setStyleSheet("font-family: monospace; color: #95a5a6; font-size: 10px;")
            group_row.addWidget(range_label)
            self.range_labels[group] = range_label
            body.addLayout(group_row)

            for i, axis in enumerate(AXES):
                row = QHBoxLayout()
                axis_label = QLabel(prefix + axis)
                axis_label.setFixedWidth(20)
                row.addWidget(axis_label)

                slider = QSlider(Qt.Orientation.Horizontal)
                slider.valueChanged.connect(lambda val, g=group, idx=i: self.on_value_edited(g, idx, val / SLIDER_SCALE))
                row.addWidget(slider)

                spin = QDoubleSpinBox()
                spin.setDecimals(1)
                spin.setFixedWidth(80)
                spin.setKeyboardTracking(False)
                spin.valueChanged.connect(lambda val, g=group, idx=i: self.on_value_edited(g, idx, val))
                row.addWidget(spin)

                dot = QLabel("●")
                dot.setFixedWidth(12)
                dot.setStyleSheet("color: #f1c40f;")
                dot.setToolTip("Differs from the value the controller uses")
                row.addWidget(dot)

                self.sliders[group].append(slider)
                self.spins[group].append(spin)
                self.dots[group].append(dot)
                body.addLayout(row)

        self.underneath_label = QLabel("⚠ Committed values changed underneath (other client). "
                                       "Reset to see them, or Apply to overwrite.")
        self.underneath_label.setWordWrap(True)
        self.underneath_label.setStyleSheet("color: #e67e22; font-size: 11px;")
        self.underneath_label.hide()
        body.addWidget(self.underneath_label)

        # Buttons
        btn_row = QHBoxLayout()
        self.btn_reset = QPushButton("Reset to current")
        self.btn_reset.setToolTip("Discard edits and re-read the values the controller uses")
        self.btn_reset.clicked.connect(self.on_reset_clicked)
        btn_row.addWidget(self.btn_reset)
        btn_row.addStretch()
        self.btn_apply = QPushButton("Apply now")
        self.btn_apply.setToolTip("Send frame and all six values together.\n"
                                  "Acts immediately, without a blend.")
        self.btn_apply.setStyleSheet("""
            QPushButton { background-color: #2980b9; color: white; font-weight: bold; padding: 4px 12px; }
            QPushButton:disabled { background-color: #3d3d3d; color: #7f8c8d; }
        """)
        self.btn_apply.clicked.connect(self.on_apply_clicked)
        btn_row.addWidget(self.btn_apply)
        body.addLayout(btn_row)

        self.result_label = QLabel("")
        self.result_label.setWordWrap(True)
        self.result_label.setStyleSheet("color: #e74c3c; font-weight: bold; font-size: 11px;")
        self.result_label.hide()
        body.addWidget(self.result_label)

        layout.addWidget(self.body)

    # --- Expand / collapse ---
    def set_expanded(self, expanded: bool):
        if self.btn_expand.isChecked() != expanded:
            self.btn_expand.setChecked(expanded)  # calls set_expanded again
            return
        self.btn_expand.setText(("▾" if expanded else "▸") + " Stiffness")
        self.body.setVisible(expanded)
        # Summary only when collapsed; expanded, the values are shown below
        self.summary_label.setVisible(not expanded)
        self.refresh_pending()

    # --- User input (changes staged only) ---
    def on_frame_selected(self, frame):
        if self.staged is not None:
            self.staged["frame"] = frame
            self.refresh_pending()

    def on_value_edited(self, group, idx, value):
        if self.staged is None:
            return
        self.staged[group][idx] = value
        # Keep slider and field in sync without triggering again
        self.set_axis_display(group, idx, value)
        self.refresh_pending()

    def on_reset_clicked(self):
        if self.committed is not None:
            self.load_staged_from_committed()
        self.result_label.hide()
        self.node.request_stiffness_read()

    def on_apply_clicked(self):
        if self.staged is None or not self.is_pending():
            return
        self.result_label.hide()
        self.apply_started = time.monotonic()
        self.refresh_enabled()
        self.node.apply_stiffness(self.staged["frame"], list(self.staged["linear"]), list(self.staged["angular"]))

    # --- Display ---
    def set_axis_display(self, group, idx, value):
        slider = self.sliders[group][idx]
        spin = self.spins[group][idx]
        slider.blockSignals(True)
        spin.blockSignals(True)
        slider.setValue(round(value * SLIDER_SCALE))
        spin.setValue(value)
        slider.blockSignals(False)
        spin.blockSignals(False)

    def load_staged_from_committed(self):
        c = self.committed
        self.staged = {"frame": c["frame"], "linear": list(c["linear"]), "angular": list(c["angular"])}
        self.changed_underneath = False
        self.show_staged()

    def show_staged(self):
        frame_btn = self.frame_buttons.get(self.staged["frame"])
        if frame_btn is not None:
            frame_btn.blockSignals(True)
            frame_btn.setChecked(True)
            frame_btn.blockSignals(False)
        for group in ("linear", "angular"):
            for idx, value in enumerate(self.staged[group]):
                self.set_axis_display(group, idx, value)
        self.refresh_pending()

    def apply_limits(self):
        """Slider ranges [0, limit]; staged values outside are clamped (and then pending)."""
        for group, limit in (("linear", self.committed["max_linear"]), ("angular", self.committed["max_angular"])):
            self.range_labels[group].setText(f"0 … {limit:g}")
            for idx in range(3):
                # setRange clamps the value and would otherwise trigger valueChanged -> staged
                for widget in (self.sliders[group][idx], self.spins[group][idx]):
                    widget.blockSignals(True)
                self.sliders[group][idx].setRange(0, round(limit * SLIDER_SCALE))
                self.spins[group][idx].setRange(0.0, limit)
                for widget in (self.sliders[group][idx], self.spins[group][idx]):
                    widget.blockSignals(False)
                if self.staged is not None:
                    self.staged[group][idx] = min(max(self.staged[group][idx], 0.0), limit)

    def is_pending(self):
        if self.staged is None or self.committed is None:
            return False
        return (self.staged["frame"] != self.committed["frame"]
                or self.staged["linear"] != list(self.committed["linear"])
                or self.staged["angular"] != list(self.committed["angular"]))

    def refresh_pending(self):
        pending = self.is_pending()
        self.pending_label.setText("● pending changes" if not self.body.isHidden() else "● pending")
        self.pending_label.setVisible(pending)
        if self.staged is not None and self.committed is not None:
            for group in ("linear", "angular"):
                for idx, dot in enumerate(self.dots[group]):
                    dot.setVisible(self.staged[group][idx] != self.committed[group][idx])
            self.frame_hint.setVisible(self.staged["frame"] != self.committed["frame"])
        if not pending:
            self.changed_underneath = False
        self.underneath_label.setVisible(self.changed_underneath)
        self.refresh_enabled()

    def refresh_enabled(self):
        ready = self.committed is not None and self.apply_started is None
        for group in ("linear", "angular"):
            for widget in self.sliders[group] + self.spins[group]:
                widget.setEnabled(ready)
        for btn in self.frame_buttons.values():
            btn.setEnabled(ready)
        self.btn_reset.setEnabled(ready)
        self.btn_apply.setEnabled(ready and self.is_pending())

    def refresh_summary(self):
        if self.committed is None:
            self.summary_label.setText("not read yet")
            return
        c = self.committed
        lin = "/".join(f"{v:g}" for v in c["linear"])
        ang = "/".join(f"{v:g}" for v in c["angular"])
        self.summary_label.setText(f"[{c['frame']}] {lin} N/m · {ang} Nm/rad")

    def refresh_info(self):
        status = self.store.get_status()
        available = self.store.get_available_modes()
        no_impedance = available is not None and not IMPEDANCE_MODES & available
        if no_impedance and not status["mode_switching"]:
            text = "No impedance mode available on the firmware."
        elif self.committed is None:
            text = f"Waiting for '{IMPEDANCE_CONTROLLER}' parameters..."
        elif status["mode"] == RobotarmMode.GRAVITY_COMP:
            text = "Settings apply to the impedance controller only, no effect in hand guiding."
        elif status["mode"] not in IMPEDANCE_MODES:
            text = "Controller inactive: changes take effect on its next activation."
        else:
            text = ""
        if text != self.info_label.text():
            self.info_label.setText(text)
            self.info_label.setVisible(bool(text))

    def update_widget(self):
        """Called periodically by the main GUI timer (~30 Hz)."""
        result = self.store.pop_stiffness_apply_result()
        if result is not None:
            successful, reason = result
            self.apply_started = None
            if not successful:
                # Staged stays for correcting, committed is unchanged
                self.result_label.setText(f"✖ Rejected: {reason}")
                self.result_label.show()
            self.refresh_enabled()
        elif self.apply_started is not None and time.monotonic() - self.apply_started > APPLY_TIMEOUT_S:
            self.apply_started = None
            self.result_label.setText("✖ No response from the controller.")
            self.result_label.show()
            self.refresh_enabled()

        version, committed = self.store.get_stiffness()
        if version != self.seen_version:
            self.seen_version = version
            if committed is None:
                self.committed = None
                self.staged = None
            else:
                had_pending = self.is_pending()
                self.committed = committed
                self.apply_limits()
                if self.staged is None or not had_pending:
                    # Without own changes, staged follows committed
                    self.load_staged_from_committed()
                else:
                    self.changed_underneath = self.is_pending()
                    self.show_staged()
            self.refresh_summary()
            self.refresh_pending()

        self.refresh_info()
