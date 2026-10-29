"""
PROGRAM workspace: goals, program editor and run bar, a client of the motion_server.

The workspace never activates a controller or sends a trajectory itself. Jobs go to the
motion_server (robotarm_motion, action /motion_server/run), which plans them with MoveIt and runs
them inside its own control_center mission. Goals and programs are read and changed only through
the motion_server's services (it owns the goals file).

Teaching works with every manual controller of the left column: move the arm there (jog, hand
guiding) and press Teach, or Space for "Teach & append" (see shortcut_teach).

The program table is a draft: PLAN and RUN send its steps (RunMotion.steps), so it can be tried
before it is saved. Save stores it as a program of the goals file.
"""

import math
import queue

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (QAbstractItemView, QAbstractSpinBox, QApplication, QComboBox,
                             QDialog, QDialogButtonBox, QDoubleSpinBox, QFormLayout, QFrame,
                             QHBoxLayout, QHeaderView, QInputDialog, QLabel, QLineEdit,
                             QListWidget, QListWidgetItem, QMessageBox, QPushButton, QSlider,
                             QSpinBox, QTableWidget, QTableWidgetItem, QTextEdit, QVBoxLayout,
                             QWidget)

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient

from robotarm_interface.action import RunMotion
from robotarm_interface.msg import MotionGoal, MotionStep, RobotarmMode
from robotarm_interface.srv import (GetGoals, RemoveGoal, RemoveProgram, SetGoal, SetProgram,
                                    TeachGoal)

MOTION_SERVER = "motion_server"
MOTIONS = ["PTP", "LIN", "OMPL"]
OFFSET_FRAMES = ["base", "tcp"]
JOINT_COUNT = 6

# Table columns
COL_NO, COL_GOAL, COL_MOTION, COL_BLEND, COL_SPEED, COL_OFFSET, COL_FRAME = range(7)
HEADERS = ["#", "Goal", "Motion", "Blend [mm]", "Speed [%]", "Offset x, y, z [mm]", "Offset in"]

RUNNING_ROW = QColor("#7d6608")
FAILED_ROW = QColor("#78281f")

BUTTON_STYLE = """
    QPushButton {{ background-color: {color}; color: white; font-weight: bold; }}
    QPushButton:disabled {{ background-color: #3d3d3d; color: #7f8c8d; }}
"""

MOTION_TOOLTIP = ("PTP: joint space, all axes synchronous\n"
                  "LIN: straight tcp line\n"
                  "OMPL: collision-free path, the robot stops before and after it")


def quat_from_rpy(roll, pitch, yaw):
    """Quaternion (x, y, z, w) of R = Rz(yaw) Ry(pitch) Rx(roll), angles in rad."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def rpy_from_quat(x, y, z, w):
    """Inverse of quat_from_rpy: (roll, pitch, yaw) in rad."""
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def degree_box(value, low=-360.0, high=360.0):
    box = QDoubleSpinBox()
    box.setRange(low, high)
    box.setDecimals(2)
    box.setValue(value)
    return box


class GoalDialog(QDialog):
    """Edit the values of a goal: joints [deg], or tcp xyz [mm], rpy [deg] and the IK seed [deg]."""

    def __init__(self, goal: MotionGoal, parent=None):
        super().__init__(parent)
        self.goal = goal
        self.setWindowTitle(f"Edit goal '{goal.name}'")
        form = QFormLayout(self)
        if goal.type == MotionGoal.TYPE_JOINT:
            self.joint_boxes = [degree_box(math.degrees(v)) for v in goal.joints]
            for i, box in enumerate(self.joint_boxes):
                form.addRow(f"Axis {i + 1} [deg]", box)
        else:
            p, o = goal.pose.position, goal.pose.orientation
            self.xyz_boxes = [degree_box(v * 1000.0, -5000.0, 5000.0) for v in (p.x, p.y, p.z)]
            for axis, box in zip("xyz", self.xyz_boxes):
                form.addRow(f"{axis} [mm]", box)
            self.rpy_boxes = [degree_box(math.degrees(a)) for a in rpy_from_quat(o.x, o.y, o.z, o.w)]
            for axis, box in zip(("roll", "pitch", "yaw"), self.rpy_boxes):
                form.addRow(f"{axis} [deg]", box)
            self.seed_edit = QLineEdit(", ".join(f"{math.degrees(v):.2f}" for v in goal.seed))
            self.seed_edit.setPlaceholderText("none")
            self.seed_edit.setToolTip("IK seed: the arm configuration the IK should keep, "
                                      "6 values in deg, empty = none")
            form.addRow("Seed [deg]", self.seed_edit)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok |
                                   QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def edited_goal(self) -> MotionGoal:
        """The goal with the edited values (ValueError for an invalid seed)."""
        goal = MotionGoal(name=self.goal.name, type=self.goal.type, source=MotionGoal.SOURCE_FILE)
        if goal.type == MotionGoal.TYPE_JOINT:
            goal.joints = [math.radians(box.value()) for box in self.joint_boxes]
            return goal
        p, o = goal.pose.position, goal.pose.orientation
        p.x, p.y, p.z = (box.value() / 1000.0 for box in self.xyz_boxes)
        o.x, o.y, o.z, o.w = quat_from_rpy(*(math.radians(b.value()) for b in self.rpy_boxes))
        text = self.seed_edit.text().strip()
        if text:
            values = [float(v) for v in text.replace(";", ",").split(",")]
            if len(values) != JOINT_COUNT:
                raise ValueError(f"the seed needs {JOINT_COUNT} values")
            goal.seed = [math.radians(v) for v in values]
        return goal


class ProgramWidget(QWidget):
    def __init__(self, store, node, parent=None):
        super().__init__(parent)
        self.store = store
        self.node = node
        self.name = "ProgramWidget"

        self.action_client = ActionClient(self.node, RunMotion, f"/{MOTION_SERVER}/run")
        self.clients = {}
        for srv_type, name in ((GetGoals, "get_goals"), (TeachGoal, "teach_goal"),
                               (SetGoal, "set_goal"), (RemoveGoal, "remove_goal"),
                               (SetProgram, "set_program"), (RemoveProgram, "remove_program")):
            self.clients[name] = (srv_type, self.node.create_client(srv_type, f"/{MOTION_SERVER}/{name}"))

        # ROS callbacks (ROS thread) put (function, args) here, update_widget runs them (Qt thread)
        self.events = queue.SimpleQueue()

        # Goals and programs of the motion_server (last get_goals response)
        self.goals = {}                 # name -> MotionGoal
        self.programs = {}              # name -> [MotionStep]
        self.goals_loaded = False
        self.goals_pending = False

        # Draft
        self.program_name = ""          # name of the program in the table, "" = not named yet
        self.dirty = False
        self.loading_table = False      # while filling the table: no dirty marks

        # Jobs
        self.goal_handle = None
        self.job_running = False
        self.job_plan_only = False
        self.job_phase = ""
        self.cancel_requested = False
        self.job_highlights = False     # the job runs the table: highlight its steps
        self.highlighted = (0, 0, None)  # (from, to, colour) of highlighted steps
        self.teach_pending = False

        self.server_ready = False       # motion_server reachable: plan, teach, edit
        self.available = False          # ... and the firmware offers the TRAJECTORY mode: run
        self.trajectory_available = False
        self.unavailable_reason = ""
        self.last_update_error = None

        self.init_ui()

    # ----- UI ------------------------------------------------------------------------------------

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        top = QHBoxLayout()
        top.addWidget(self.init_goals_panel())
        top.addWidget(self.init_program_panel(), stretch=1)
        layout.addLayout(top, stretch=1)
        layout.addWidget(self.init_run_bar())
        self.result_label = QLabel("")
        self.result_label.setWordWrap(True)
        self.result_label.hide()
        layout.addWidget(self.result_label)
        self.update_ui_lock_state()

    def init_goals_panel(self):
        panel = QFrame()
        panel.setFrameShape(QFrame.Shape.StyledPanel)
        panel.setFixedWidth(300)
        box = QVBoxLayout(panel)
        box.setContentsMargins(8, 6, 8, 6)

        header = QHBoxLayout()
        header.addWidget(QLabel("<b>Goals</b>"))
        header.addStretch()
        self.btn_reload = QPushButton("⟳")
        self.btn_reload.setFixedWidth(32)
        self.btn_reload.setToolTip("Reload goals and programs from the motion_server")
        self.btn_reload.clicked.connect(self.reload_goals)
        header.addWidget(self.btn_reload)
        box.addLayout(header)

        self.goal_list = QListWidget()
        self.goal_list.itemDoubleClicked.connect(lambda _: self.edit_goal())
        self.goal_list.currentItemChanged.connect(lambda *_: self.update_ui_lock_state())
        box.addWidget(self.goal_list, stretch=1)

        self.teach_name = QLineEdit()
        self.teach_name.setPlaceholderText("name (empty = automatic)")
        box.addWidget(self.teach_name)
        row = QHBoxLayout()
        self.btn_teach_joint = QPushButton("Teach joint")
        self.btn_teach_joint.setToolTip("Save the current joint values (unambiguous)")
        self.btn_teach_joint.clicked.connect(lambda: self.teach(cartesian=False))
        self.btn_teach_cart = QPushButton("Teach Cartesian")
        self.btn_teach_cart.setToolTip("Save the tcp pose, with the current joints as IK seed")
        self.btn_teach_cart.clicked.connect(lambda: self.teach(cartesian=True))
        row.addWidget(self.btn_teach_joint)
        row.addWidget(self.btn_teach_cart)
        box.addLayout(row)

        row = QHBoxLayout()
        self.goal_motion = QComboBox()
        self.goal_motion.addItems(MOTIONS)
        self.goal_motion.setToolTip(MOTION_TOOLTIP)
        self.btn_move_goal = QPushButton("Move to")
        self.btn_move_goal.setToolTip("Move to the selected goal (robot must be ARMED)")
        self.btn_move_goal.clicked.connect(self.move_to_goal)
        self.btn_edit_goal = QPushButton("Edit")
        self.btn_edit_goal.setToolTip("Edit the values of the selected goal (also: double-click)")
        self.btn_edit_goal.clicked.connect(self.edit_goal)
        self.btn_delete_goal = QPushButton("Delete")
        self.btn_delete_goal.clicked.connect(self.delete_goal)
        row.addWidget(self.goal_motion)
        row.addWidget(self.btn_move_goal)
        row.addWidget(self.btn_edit_goal)
        row.addWidget(self.btn_delete_goal)
        box.addLayout(row)

        self.goals_file_label = QLabel("")
        self.goals_file_label.setWordWrap(True)
        self.goals_file_label.setStyleSheet("color: #7f8c8d; font-size: 10px;")
        box.addWidget(self.goals_file_label)
        return panel

    def init_program_panel(self):
        panel = QFrame()
        panel.setFrameShape(QFrame.Shape.StyledPanel)
        box = QVBoxLayout(panel)
        box.setContentsMargins(8, 6, 8, 6)

        row = QHBoxLayout()
        row.addWidget(QLabel("<b>Program</b>"))
        self.program_combo = QComboBox()
        self.program_combo.setMinimumWidth(180)
        self.program_combo.activated.connect(self.program_selected)
        row.addWidget(self.program_combo)
        self.btn_new = QPushButton("New")
        self.btn_new.clicked.connect(self.new_program)
        self.btn_save = QPushButton("Save")
        self.btn_save.clicked.connect(self.save_program)
        self.btn_delete_program = QPushButton("Delete")
        self.btn_delete_program.clicked.connect(self.delete_program)
        for btn in (self.btn_new, self.btn_save, self.btn_delete_program):
            row.addWidget(btn)
        row.addStretch()
        box.addLayout(row)

        self.table = QTableWidget(0, len(HEADERS))
        self.table.setHorizontalHeaderLabels(HEADERS)
        self.table.verticalHeader().hide()
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(COL_GOAL, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(COL_OFFSET, QHeaderView.ResizeMode.Stretch)
        self.table.itemSelectionChanged.connect(self.update_ui_lock_state)
        box.addWidget(self.table, stretch=1)

        row = QHBoxLayout()
        self.btn_add = QPushButton("+ Selected goal")
        self.btn_add.setToolTip("Append the goal selected on the left as a step")
        self.btn_add.clicked.connect(self.add_selected_goal)
        self.btn_up = QPushButton("▲")
        self.btn_up.clicked.connect(lambda: self.move_row(-1))
        self.btn_down = QPushButton("▼")
        self.btn_down.clicked.connect(lambda: self.move_row(1))
        self.btn_remove_row = QPushButton("✕")
        self.btn_remove_row.setToolTip("Remove the selected step (the goal stays)")
        self.btn_remove_row.clicked.connect(self.remove_row)
        for btn in (self.btn_up, self.btn_down, self.btn_remove_row):
            btn.setFixedWidth(36)
        self.btn_teach_append = QPushButton("Teach && append  [Space]")
        self.btn_teach_append.setToolTip("Save the current pose as a new joint goal and append "
                                         "it as a step (motion and blend of the previous step)")
        self.btn_teach_append.setStyleSheet(BUTTON_STYLE.format(color="#8e44ad"))
        self.btn_teach_append.clicked.connect(self.teach_and_append)
        for widget in (self.btn_add, self.btn_up, self.btn_down, self.btn_remove_row):
            row.addWidget(widget)
        row.addStretch()
        row.addWidget(self.btn_teach_append)
        box.addLayout(row)

        row = QHBoxLayout()
        self.btn_retouch = QPushButton("Re-teach goal")
        self.btn_retouch.setToolTip("Overwrite the goal of the selected step with the current pose")
        self.btn_retouch.clicked.connect(self.reteach_selected)
        self.btn_move_step = QPushButton("Move to step")
        self.btn_move_step.setToolTip("Move to the selected step alone (with its motion and offset)")
        self.btn_move_step.clicked.connect(self.move_to_step)
        row.addWidget(self.btn_retouch)
        row.addWidget(self.btn_move_step)
        row.addStretch()
        box.addLayout(row)

        self.warning_box = QTextEdit()
        self.warning_box.setReadOnly(True)
        self.warning_box.setMaximumHeight(60)
        self.warning_box.setStyleSheet("color: #f39c12; background-color: #2b2416;")
        self.warning_box.hide()
        box.addWidget(self.warning_box)
        self.refresh_program_combo()
        return panel

    def init_run_bar(self):
        bar = QFrame()
        bar.setStyleSheet("QFrame#RunBar { background-color: #2c3e50; border-radius: 8px; "
                          "border: 1px solid #3498db; }")
        bar.setObjectName("RunBar")
        row = QHBoxLayout(bar)
        title = QLabel("Speed")
        title.setStyleSheet("font-weight: bold; color: #3498db;")
        row.addWidget(title)
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        # Scaling of joint_limits.yaml / pilz_cartesian_limits.yaml, the motion_server takes (0, 1]
        self.speed_slider.setRange(5, 100)
        self.speed_slider.setValue(20)
        self.speed_slider.setFixedWidth(180)
        self.speed_display = QLabel("20%")
        self.speed_display.setStyleSheet("font-weight: bold; color: #f1c40f;")
        self.speed_slider.valueChanged.connect(lambda val: self.speed_display.setText(f"{val}%"))
        row.addWidget(self.speed_slider)
        row.addWidget(self.speed_display)
        row.addSpacing(12)
        self.btn_plan = QPushButton("PLAN")
        self.btn_plan.setToolTip("Plan the program in the table and show it in RViz, nothing moves")
        self.btn_plan.setStyleSheet(BUTTON_STYLE.format(color="#2980b9"))
        self.btn_plan.clicked.connect(lambda: self.run_draft(plan_only=True))
        self.btn_run = QPushButton("RUN")
        self.btn_run.setToolTip("Plan everything, then execute (robot must be ARMED, "
                                "no manual mission active)")
        self.btn_run.setStyleSheet(BUTTON_STYLE.format(color="#2ecc71"))
        self.btn_run.clicked.connect(lambda: self.run_draft(plan_only=False))
        self.btn_cancel = QPushButton("CANCEL")
        self.btn_cancel.setToolTip("Cancel the running job, the robot stops and holds")
        self.btn_cancel.setStyleSheet(BUTTON_STYLE.format(color="#e74c3c"))
        self.btn_cancel.clicked.connect(self.cancel_job)
        for btn in (self.btn_plan, self.btn_run, self.btn_cancel):
            btn.setMinimumHeight(34)
            btn.setMinimumWidth(90)
            row.addWidget(btn)
        self.status_label = QLabel("")
        self.status_label.setWordWrap(True)
        row.addWidget(self.status_label, stretch=1)
        return bar

    # ----- service calls -------------------------------------------------------------------------

    def ros_ok(self):
        return rclpy.ok(context=self.node.context)

    def call(self, name, request, on_done):
        """Call a motion_server service; on_done(response or None) runs in the Qt thread."""
        _, client = self.clients[name]
        if not self.ros_ok() or not client.service_is_ready():
            self.show_result(False, "motion_server not running")
            return False
        future = client.call_async(request)
        future.add_done_callback(lambda f: self.events.put((on_done, (f.result(),))))
        return True

    # ----- goals ---------------------------------------------------------------------------------

    def reload_goals(self):
        """Ask the motion_server for its goals and programs."""
        if self.goals_pending:
            return
        self.goals_pending = self.call("get_goals", GetGoals.Request(), self.apply_goals)

    def apply_goals(self, res):
        self.goals_pending = False
        self.goals_loaded = True
        if res is None or not res.success:
            self.show_result(False, f"Goals file: {res.message if res else 'no response'}")
            return
        self.goals = {goal.name: goal for goal in res.goals}
        self.programs = {program.name: list(program.steps) for program in res.programs}
        selected = self.selected_goal_name()
        self.goal_list.clear()
        for goal in res.goals:
            kind = "joint" if goal.type == MotionGoal.TYPE_JOINT else "cartesian"
            source = ", SRDF" if goal.source == MotionGoal.SOURCE_SRDF else ""
            item = QListWidgetItem(f"● {goal.name}  ({kind}{source})")
            item.setData(Qt.ItemDataRole.UserRole, goal.name)
            self.goal_list.addItem(item)
            if goal.name == selected:
                self.goal_list.setCurrentItem(item)
        self.goals_file_label.setText(res.goals_file)
        self.refresh_goal_combos()
        self.refresh_program_combo()
        self.update_ui_lock_state()

    def selected_goal_name(self):
        item = self.goal_list.currentItem()
        return item.data(Qt.ItemDataRole.UserRole) if item else None

    def select_goal(self, name):
        for i in range(self.goal_list.count()):
            if self.goal_list.item(i).data(Qt.ItemDataRole.UserRole) == name:
                self.goal_list.setCurrentRow(i)
                return

    def teach(self, cartesian, name=None, prefix="", on_saved=None):
        """Let the motion_server save the current pose as a goal (replaces a goal of the file)."""
        name = self.teach_name.text().strip() if name is None else name
        if self.goals.get(name) is not None and self.goals[name].source == MotionGoal.SOURCE_SRDF:
            self.show_result(False, f"'{name}' is a named state of the SRDF, pick another name")
            return
        request = TeachGoal.Request(name=name, prefix=prefix, cartesian=cartesian, overwrite=True)

        def done(res):
            self.teach_pending = False
            if res is None or not res.success:
                self.show_result(False, f"Teach failed: {res.message if res else 'no response'}")
                return
            self.show_result(True, res.message)
            self.teach_name.clear()
            if on_saved is not None:
                on_saved(res.goal.name)
            self.reload_goals_then_select(res.goal.name)
        self.teach_pending = self.call("teach_goal", request, done)
        self.update_ui_lock_state()

    def reload_goals_then_select(self, name):
        def done(res):
            self.apply_goals(res)
            self.select_goal(name)
        self.goals_pending = self.call("get_goals", GetGoals.Request(), done)

    def edit_goal(self):
        name = self.selected_goal_name()
        goal = self.goals.get(name)
        if goal is None:
            return
        if goal.source == MotionGoal.SOURCE_SRDF:
            self.show_result(False, f"'{name}' is a named state of the SRDF; teach or edit a "
                                    "goal of another name")
            return
        dialog = GoalDialog(goal, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        try:
            edited = dialog.edited_goal()
        except ValueError as e:
            self.show_result(False, f"Goal not changed: {e}")
            return

        def done(res):
            ok = res is not None and res.success
            self.show_result(ok, res.message if res else "no response")
            if ok:
                self.reload_goals_then_select(name)
        self.call("set_goal", SetGoal.Request(goal=edited, overwrite=True), done)

    def delete_goal(self):
        name = self.selected_goal_name()
        if name is None:
            return
        answer = QMessageBox.question(self, "Delete goal", f"Delete goal '{name}'?")
        if answer != QMessageBox.StandardButton.Yes:
            return

        def done(res):
            ok = res is not None and res.success
            self.show_result(ok, res.message if res else "no response")
            if ok:
                self.reload_goals()
        self.call("remove_goal", RemoveGoal.Request(name=name), done)

    # ----- program table -------------------------------------------------------------------------

    def refresh_program_combo(self):
        self.program_combo.blockSignals(True)
        self.program_combo.clear()
        names = sorted(self.programs)
        if self.program_name and self.program_name not in self.programs:
            names.append(self.program_name)     # new, not saved yet
        for name in names:
            unsaved = name == self.program_name and self.dirty
            self.program_combo.addItem(name + (" *" if unsaved else ""), name)
        if not self.program_name:
            self.program_combo.addItem("(unnamed draft)" + (" *" if self.dirty else ""), "")
        index = self.program_combo.findData(self.program_name)
        self.program_combo.setCurrentIndex(max(index, 0))
        self.program_combo.blockSignals(False)

    def program_selected(self, index):
        name = self.program_combo.itemData(index)
        if name == self.program_name:
            return
        if not self.confirm_discard():
            self.refresh_program_combo()
            return
        self.load_program(name)

    def confirm_discard(self):
        """True if there are no unsaved changes or the user wants to discard them."""
        if not self.dirty:
            return True
        answer = QMessageBox.question(
            self, "Unsaved changes",
            f"Discard the unsaved changes of '{self.program_name or 'the draft'}'?")
        return answer == QMessageBox.StandardButton.Yes

    def load_program(self, name):
        self.program_name = name
        self.set_steps(self.programs.get(name, []))
        self.set_dirty(False)

    def new_program(self):
        if not self.confirm_discard():
            return
        name, ok = QInputDialog.getText(self, "New program", "Program name:")
        name = name.strip()
        if not ok or not name:
            return
        if name in self.programs:
            self.show_result(False, f"Program '{name}' exists, select it in the list")
            return
        self.program_name = name
        self.set_steps([])
        self.set_dirty(True)

    def set_dirty(self, dirty=True):
        if self.loading_table:
            return
        self.dirty = dirty
        self.refresh_program_combo()
        self.refresh_row_states()
        self.update_ui_lock_state()

    def set_steps(self, steps):
        self.loading_table = True
        try:
            self.table.setRowCount(0)
            for step in steps:
                self.append_row(step)
        finally:
            self.loading_table = False
        self.clear_highlight()
        self.refresh_row_states()
        self.warning_box.hide()

    def append_row(self, step: MotionStep):
        row = self.table.rowCount()
        self.table.insertRow(row)
        number = QTableWidgetItem(str(row + 1))
        number.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable)
        number.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table.setItem(row, COL_NO, number)

        goal = QComboBox()
        self.fill_goal_combo(goal, step.goal)
        goal.currentTextChanged.connect(lambda _: self.set_dirty())
        self.table.setCellWidget(row, COL_GOAL, goal)

        motion = QComboBox()
        motion.addItems(MOTIONS)
        motion.setToolTip(MOTION_TOOLTIP)
        motion.setCurrentText(step.motion.upper() if step.motion.upper() in MOTIONS else "PTP")
        motion.currentTextChanged.connect(lambda _: self.set_dirty())
        self.table.setCellWidget(row, COL_MOTION, motion)

        blend = QDoubleSpinBox()
        blend.setRange(0.0, 500.0)
        blend.setDecimals(1)
        blend.setValue(step.blend * 1000.0)
        blend.setToolTip("Blend radius: the tcp passes the goal this close without stopping")
        blend.valueChanged.connect(lambda _: self.set_dirty())
        self.table.setCellWidget(row, COL_BLEND, blend)

        speed = QSpinBox()
        speed.setRange(0, 100)
        speed.setSpecialValueText("job")
        speed.setValue(int(round(step.scaling * 100)))
        speed.setToolTip("Speed of this step, 'job' = the speed of the run bar")
        speed.valueChanged.connect(lambda _: self.set_dirty())
        self.table.setCellWidget(row, COL_SPEED, speed)

        offset = QLineEdit(", ".join(f"{v * 1000:g}" for v in step.offset_xyz))
        offset.setPlaceholderText("none")
        offset.setToolTip("Added to the goal's tcp position, e.g. 0, 0, 50 = 50 mm up (base) "
                          "or along the tool's z axis (tcp)")
        offset.textChanged.connect(lambda _: self.set_dirty())
        self.table.setCellWidget(row, COL_OFFSET, offset)

        frame = QComboBox()
        frame.addItems(OFFSET_FRAMES)
        frame.setCurrentIndex(1 if step.offset_frame == MotionStep.OFFSET_TCP else 0)
        frame.setToolTip("base: offset in the base frame; tcp: in the tool frame of the goal")
        frame.currentTextChanged.connect(lambda _: self.set_dirty())
        self.table.setCellWidget(row, COL_FRAME, frame)

    def fill_goal_combo(self, combo, current):
        combo.blockSignals(True)
        combo.clear()
        names = sorted(self.goals)
        if current and current not in self.goals:
            names.append(current)       # unknown goal: kept, the server reports it
        combo.addItems(names)
        combo.setCurrentText(current)
        combo.blockSignals(False)

    def refresh_goal_combos(self):
        for row in range(self.table.rowCount()):
            combo = self.table.cellWidget(row, COL_GOAL)
            self.fill_goal_combo(combo, combo.currentText())

    def refresh_row_states(self):
        """Numbers, and blend only where it can apply (not on OMPL or the last step)."""
        rows = self.table.rowCount()
        for row in range(rows):
            self.table.item(row, COL_NO).setText(str(row + 1))
            motion = self.table.cellWidget(row, COL_MOTION).currentText()
            next_motion = self.table.cellWidget(row + 1, COL_MOTION).currentText() \
                if row + 1 < rows else None
            blendable = motion != "OMPL" and next_motion not in (None, "OMPL")
            self.table.cellWidget(row, COL_BLEND).setEnabled(blendable)

    def steps(self):
        """MotionStep list of the table (ValueError with the row for invalid input)."""
        result = []
        for row in range(self.table.rowCount()):
            text = self.table.cellWidget(row, COL_OFFSET).text().strip()
            offset = []
            if text:
                try:
                    offset = [float(v) / 1000.0 for v in text.replace(";", ",").split(",")]
                except ValueError:
                    offset = None
                if offset is None or len(offset) != 3:
                    raise ValueError(f"step {row + 1}: offset needs 3 numbers in mm, e.g. 0, 0, 50")
            blend_box = self.table.cellWidget(row, COL_BLEND)
            frame = self.table.cellWidget(row, COL_FRAME).currentText()
            result.append(MotionStep(
                goal=self.table.cellWidget(row, COL_GOAL).currentText(),
                motion=self.table.cellWidget(row, COL_MOTION).currentText(),
                blend=blend_box.value() / 1000.0 if blend_box.isEnabled() else 0.0,
                scaling=self.table.cellWidget(row, COL_SPEED).value() / 100.0,
                offset_xyz=offset,
                offset_frame=MotionStep.OFFSET_TCP if frame == "tcp" else MotionStep.OFFSET_BASE))
        return result

    def selected_row(self):
        rows = self.table.selectionModel().selectedRows() if self.table.selectionModel() else []
        return rows[0].row() if rows else -1

    def append_step(self, goal_name):
        """Append a step to goal_name with the motion and blend of the previous step."""
        previous = self.steps_or_none()
        last = previous[-1] if previous else None
        self.append_row(MotionStep(goal=goal_name, motion=last.motion if last else "PTP",
                                   blend=last.blend if last else 0.0))
        self.table.selectRow(self.table.rowCount() - 1)
        self.set_dirty()

    def steps_or_none(self):
        try:
            return self.steps()
        except ValueError:
            return None

    def add_selected_goal(self):
        name = self.selected_goal_name()
        if name:
            self.append_step(name)

    def move_row(self, delta):
        row = self.selected_row()
        target = row + delta
        if row < 0 or not 0 <= target < self.table.rowCount():
            return
        steps = self.steps_or_none()
        if steps is None:
            self.show_result(False, "Fix the invalid offset first")
            return
        steps[row], steps[target] = steps[target], steps[row]
        self.set_steps(steps)
        self.table.selectRow(target)
        self.set_dirty()

    def remove_row(self):
        row = self.selected_row()
        if row < 0:
            return
        self.table.removeRow(row)
        if self.table.rowCount():
            self.table.selectRow(min(row, self.table.rowCount() - 1))
        self.set_dirty()

    def teach_and_append(self):
        prefix = f"{self.program_name}_p" if self.program_name else "p"
        self.teach(cartesian=False, name="", prefix=prefix, on_saved=self.append_step)

    def reteach_selected(self):
        row = self.selected_row()
        if row < 0:
            return
        name = self.table.cellWidget(row, COL_GOAL).currentText()
        answer = QMessageBox.question(self, "Re-teach goal",
                                      f"Overwrite goal '{name}' with the current pose?")
        if answer == QMessageBox.StandardButton.Yes:
            cartesian = self.goals.get(name) is not None and \
                self.goals[name].type == MotionGoal.TYPE_CARTESIAN
            self.teach(cartesian=cartesian, name=name)

    def save_program(self):
        if not self.program_name:
            name, ok = QInputDialog.getText(self, "Save program", "Program name:")
            if not ok or not name.strip():
                return
            self.program_name = name.strip()
        try:
            steps = self.steps()
        except ValueError as e:
            self.show_result(False, str(e))
            return
        request = SetProgram.Request(name=self.program_name, steps=steps,
                                     overwrite=self.program_name in self.programs)

        def done(res):
            if res is None or not res.success:
                self.show_result(False, f"Not saved: {res.message if res else 'no response'}")
                if res is not None:
                    self.highlight(res.bad_step, res.bad_step, FAILED_ROW)
                return
            self.show_result(True, res.message)
            self.show_warnings(list(res.warnings))
            if res.bad_step:
                self.highlight(res.bad_step, res.bad_step, FAILED_ROW)
            self.set_dirty(False)
            self.reload_goals()
        self.call("set_program", request, done)

    def delete_program(self):
        name = self.program_name
        if not name or name not in self.programs:
            # not saved yet: just discard the draft
            if self.confirm_discard():
                self.program_name = ""
                self.set_steps([])
                self.set_dirty(False)
            return
        box = QMessageBox(QMessageBox.Icon.Question, "Delete program",
                          f"Delete program '{name}'?\nAlso delete its automatically named goals "
                          f"({name}_p1, ...) that no other program uses?", parent=self)
        with_goals = box.addButton("Program and its goals", QMessageBox.ButtonRole.YesRole)
        program_only = box.addButton("Program only", QMessageBox.ButtonRole.NoRole)
        box.addButton(QMessageBox.StandardButton.Cancel)
        box.exec()
        if box.clickedButton() not in (with_goals, program_only):
            return
        request = RemoveProgram.Request(name=name,
                                        remove_unused_goals=box.clickedButton() is with_goals)

        def done(res):
            ok = res is not None and res.success
            self.show_result(ok, res.message if res else "no response")
            if ok:
                self.program_name = ""
                self.set_steps([])
                self.set_dirty(False)
                self.reload_goals()
        self.call("remove_program", request, done)

    def show_warnings(self, warnings):
        if warnings:
            self.warning_box.setPlainText("\n".join(f"⚠ {w}" for w in warnings))
            self.warning_box.show()
        else:
            self.warning_box.hide()

    def highlight(self, step_from, step_to, colour):
        self.clear_highlight()
        self.highlighted = (step_from, step_to, colour)
        for row in range(step_from - 1, min(step_to, self.table.rowCount())):
            if row >= 0:
                self.table.item(row, COL_NO).setBackground(colour)

    def clear_highlight(self):
        for row in range(self.table.rowCount()):
            self.table.item(row, COL_NO).setBackground(QColor(0, 0, 0, 0))
        self.highlighted = (0, 0, None)

    # ----- jobs ----------------------------------------------------------------------------------

    def run_draft(self, plan_only):
        try:
            steps = self.steps()
        except ValueError as e:
            self.show_result(False, str(e))
            return
        if not steps:
            return
        label = f"program '{self.program_name}'" if self.program_name else "draft"
        self.send_job(RunMotion.Goal(steps=steps), plan_only, label, highlight_steps=True)

    def move_to_goal(self):
        name = self.selected_goal_name()
        if name:
            motion = self.goal_motion.currentText()
            self.send_job(RunMotion.Goal(goal=name, motion=motion), False,
                          f"{motion} to '{name}'")

    def move_to_step(self):
        row = self.selected_row()
        try:
            steps = self.steps()
        except ValueError as e:
            self.show_result(False, str(e))
            return
        if 0 <= row < len(steps):
            step = steps[row]
            step.blend = 0.0
            self.send_job(RunMotion.Goal(steps=[step]), False, f"step {row + 1}")

    def send_job(self, goal, plan_only, label, highlight_steps=False):
        if self.job_running:
            return
        goal.scaling = self.speed_slider.value() / 100.0
        goal.plan_only = plan_only
        self.job_running = True
        self.job_plan_only = plan_only
        self.job_phase = "SENDING"
        self.job_highlights = highlight_steps
        self.cancel_requested = False
        self.result_label.hide()
        self.clear_highlight()
        self.node.get_logger().info(
            f"{self.name}: {'plan' if plan_only else 'run'} {label} at {goal.scaling:.0%}")
        future = self.action_client.send_goal_async(
            goal, feedback_callback=lambda msg: self.events.put((self.on_feedback, (msg.feedback,))))
        future.add_done_callback(lambda f: self.events.put((self.on_goal_response, (f.result(),))))
        self.update_ui_lock_state()

    def on_feedback(self, fb):
        self.job_phase = fb.phase + (f" {fb.segment}/{fb.segments}" if fb.segment else "")
        if self.job_highlights and fb.step_from:
            self.job_phase += (f" (step {fb.step_from})" if fb.step_from == fb.step_to
                               else f" (steps {fb.step_from}-{fb.step_to})")
            self.highlight(fb.step_from, fb.step_to, RUNNING_ROW)
        elif fb.phase == "STOPPING_MISSION" and self.job_highlights:
            self.clear_highlight()

    def on_goal_response(self, handle):
        if handle is None or not handle.accepted:
            self.job_running = False
            self.show_result(False, "Rejected by the motion_server (another job running?)")
            return
        self.goal_handle = handle
        if self.cancel_requested:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(
            lambda f: self.events.put((self.on_result, (f.result(),))))

    def on_result(self, response):
        self.goal_handle = None
        self.job_running = False
        result = response.result
        ok = response.status == GoalStatus.STATUS_SUCCEEDED and result.success
        self.clear_highlight()
        if not ok and self.job_highlights and result.failed_step_from:
            self.highlight(result.failed_step_from, result.failed_step_to, FAILED_ROW)
        self.show_result(ok, result.message or "no message")

    def cancel_job(self):
        self.cancel_requested = True
        handle = self.goal_handle
        if handle is not None:
            handle.cancel_goal_async()
        self.node.get_logger().warn(f"{self.name}: job cancelled by the operator")

    # ----- keyboard ------------------------------------------------------------------------------

    def shortcut_teach(self):
        """Space: Teach & append (or teach a goal without a program); not while typing."""
        if not self.isVisible():
            return
        focus = QApplication.focusWidget()
        if isinstance(focus, (QLineEdit, QAbstractSpinBox, QTextEdit)):
            return
        if not (self.server_ready and not self.job_running and not self.teach_pending):
            return
        if self.program_name or self.table.rowCount():
            self.teach_and_append()
        else:
            self.teach(cartesian=False, name="")

    # ----- state ---------------------------------------------------------------------------------

    def show_result(self, ok, text):
        color = "#2ecc71" if ok else "#e74c3c"
        background = "#1a2c1f" if ok else "#2c1a1a"
        self.result_label.setStyleSheet(f"""
            background-color: {background}; color: {color}; font-weight: bold;
            border: 1px solid {color}; border-radius: 6px; padding: 6px;
        """)
        self.result_label.setText(("✔ " if ok else "⚠ ") + text)
        self.result_label.show()

    def run_blocker(self, status):
        """Why RUN is not possible right now, "" if it is."""
        if not self.available:
            return self.unavailable_reason
        if status["state"] == "STOP":
            return "Robot stopped: resume first (PLAN works)"
        if not status["armed"]:
            return "Robot not armed: PLAN only"
        owner = status["active_node"]
        if owner and owner != MOTION_SERVER:
            return f"Manual mission of {owner} active: stop it to run (PLAN works)"
        return ""

    def update_ui_lock_state(self):
        status = self.store.get_status()
        running = self.job_running
        idle = self.server_ready and not running
        blocker = self.run_blocker(status)
        has_goal = self.selected_goal_name() is not None
        has_row = self.selected_row() >= 0
        has_steps = self.table.rowCount() > 0
        teach = idle and not self.teach_pending

        for widget in (self.goal_list, self.btn_reload, self.program_combo, self.btn_new,
                       self.table, self.btn_add, self.btn_up, self.btn_down, self.btn_remove_row,
                       self.speed_slider, self.goal_motion):
            widget.setEnabled(not running)
        self.btn_save.setEnabled(idle and has_steps)
        self.btn_delete_program.setEnabled(idle and bool(self.program_name or has_steps))
        self.btn_edit_goal.setEnabled(idle and has_goal)
        self.btn_delete_goal.setEnabled(idle and has_goal)
        self.btn_teach_joint.setEnabled(teach)
        self.btn_teach_cart.setEnabled(teach)
        self.btn_teach_append.setEnabled(teach)
        self.btn_retouch.setEnabled(teach and has_row)
        self.btn_add.setEnabled(not running and has_goal)
        self.btn_move_goal.setEnabled(idle and has_goal and not blocker)
        self.btn_move_step.setEnabled(idle and has_row and not blocker)
        self.btn_plan.setEnabled(idle and has_steps)
        self.btn_run.setEnabled(idle and has_steps and not blocker)
        self.btn_cancel.setEnabled(running)

        if running:
            verb = "Planning" if self.job_plan_only else "Running"
            self.set_status(f"{verb}: {self.job_phase}", "#f1c40f", bold=True)
        elif blocker:
            self.set_status(blocker, "#95a5a6")
        else:
            self.set_status("Ready", "#2ecc71")

    def set_status(self, text, color, bold=False):
        if self.status_label.text() != text:
            self.status_label.setText(text)
            self.status_label.setStyleSheet(f"color: {color};" + (" font-weight: bold;" if bold else ""))

    def update_widget(self):
        """
        Called periodically by the main window timer (~30 Hz).

        Never raises: the main window updates its navigation lock, header and log after this call
        in the same timer slot, an exception here would skip them.
        """
        try:
            self._update()
        except Exception as e:   # noqa: B902 (keep the main window timer running)
            text = f"{type(e).__name__}: {e}"
            if text != self.last_update_error:
                self.last_update_error = text
                print(f"{self.name}: update failed: {text}", flush=True)

    def _update(self):
        while True:
            try:
                function, args = self.events.get_nowait()
            except queue.Empty:
                break
            function(*args)

        # after Ctrl-C rclpy is shut down while the window still runs: no more ROS calls
        server = self.ros_ok() and self.action_client.server_is_ready()
        # Keep the last answer while the firmware switches (available is empty then)
        if not self.store.get_status()["mode_switching"]:
            modes = self.store.get_available_modes()
            self.trajectory_available = modes is not None and RobotarmMode.TRAJECTORY in modes
        controller = self.trajectory_available
        self.server_ready = server
        self.available = server and controller
        if not server:
            self.unavailable_reason = ("motion_server not running "
                                       "(robot_bringup launch_robot.py, moveit:=true)")
        elif not controller:
            self.unavailable_reason = ("TRAJECTORY mode not available on the firmware "
                                       "(controller not loaded or hardware not usable)")
        if server and not self.goals_loaded:
            self.reload_goals()
        self.update_ui_lock_state()
