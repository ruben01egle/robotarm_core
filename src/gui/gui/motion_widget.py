"""
MoveIt page: plan and execute goals and programs through the motion_server.

The page itself never activates a controller or sends a trajectory. It sends jobs to the
motion_server (robotarm_motion, action /motion_server/run), which plans them with MoveIt, runs them
inside its own control_center mission and reports phases and result back. While a job executes,
active_nodes of /system_status is "motion_server", so the page navigation of the main window is
locked like during any other mission.

Goals and programs come from the goals file of the motion_server (its parameter goals_file);
"Teach" writes the current pose into the same file. See robotarm_motion/README.md.
"""

import math
import re

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (QComboBox, QFrame, QHBoxLayout, QLabel, QLineEdit, QPushButton,
                             QSlider, QVBoxLayout, QWidget)

from action_msgs.msg import GoalStatus
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

from robotarm_motion.goals import (CartesianGoal, default_goals_file, FRAME, GoalFile,
                                   GoalFileError, JointGoal, JOINTS, quat_from_rpy)
from robotarm_motion.moveit_client import load_named_states
from robotarm_interface.action import RunMotion

from .controller_modes import POSITION

MOTION_SERVER = 'motion_server'
TRAJECTORY_CONTROLLER = 'joint_trajectory_controller'
PROGRAM, GOAL = 'program', 'goal'
MOTIONS = ['PTP', 'LIN', 'OMPL']
GOAL_NAME = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')

BUTTON_STYLE = """
    QPushButton {{ background-color: {color}; color: white; font-weight: bold; }}
    QPushButton:disabled {{ background-color: #3d3d3d; color: #7f8c8d; }}
"""


class MotionWidget(QWidget):
    def __init__(self, store, node, parent=None):
        super().__init__(parent)
        self.store = store
        self.node = node
        self.name = "MotionWidget"

        self.action_client = ActionClient(self.node, RunMotion, f'/{MOTION_SERVER}/run')
        self.param_client = self.node.create_client(
            GetParameters, f'/{MOTION_SERVER}/get_parameters')
        # Same file as the motion_server: its parameter, the default until it has been read
        self.goals_file = default_goals_file()
        self.goals_file_read = False
        self.goals_file_pending = False

        # Joint names and positions for "Teach joint goal" (the data store has no joint names)
        self.joint_state = None
        self.node.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        # Job state: written by ROS callbacks (ROS thread), read by update_widget (Qt thread)
        self.goal_handle = None
        self.job_running = False
        self.job_plan_only = False
        self.job_phase = ""
        self.job_result = None          # (ok, message) once finished, taken by update_widget
        self.cancel_requested = False

        self.available = False
        self.unavailable_reason = ""
        self.last_update_error = None

        self.main_layout = QVBoxLayout(self)
        self.main_layout.setSpacing(10)
        self.init_ui()
        self.reload_goals()

    # ----- UI ------------------------------------------------------------------------------------

    def init_ui(self):
        header = QLabel("<b>MoveIt</b>")
        header.setStyleSheet("font-size: 16px;")
        self.main_layout.addWidget(header)

        # --- target: program or single goal ---
        target_card = QFrame()
        target_card.setFrameShape(QFrame.Shape.StyledPanel)
        target_layout = QVBoxLayout(target_card)
        target_layout.setContentsMargins(8, 6, 8, 6)
        target_layout.addWidget(QLabel("<b>Target</b>"))

        row = QHBoxLayout()
        self.target_combo = QComboBox()
        self.target_combo.currentIndexChanged.connect(self.update_ui_lock_state)
        row.addWidget(self.target_combo, stretch=1)
        self.btn_reload = QPushButton("⟳")
        self.btn_reload.setFixedWidth(32)
        self.btn_reload.setToolTip("Reload the goals file")
        self.btn_reload.clicked.connect(self.reload_goals)
        row.addWidget(self.btn_reload)
        target_layout.addLayout(row)

        motion_row = QHBoxLayout()
        motion_label = QLabel("Motion:")
        motion_label.setStyleSheet("color: #95a5a6;")
        motion_row.addWidget(motion_label)
        self.motion_combo = QComboBox()
        self.motion_combo.addItems(MOTIONS)
        self.motion_combo.setToolTip("PTP: joint space, all axes synchronous\n"
                                     "LIN: straight tcp line\n"
                                     "OMPL: collision-free path\n"
                                     "(programs define the motion per step)")
        motion_row.addWidget(self.motion_combo)
        motion_row.addStretch()
        target_layout.addLayout(motion_row)

        self.goals_file_label = QLabel("")
        self.goals_file_label.setWordWrap(True)
        self.goals_file_label.setStyleSheet("color: #7f8c8d; font-size: 10px;")
        target_layout.addWidget(self.goals_file_label)
        self.main_layout.addWidget(target_card)

        # --- speed, same look as the jog pages ---
        speed_container = QFrame()
        speed_container.setStyleSheet(
            "background-color: #2c3e50; border-radius: 8px; border: 1px solid #3498db;")
        speed_layout = QVBoxLayout(speed_container)
        speed_header = QHBoxLayout()
        speed_title = QLabel("Speed Scale")
        speed_title.setStyleSheet("font-weight: bold; color: #3498db; border: none;")
        self.speed_display = QLabel("20%")
        self.speed_display.setStyleSheet("font-weight: bold; color: #f1c40f; border: none;")
        speed_header.addWidget(speed_title)
        speed_header.addStretch()
        speed_header.addWidget(self.speed_display)
        speed_layout.addLayout(speed_header)
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        # Scaling of joint_limits.yaml / pilz_cartesian_limits.yaml, the motion_server takes (0, 1]
        self.speed_slider.setRange(5, 100)
        self.speed_slider.setValue(20)
        self.speed_slider.valueChanged.connect(lambda val: self.speed_display.setText(f"{val}%"))
        speed_layout.addWidget(self.speed_slider)
        self.main_layout.addWidget(speed_container)

        # --- plan / execute / cancel ---
        buttons = QHBoxLayout()
        self.btn_plan = QPushButton("PLAN")
        self.btn_plan.setToolTip("Plan only and show the result in RViz, nothing moves")
        self.btn_plan.setStyleSheet(BUTTON_STYLE.format(color="#2980b9"))
        self.btn_plan.clicked.connect(lambda: self.send_job(plan_only=True))
        self.btn_execute = QPushButton("EXECUTE")
        self.btn_execute.setToolTip("Plan everything, then execute (robot must be ARMED)")
        self.btn_execute.setStyleSheet(BUTTON_STYLE.format(color="#2ecc71"))
        self.btn_execute.clicked.connect(lambda: self.send_job(plan_only=False))
        self.btn_cancel = QPushButton("CANCEL")
        self.btn_cancel.setToolTip("Cancel the running job, the robot stops and holds")
        self.btn_cancel.setStyleSheet(BUTTON_STYLE.format(color="#e74c3c"))
        self.btn_cancel.clicked.connect(self.cancel_job)
        for btn in (self.btn_plan, self.btn_execute, self.btn_cancel):
            btn.setMinimumHeight(36)
            buttons.addWidget(btn)
        self.main_layout.addLayout(buttons)

        self.status_label = QLabel("")
        self.status_label.setWordWrap(True)
        self.main_layout.addWidget(self.status_label)

        self.result_label = QLabel("")
        self.result_label.setWordWrap(True)
        self.result_label.hide()
        self.main_layout.addWidget(self.result_label)

        # --- teach ---
        teach_card = QFrame()
        teach_card.setFrameShape(QFrame.Shape.StyledPanel)
        teach_layout = QVBoxLayout(teach_card)
        teach_layout.setContentsMargins(8, 6, 8, 6)
        teach_layout.addWidget(QLabel("<b>Teach current pose</b>"))
        self.teach_name = QLineEdit()
        self.teach_name.setPlaceholderText("goal name, e.g. pick_above")
        teach_layout.addWidget(self.teach_name)
        teach_buttons = QHBoxLayout()
        self.btn_teach_joint = QPushButton("JOINT GOAL")
        self.btn_teach_joint.setToolTip("Save the current joint values (unambiguous)")
        self.btn_teach_joint.clicked.connect(lambda: self.teach(cartesian=False))
        self.btn_teach_cart = QPushButton("CARTESIAN GOAL")
        self.btn_teach_cart.setToolTip("Save the tcp pose, with the current joints as IK seed")
        self.btn_teach_cart.clicked.connect(lambda: self.teach(cartesian=True))
        teach_buttons.addWidget(self.btn_teach_joint)
        teach_buttons.addWidget(self.btn_teach_cart)
        teach_layout.addLayout(teach_buttons)
        self.main_layout.addWidget(teach_card)

        self.main_layout.addStretch()
        self.update_ui_lock_state()

    def execution_mode(self) -> str:
        # MoveIt trajectories are position controlled (no compliant variant yet)
        return POSITION

    # ----- goals file ----------------------------------------------------------------------------

    def reload_goals(self):
        previous = self.target_combo.currentData()
        self.target_combo.blockSignals(True)
        self.target_combo.clear()
        try:
            goal_file = GoalFile(self.goals_file, load_named_states())
        except (GoalFileError, OSError) as e:
            self.show_result(False, f"Goals file: {e}")
            goal_file = None
        if goal_file is not None:
            for name in sorted(goal_file.programs):
                steps = len(goal_file.programs[name])
                self.target_combo.addItem(f"▶ {name}  ({steps} steps)", (PROGRAM, name))
            for name, goal in sorted(goal_file.goals.items()):
                kind = 'joint' if isinstance(goal, JointGoal) else 'cartesian'
                source = ', SRDF' if goal.source == 'srdf' else ''
                self.target_combo.addItem(f"● {name}  ({kind}{source})", (GOAL, name))
        index = self.find_target(*previous) if previous else -1
        self.target_combo.setCurrentIndex(max(index, 0))
        self.target_combo.blockSignals(False)
        self.goals_file_label.setText(self.goals_file)
        self.update_ui_lock_state()

    def find_target(self, kind, name):
        """Index of a combo entry (PROGRAM/GOAL, name) or -1 (findData cannot compare tuples)."""
        for i in range(self.target_combo.count()):
            if self.target_combo.itemData(i) == (kind, name):
                return i
        return -1

    def fetch_goals_file(self):
        """Read the goals_file parameter of the motion_server once it is available."""
        if self.goals_file_read or self.goals_file_pending or not self.ros_ok() \
                or not self.param_client.service_is_ready():
            return
        self.goals_file_pending = True
        future = self.param_client.call_async(GetParameters.Request(names=['goals_file']))

        def done(f):
            self.goals_file_pending = False
            try:
                value = f.result().values[0].string_value
            except Exception:
                return
            if value:
                self.goals_file = value
                self.goals_file_read = True
        future.add_done_callback(done)

    # ----- jobs ----------------------------------------------------------------------------------

    def send_job(self, plan_only):
        data = self.target_combo.currentData()
        if data is None or self.job_running:
            return
        kind, name = data
        goal = RunMotion.Goal()
        if kind == PROGRAM:
            goal.program = name
        else:
            goal.goal = name
            goal.motion = self.motion_combo.currentText()
        goal.scaling = self.speed_slider.value() / 100.0
        goal.plan_only = plan_only

        self.job_running = True
        self.job_plan_only = plan_only
        self.job_phase = "SENDING"
        self.cancel_requested = False
        self.result_label.hide()
        what = f"program '{name}'" if kind == PROGRAM else f"{goal.motion} to '{name}'"
        self.node.get_logger().info(
            f"{self.name}: {'plan' if plan_only else 'execute'} {what} at {goal.scaling:.0%}")

        future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)
        self.update_ui_lock_state()

    def feedback_cb(self, msg):
        fb = msg.feedback
        self.job_phase = fb.phase + (f" {fb.segment}/{fb.segments}" if fb.segment else "")

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.job_result = (False, "Rejected by the motion_server (another job running?)")
            self.job_running = False
            return
        self.goal_handle = handle
        if self.cancel_requested:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        response = future.result()
        result = response.result
        ok = response.status == GoalStatus.STATUS_SUCCEEDED and result.success
        self.job_result = (ok, result.message or "no message")
        self.goal_handle = None
        self.job_running = False

    def cancel_job(self):
        self.cancel_requested = True
        handle = self.goal_handle
        if handle is not None:
            handle.cancel_goal_async()
        self.node.get_logger().warn(f"{self.name}: job cancelled by the operator")

    # ----- teach ---------------------------------------------------------------------------------

    def joint_state_cb(self, msg):
        self.joint_state = msg

    def teach(self, cartesian):
        name = self.teach_name.text().strip()
        if not GOAL_NAME.match(name):
            self.show_result(False, "Goal name: letters, digits and _ only, starting with a letter")
            return
        msg = self.joint_state
        joints = dict(zip(msg.name, msg.position)) if msg is not None else {}
        if not all(j in joints for j in JOINTS):
            self.show_result(False, "No joint state of all six axes received yet")
            return
        joints = {j: joints[j] for j in JOINTS}
        try:
            goal_file = GoalFile(self.goals_file, load_named_states())
            if name in goal_file.goals and goal_file.goals[name].source == 'srdf':
                self.show_result(False, f"'{name}' is a named state of the SRDF, pick another name")
                return
            if cartesian:
                pose = self.store.get_current_tcp_pose()
                if pose is None or self.store.get_base_frame() != FRAME:
                    self.show_result(False, f"No tcp pose in frame '{FRAME}' available (TF)")
                    return
                goal = CartesianGoal(name, tuple(pose[:3]), quat_from_rpy(*pose[3:]), seed=joints)
            else:
                goal = JointGoal(name, joints)
            replaced = name in goal_file.goals
            goal_file.set_goal(goal)
            goal_file.save()
        except (GoalFileError, OSError) as e:
            self.show_result(False, f"Teach failed: {e}")
            return
        kind = "Cartesian" if cartesian else "joint"
        self.show_result(True, f"{'Replaced' if replaced else 'Saved'} {kind} goal '{name}'")
        self.node.get_logger().info(f"{self.name}: taught {kind} goal '{name}' "
                                    f"({', '.join(f'{math.degrees(v):.1f}' for v in joints.values())} deg)")
        self.reload_goals()
        index = self.find_target(GOAL, name)
        if index >= 0:
            self.target_combo.setCurrentIndex(index)

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

    def update_ui_lock_state(self):
        status = self.store.get_status()
        foreign_mission = bool(status["active_nodes"]) and MOTION_SERVER not in status["active_nodes"]
        idle = self.available and not self.job_running
        has_target = self.target_combo.currentData() is not None
        is_goal = has_target and self.target_combo.currentData()[0] == GOAL

        self.target_combo.setEnabled(not self.job_running)
        self.btn_reload.setEnabled(not self.job_running)
        self.motion_combo.setEnabled(not self.job_running and is_goal)
        self.speed_slider.setEnabled(not self.job_running)
        self.btn_plan.setEnabled(idle and has_target)
        self.btn_execute.setEnabled(idle and has_target and status["armed"] and not foreign_mission)
        self.btn_cancel.setEnabled(self.job_running)
        self.btn_teach_joint.setEnabled(not self.job_running)
        self.btn_teach_cart.setEnabled(not self.job_running)

        if self.job_running:
            verb = "Planning" if self.job_plan_only else "Running"
            self.status_label.setText(f"{verb}: {self.job_phase}")
            self.status_label.setStyleSheet("color: #f1c40f; font-weight: bold;")
        elif not self.available:
            self.status_label.setText(self.unavailable_reason)
            self.status_label.setStyleSheet("color: #7f8c8d;")
        elif not status["armed"]:
            self.status_label.setText("Robot not armed: PLAN only")
            self.status_label.setStyleSheet("color: #95a5a6;")
        elif foreign_mission:
            self.status_label.setText(f"Mission of {status['active_nodes']} active: PLAN only")
            self.status_label.setStyleSheet("color: #95a5a6;")
        else:
            self.status_label.setText("Ready")
            self.status_label.setStyleSheet("color: #2ecc71;")

    def update_widget(self):
        """
        Called cyclically by the main window timer (~30 Hz).

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

    def ros_ok(self):
        return rclpy.ok(context=self.node.context)

    def _update(self):
        # after Ctrl-C rclpy is shut down while the window still runs: no more ROS calls
        server = self.ros_ok() and self.action_client.server_is_ready()
        loaded = self.store.get_loaded_controllers()
        controller = loaded is None or TRAJECTORY_CONTROLLER in loaded
        self.available = server and controller
        if not server:
            self.unavailable_reason = ("motion_server not running "
                                       "(robot_bringup launch_robot.py, moveit:=true)")
        elif not controller:
            self.unavailable_reason = (f"{TRAJECTORY_CONTROLLER} not loaded on the RT PC "
                                       "(firmware patch, see robotarm_moveit_config doc/)")

        if server:
            self.fetch_goals_file()
            if self.goals_file_read and self.goals_file_label.text() != self.goals_file:
                self.reload_goals()

        if self.job_result is not None:
            ok, message = self.job_result
            self.job_result = None
            self.show_result(ok, message)

        self.update_ui_lock_state()
