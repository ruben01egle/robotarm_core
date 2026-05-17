import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


from interface.msg import SystemState, TelemetryBatch, JointAngles
from interface.action import Mission, PlanJointSpace, PlanCSV
from interface.srv import RequestAction

from .PlannerActionClient import PlannerActionClient
from .TrajectoryExecutioner import TrajectoryExecutioner
from utility.HeartbeatClient import HeartbeatClient
from utility.RequestActionClient import RequestActionClient

class MissionControllerNode(Node):
    def __init__(self):
        super().__init__('mission_controller')

        self.callback_group = ReentrantCallbackGroup()

        qos_profile_telemetry = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.heartbeat_client = HeartbeatClient(self)
        self.request_action_client = RequestActionClient(self)
        self.plan_p2p_jointspace_client = PlannerActionClient(self,
                                                               "plan_trajectory/p2p_jointspace",
                                                                PlanJointSpace,
                                                               callback_group=self.callback_group)
        self.plan_csv_client = PlannerActionClient(self,
                                                    "plan_trajectory/csv",
                                                    PlanCSV,
                                                    callback_group=self.callback_group)
        self.executener = TrajectoryExecutioner(self, callback_group=MutuallyExclusiveCallbackGroup())

        self.create_subscription(SystemState, 'system_state', self.system_state_cb, 1)
        self.create_subscription(TelemetryBatch, 'telemetry', self.telemetry_cb, qos_profile_telemetry)

        self._mission_action_server = ActionServer(
            self,
            Mission,
            'mission',
            execute_callback=self.mission_execute_cb,
            goal_callback=self.mission_goal_cb,
            cancel_callback=self.mission_cancel_cb,
            callback_group=self.callback_group
        )

        self.planning_progress = 0
        self.executing_progress = 0

        self.system_state = SystemState.IDLE
        self.current_robot_frame = None
        self.current_joint_angles = None
        self.planned_trajectory = []

    def system_state_cb(self, msg):
            self.system_state = msg.state

    def telemetry_cb(self, msg: TelemetryBatch):
        if not msg.data:
            self.get_logger().warn("Telemetrie message without data revieved")
            return
        
        self.current_robot_frame = msg.data[-1] # type: ignore
        self.current_joint_angles = [axis.position for axis in self.current_robot_frame.axes]

    def mission_goal_cb(self, goal_request):
        if goal_request.option == Mission.Goal.OPTION_SET_JOINT_ANGLES:
            self.get_logger().info(f"Mission set joint angles requested: {goal_request.target_joint_angles}, scale: {goal_request.motion_scale})")
        elif goal_request.option == Mission.Goal.OPTION_CSV:
            self.get_logger().info(f"Mission csv requested: {goal_request.csv_path})")

        if not self.request_action_client.send_request(RequestAction.Request.ACTION_MISSION, RequestAction.Request.TYPE_START):
            self.get_logger().error(f"Mission declined")
            return GoalResponse.REJECT
        
        self.planning_progress = 0
        self.executing_progress = 0
        return GoalResponse.ACCEPT

    def mission_execute_cb(self, goal_handle):
        request = goal_handle.request
        result = Mission.Result()

        self.planned_trajectory = []

        rate = self.create_rate(2)
        while self.system_state != SystemState.MISSION:
            self.get_logger().debug("Waiting for state transition")
            rate.sleep()

        self.get_logger().info("Starting mission")

        # --- PHASE 1: PLANNING ---
        success = False
        if request.option == Mission.Goal.OPTION_SET_JOINT_ANGLES:
            success = self.task_plan_p2p_jointspace(goal_handle, request.target_joint_angles, request.motion_scale)
        elif request.option == Mission.Goal.OPTION_CSV:
            success = self.task_plan_csv(goal_handle, request.csv_path)

        if not success:
            self.get_logger().error("Planning phase failed")
            goal_handle.abort()
            self.request_action_client.send_request(RequestAction.Request.ACTION_MISSION, RequestAction.Request.TYPE_STOP)
            return Mission.Result(success=False)
        
        # --- PHASE 2: EXECUTING ---
        success = self.task_execute_trajectory(goal_handle, self.planned_trajectory)
    
        if not success:
            self.get_logger().error("Executing phase failed")
            goal_handle.abort()
            self.request_action_client.send_request(RequestAction.Request.ACTION_MISSION, RequestAction.Request.TYPE_STOP)
            return Mission.Result(success=False)

        # --- FINISH ---
        goal_handle.succeed()
        if not self.request_action_client.send_request(RequestAction.Request.ACTION_MISSION, RequestAction.Request.TYPE_STOP):
            result.success = False
            self.get_logger().error("Mission complete but state machine rejected termination")
        else:
            result.success = True
            result.final_status = "Mission complete"
        return result

    def mission_cancel_cb(self, goal_handle):
        self.get_logger().info("Mission stop requested")
        return CancelResponse.ACCEPT

    def publish_mission_feedback(self, goal_handle, phase):
        msg = Mission.Feedback()
        msg.planning_progress = float(self.planning_progress)
        msg.execution_progress = float(self.executing_progress)
        msg.phase = phase
        goal_handle.publish_feedback(msg)

    # TODO: Refactor tasks into a separate library to reduce boilerplate (try-finally, feedback handling) and improve maintainability.

    def task_plan_p2p_jointspace(self, goal_handle, target_angles, scale):
        request = PlanJointSpace.Goal()

        start_pt = JointAngles()
        start_pt.joint_angles = self.current_joint_angles
        target_pt = JointAngles()
        target_pt.joint_angles = target_angles

        request.waypoints = [start_pt, target_pt]
        request.motion_scale = scale

        def handle_progress(progress):
            self.planning_progress = progress
            self.publish_mission_feedback(
                goal_handle, 
                "PLANNING"
            )
        self.plan_p2p_jointspace_client.set_feedback_handler(handle_progress)

        try:
            self.plan_p2p_jointspace_client.request_plan_trajectory(request)
            success = self.wait_for_task(goal_handle,
                                        self.plan_p2p_jointspace_client.is_planning_done,
                                        self.plan_p2p_jointspace_client.is_success,
                                        cancel_func=self.plan_p2p_jointspace_client.cancel)
            if success:
                self.planned_trajectory = self.plan_p2p_jointspace_client.trajectory
                if len(self.planned_trajectory) <= 0:
                    self.get_logger().error(f"Recieved empty trajectory")
                    success = False  
            return success
        finally:
            self.plan_p2p_jointspace_client.clear_feedback_handler()
    
    def task_plan_csv(self, goal_handle, csv):
        request = PlanCSV.Goal()
        request.csv_path = csv

        def handle_progress(progress):
            self.planning_progress = progress
            self.publish_mission_feedback(
                goal_handle, 
                "PLANNING"
            )
        self.plan_csv_client.set_feedback_handler(handle_progress)

        try:
            self.plan_csv_client.request_plan_trajectory(request)
            success = self.wait_for_task(goal_handle,
                                        self.plan_csv_client.is_planning_done,
                                        self.plan_csv_client.is_success,
                                        cancel_func=self.plan_csv_client.cancel)
            if success:
                    self.planned_trajectory = self.plan_csv_client.trajectory
            return success
        finally:
            self.plan_csv_client.clear_feedback_handler()
    
    def task_execute_trajectory(self, goal_handle, trajectory):
        def handle_progress(progress):
            self.executing_progress = progress
            self.publish_mission_feedback(
                goal_handle, 
                "EXECUTING"
            )
        self.executener.set_feedback_handler(handle_progress)
        try:
            if not self.executener.start_execution(trajectory):
                return False
            return self.wait_for_task(
                goal_handle,
                check_done_func=self.executener.is_done,
                check_success_func=self.executener.is_success,
                cancel_func=self.executener.cancel,
                timeout_sec=300.0
            )
        finally:
            self.executener.clear_feedback_handler()

    def wait_for_task(self, goal_handle, check_done_func, check_success_func, cancel_func=None, timeout_sec=60.0):
        """
        Universelle Hilfsmethode, um auf den Abschluss eines Sub-Tasks zu warten.
        
        :param goal_handle: Das Handle der aktuellen Mission-Action (um Abbrüche zu prüfen).
        :param check_done_func: Eine Funktion/Methode, die True zurückgibt, wenn der Task fertig ist.
        :param timeout_sec: Maximale Zeit in Sekunden, bevor die Methode mit False abbricht.
        :return: True, wenn der Task erfolgreich beendet wurde, False bei Abbruch oder Timeout.
        """
        start_time = self.get_clock().now()
        self.get_logger().debug("Waiting for sub-task to complete...")

        rate = self.create_rate(20)
        while not check_done_func():
            # 1. Überprüfen, ob die übergeordnete Mission vom User abgebrochen wurde
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("Sub-task wait interrupted: Mission was cancelled by user.")
                if cancel_func:
                    cancel_func()
                return False

            # 2. Überprüfen auf Zeitüberschreitung (Timeout)
            current_time = self.get_clock().now()
            elapsed_time = (current_time - start_time).nanoseconds / 1e9 # Umrechnung in Sekunden
            
            if elapsed_time > timeout_sec:
                self.get_logger().error(f"Sub-task wait timed out after {timeout_sec} seconds.")
                if cancel_func:
                    cancel_func()
                return False

            rate.sleep()

        if not check_success_func():
            self.get_logger().error("Sub-task finished, but reported FAILURE.")
            return False

        self.get_logger().debug("Sub-task completed successfully.")
        return True

def main(args=None):
    rclpy.init(args=args)
    
    ros_node = MissionControllerNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()