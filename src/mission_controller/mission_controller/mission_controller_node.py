import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from enum import IntEnum
import time

from interface.msg import SystemState
from interface.action import Mission
from interface.srv import RequestAction

from utility.HeartbeatClient import HeartbeatClient
from utility.RequestActionClient import RequestActionClient

class MissionControllerNode(Node):
    class State(IntEnum):
        IDLE = 0
        PLANNING = 1
        EXECUTING = 2
        CONTINIOUS = 3

    def __init__(self):
        super().__init__('mission_controller')

        self.callback_group = ReentrantCallbackGroup()

        self.heartbeat_client = HeartbeatClient(self)
        self.request_action_client = RequestActionClient(self)

        self.create_subscription(SystemState, 'system_state', self.system_state_cb, 1)

        self._action_server = ActionServer(
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

    def system_state_cb(self, msg):
            self.system_state = msg.state

    def mission_execute_cb(self, goal_handle):
        self.get_logger().info(f"entering execution")
        request = goal_handle.request
        feedback_msg = Mission.Feedback()
        result = Mission.Result()

        while self.system_state != SystemState.MISSION:
            self.get_logger().info("Waiting for state transition")
            time.sleep(0.5)

        self.get_logger().info("Starting mission")

        # --- PHASE 1: PLANNING ---
        feedback_msg.phase = "PLANNING"
        feedback_msg.planning_progress = 0.0

        # Simulierter Planungsfortschritt
        for i in range(1, 11):
            if goal_handle.is_cancel_requested:
                self.request_action_client.send_request(RequestAction.Request.ACTION_MISSION, RequestAction.Request.TYPE_STOP)
                goal_handle.canceled()
                result.success = False
                result.final_status = "Planning cancelled"
                return result
            
            feedback_msg.planning_progress = i * 10.0
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        # --- PHASE 2: EXECUTING ---
        feedback_msg.phase = "EXECUTING"
        feedback_msg.execution_progress = 0.0
        
        for i in range(1, 11):
            if goal_handle.is_cancel_requested:
                self.request_action_client.send_request(RequestAction.Request.ACTION_MISSION, RequestAction.Request.TYPE_STOP)
                goal_handle.canceled()
                result.success = False
                result.final_status = "Execution cancelled"
                return result

            feedback_msg.execution_progress = i * 10.0
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        # --- FINISH ---
        goal_handle.succeed()
        if not self.request_action_client.send_request(RequestAction.Request.ACTION_MISSION, RequestAction.Request.TYPE_STOP):
            result.success = False
            self.get_logger().error("Mission complete but state machine rejected termination")
        else:
            result.success = True
            result.final_status = "Mission complete"
        return result

    def mission_goal_cb(self, goal_request):
        if goal_request.option == Mission.Goal.OPTION_SET_JOINT_ANGLES:
            self.get_logger().info(f"Mission set joint angles requested: {goal_request.target_joint_angles})")
        elif goal_request.option == Mission.Goal.OPTION_CSV:
            self.get_logger().info(f"Mission csv requested: {goal_request.csv_path})")

        if not self.request_action_client.send_request(RequestAction.Request.ACTION_MISSION, RequestAction.Request.TYPE_START):
            self.get_logger().error(f"Mission declined")
            return GoalResponse.REJECT
        
        self.planning_progress = 0
        self.executing_progress = 0
        self.get_logger().info(f"returning goal")
        return GoalResponse.ACCEPT

    def mission_cancel_cb(self, goal_handle):
        self.get_logger().info("Mission stop requested")
        return CancelResponse.ACCEPT

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