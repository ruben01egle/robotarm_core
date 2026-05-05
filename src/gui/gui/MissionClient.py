from rclpy.action import ActionClient
from interface.action import Mission
from action_msgs.msg import GoalStatus

class MissionClient:
    def __init__(self, node, callback_on_feedback):
        """
        :param node: Die ROS-Node (für Logger und Client-Erstellung)
        :param callback_on_feedback: Funktion, die aufgerufen wird, wenn Daten da sind
        """
        self.node = node
        self.on_feedback_user_cb = callback_on_feedback
        
        self.client = ActionClient(self.node, Mission, 'mission')
        
        self._goal_future = None
        self._result_future = None
        self._goal_handle = None

    def request_mission(self, option, csv_path, target_joint_angles):
        """Startet die Action."""
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error("Action Server to start mission not online")
            return
        goal_msg = Mission.Goal()
        goal_msg.option = option
        if csv_path:
            goal_msg.csv_path = csv_path
        if target_joint_angles is not None:
            goal_msg.target_joint_angles = [float(x) for x in target_joint_angles]
        
        self.node.get_logger().info("Request mission start")
        
        self._goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.on_feedback_user_cb
        )
        self._goal_future.add_done_callback(self._internal_goal_response_cb)

    def _internal_goal_response_cb(self, future):
        """Loggt, ob die Hardware die Anfrage akzeptiert hat."""
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.node.get_logger().error("Mission denied")
            return

        self.node.get_logger().info("Mission accepted")
        
        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self._internal_result_response_cb)

    def _internal_result_response_cb(self, future):
        result_handle = future.result()
        result = result_handle.result
        self._goal_handle = None
        if result_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f"Mission complete: {result.final_status}")
        else:
            self.node.get_logger().error(f"Mission failed: {result.final_status}")

    def cancel_current_goal(self):
        if self._goal_handle is not None:
            self.node.get_logger().info('sending cancel mission request')
            self._goal_handle.cancel_goal_async()
        else:
            self.node.get_logger().warn('No active goal')