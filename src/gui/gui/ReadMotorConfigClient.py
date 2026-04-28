from rclpy.action import ActionClient
from interface.action import ReadMotorConfigs
from action_msgs.msg import GoalStatus

class ReadMotorActionClient:
    def __init__(self, node, callback_on_result):
        """
        :param node: Die ROS-Node (für Logger und Client-Erstellung)
        :param callback_on_result: Funktion, die aufgerufen wird, wenn Daten da sind
        """
        self.node = node
        self.on_result_user_cb = callback_on_result
        
        self.client = ActionClient(self.node, ReadMotorConfigs, 'config/read')
        
        self._goal_future = None
        self._result_future = None

    def request_action(self):
        """Startet die Action."""
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error("Action Server to read motor config not online")
            return

        goal_msg = ReadMotorConfigs.Goal()
        
        self.node.get_logger().info("Request query to read motor params")
        
        self._goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self._internal_feedback_cb
        )
        self._goal_future.add_done_callback(self._internal_goal_response_cb)

    def _internal_goal_response_cb(self, future):
        """Loggt, ob die Hardware die Anfrage akzeptiert hat."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Query denied")
            return

        self.node.get_logger().info("Query accepted")
        
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._internal_result_cb)

    def _internal_feedback_cb(self, feedback_msg):
        """Loggt den Fortschritt (Feedback)."""
        params_count = feedback_msg.feedback.params_read
        self.node.get_logger().info(f"Feedback: {params_count} params read")

    def _internal_result_cb(self, future):
        """Verarbeitet das Endergebnis und gibt es an die Node weiter."""
        result_handle = future.result()
        
        if result_handle.status == GoalStatus.STATUS_SUCCEEDED:
            result = result_handle.result
            self.node.get_logger().info(f"Succes {len(result.values)} axis configurations read")
            
            # Hier geben wir nur die relevanten Daten an die äußere Node zurück
            self.on_result_user_cb(result.values, result.success)
        else:
            self.node.get_logger().error(f"Action failed: {result_handle.status}")
            self.on_result_user_cb([], False)