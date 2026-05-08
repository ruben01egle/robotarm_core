from rclpy.action import ActionClient

import time

class PlannerActionClient:
    def __init__(self, node, channel, action_type, callback_group=None):
        self.node = node
        self.clear_feedback_handler()
        
        self.client = ActionClient(self.node, action_type, channel, callback_group=callback_group)
        
        self._result_future = None
        self._goal_handle = None

        self._success = False
        self.trajectory = []

    def request_plan_trajectory(self, request):
        """Startet die Action."""
        self._result_future = None
        self.trajectory = []
        self._success = False
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error("Action Server to plan trajectory not online")
            return False
        goal_msg = request
        
        self.node.get_logger().info("Request plan trajectory")
        
        send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_cb
        )

        while not send_goal_future.done():
            time.sleep(0.01)

        self._goal_handle = send_goal_future.result()
        if self._goal_handle is None:
            self.node.get_logger().error("Goal Handle is none - server did not reply")
            return False

        if not self._goal_handle.accepted:
            self.node.get_logger().error("Plan trajectory denied")
            return False

        self.node.get_logger().info("Plan trajectory accepted")
        
        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_cb)

        return True
    
    def result_cb(self, future):
        result = future.result().result

        if result.success:
            self.node.get_logger().info("PlanTrajectoryClient: Trajectory received")
            self.trajectory = result.trajectory
            self._success = True
        else:
            self.node.get_logger().error("PlanTrajectoryClient: Planning failed on server side")

    def feedback_cb(self, msg):
        progress = msg.feedback.progress
        if self._user_feedback_cb:
            self._user_feedback_cb(progress)
    
    def is_planning_done(self):
        if self._result_future is None:
            return False
        return self._result_future.done()
    
    def is_success(self):
        return self._success
    
    def set_feedback_handler(self, callback):
        self._user_feedback_cb = callback

    def clear_feedback_handler(self):
        self._user_feedback_cb = None

    def cancel(self):
        if self._goal_handle is not None:
            self.node.get_logger().info('sending cancel plan trajcetory request')
            self._goal_handle.cancel_goal_async()
        else:
            self.node.get_logger().warn('No active goal')