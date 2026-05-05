from rclpy.action import ActionClient
from interface.action import PlanTrajectory

import time

class PlanTrajectoryClient:
    def __init__(self, node, callback_on_feedback, channel, callback_group=None):
        self.node = node
        self.on_feedback_user_cb = callback_on_feedback
        
        self.client = ActionClient(self.node, PlanTrajectory, channel, callback_group=callback_group)
        
        self._result_future = None
        self._goal_handle = None

        self._success = False
        self.trajectory = []

    def request_plan_trajectory(self, option, csv_path, target_joint_angles):
        """Startet die Action."""
        self._result_future = None
        self.trajectory = []
        self._success = False
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error("Action Server to plan trajectory not online")
            return False
        goal_msg = PlanTrajectory.Goal()
        goal_msg.option = option
        if csv_path:
            goal_msg.csv_path = csv_path
        if target_joint_angles is not None:
            goal_msg.target_joint_angles = target_joint_angles
        
        self.node.get_logger().info("Request plan trajectory")
        
        send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.on_feedback_user_cb
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
    
    def is_planning_done(self):
        if self._result_future is None:
            return False
        return self._result_future.done()
    
    def is_success(self):
        return self._success

    def cancel_current_goal(self):
        if self._goal_handle is not None:
            self.node.get_logger().info('sending cancel plan trajcetory request')
            self._goal_handle.cancel_goal_async()
        else:
            self.node.get_logger().warn('No active goal')