from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from interface.msg import TrajectoryFrame, AxisData

class GenericPlannerNode(Node):
    def __init__(self, node_name, action_name, action_type, planner_instance):
        super().__init__(node_name)
        self.planner_logic = planner_instance

        self.action_type = action_type

        self.callback_group = ReentrantCallbackGroup()
        
        self._action_server = ActionServer(
            self,
            action_type,
            action_name,
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_cb,
            callback_group=self.callback_group
        )

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        
        def send_ros_feedback(progress_value):
            feedback_msg = self.action_type.Feedback()
            feedback_msg.progress = float(progress_value)
            goal_handle.publish_feedback(feedback_msg)

        def is_canceled():
            return goal_handle.is_cancel_requested
        
        result = self.action_type.Result()

        try:
            trajectory = self.planner_logic.calculate(
                request, 
                progress_cb=send_ros_feedback,
                is_canceled_cb=is_canceled
            )
        except Exception as e:
            self.get_logger().error(f"Error raised in execute callback: {e}")
            result.success = False
            result.trajectory = []
            goal_handle.abort()
            return result


        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            self.get_logger().info("Planning execution canceled")
            return result
        
        if trajectory is not None and len(trajectory) > 0:
            ros_trajectory = []
            num_points = len(trajectory)
            self.get_logger().info(f"Planning successful. Generated {num_points} trajectory points.")
            for i, point in enumerate(trajectory):
                frame = TrajectoryFrame()
                
                # SEHR WICHTIG: Den Index befüllen!
                frame.idx = int(i)
                # Angenommen point ist eine Liste/Array mit 6 Werten
                for i in range(1, 7):
                    axis = AxisData()
                    axis.position = float(point[i-1])
                    # Falls dein Planer auch Geschwindigkeiten liefern würde:
                    # axis.velocity = float(point[i-1 + offset]) 
                    setattr(frame, f"axis{i}", axis)
                
                ros_trajectory.append(frame)
        
            result.success = True
            result.trajectory = ros_trajectory
            goal_handle.succeed()

        else:
            result.success = False
            result.trajectory = []
            goal_handle.abort()
            self.get_logger().error("Planning failed or returned empty trajectory")
        return result
        
    def cancel_cb(self, goal_handle):
        self.get_logger().info("Planning stop requested")
        return CancelResponse.ACCEPT