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
            
            if trajectory is not None and len(trajectory) > 0:
                self.get_logger().info(f"Planning successful. Points: {len(trajectory)}")
                
                for i, point in enumerate(trajectory):
                    frame = TrajectoryFrame()
                    frame.idx = int(i)
                    
                    # Dynamische Bestimmung der Gelenk-Anzahl
                    num_joints = len(point.positions)
                    
                    for j in range(num_joints):
                        # Wir erstellen den Attributnamen dynamisch (axis1, axis2, ...)
                        # Da ROS-Messages oft 1-basiert benannt sind (axis1):
                        axis_attr_name = f"axis{j+1}"
                        
                        # Prüfen, ob das Feld in der Message überhaupt existiert
                        if hasattr(frame, axis_attr_name):
                            axis = AxisData()
                            axis.position = float(point.positions[j])
                            axis.velocity = float(point.velocities[j])
                            axis.torque   = float(point.torques[j])
                            
                            setattr(frame, axis_attr_name, axis)
                        else:
                            # Optional: Warnung, wenn mehr Gelenke geplant wurden als die Message unterstützt
                            if i == 0: # Nur einmal loggen
                                self.get_logger().warn(f"Message TrajectoryFrame hat kein Feld {axis_attr_name}!")
                    
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