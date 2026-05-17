from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from interface.msg import TrajectoryBatch, TrajectoryFeedback

class TrajectoryExecutioner():
    def __init__(self, node, callback_group=None):
        self.node = node

        self.clear_feedback_handler()

        self.trajectory_id = 0
        self.packet_num = 0
        self.last_send_idx = 0
        self.last_hardware_idx = 0
        self.trajectory = []

        self._is_running = False
        self._success = False

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.node.create_subscription(TrajectoryFeedback, 'trajectory/feedback', self.trajectory_feedback_cb, 5, callback_group=callback_group)

        self.trajectory_pub = self.node.create_publisher(TrajectoryBatch, 'trajectory/data', qos_profile)


    def start_execution(self, trajectory):
        if self._is_running:
            return False
        self.trajectory = trajectory
        if len(self.trajectory) <= 0:
            self.node.get_logger().error(f"Executioner revieved empty trajectory")
            return False
        self.trajectory_id += 1
        self.last_send_idx = 0
        self.last_hardware_idx = 0
        self._is_running = True
        self._success = False
        self._start_confirmed = False

        start_msg = TrajectoryBatch()
        start_msg.trajectory_id = self.trajectory_id
        start_msg.trajectory_status = TrajectoryBatch.START

        self.node.get_logger().debug("Waiting for START_ACK...")
        timeout_counter = 0
        rate = self.node.create_rate(10)
        while not self._start_confirmed and self._is_running:
            if timeout_counter % 5 == 0:
                self.trajectory_pub.publish(start_msg)
            if timeout_counter > 100:
                self.node.get_logger().error("START_ACK timeout")
                self._is_running = False
            timeout_counter += 1
            rate.sleep()
        self.node.get_logger().debug("START_ACK recieved")

        return True
    
    def append_trajectory(self, trajectory):
        if not self._is_running:
            return False
        self.trajectory.extend(trajectory)

    def trajectory_feedback_cb(self, msg):
        if msg.trajectory_id == self.trajectory_id:
            if msg.trajectory_status == TrajectoryFeedback.START_ACK:
                self._start_confirmed = True
                
            elif msg.trajectory_status == TrajectoryFeedback.REQUEST_DATA:
                self.node.get_logger().debug(f"Recieved request for {msg.request_next_count} frames")
                self.last_hardware_idx = msg.current_hardware_idx
                self.send_next_packets(msg.request_next_count)
                
            elif msg.trajectory_status == TrajectoryFeedback.END_REACHED:
                self.node.get_logger().debug("Executioner recieved END")
                self.last_hardware_idx = msg.current_hardware_idx
                self._is_running = False
                self._success = True
            
            elif msg.trajectory_status == TrajectoryFeedback.ERROR:
                self.node.get_logger().error("Trajectory Execution ended with error")
                self._is_running = False
                self._success = False

        else:
            self.node.get_logger().error("Wrong trajectory id recieved")
            self._is_running = False

        if self._feedback_cb:
            self._feedback_cb(((self.last_hardware_idx+1) / len(self.trajectory))*100.0)

    def send_next_packets(self, count_requested):
        """Sendet die nächsten N Punkte in 10er Batches."""
        sent_in_this_call = 0

        if self.last_send_idx < len(self.trajectory):
            while sent_in_this_call < count_requested and self.last_send_idx < len(self.trajectory):
                batch = TrajectoryBatch()
                batch.trajectory_id = self.trajectory_id
                
                upper_limit = min(self.last_send_idx + 10, len(self.trajectory))        # TODO: magic number
                batch.data = self.trajectory[self.last_send_idx : upper_limit]

                if upper_limit >= len(self.trajectory):
                    self.node.get_logger().debug("Sending last trajectory batch")
                    batch.trajectory_status = TrajectoryBatch.END
                else:
                    batch.trajectory_status = TrajectoryBatch.RUNNING
                
                num_points = len(batch.data)
                batch.packet_num = self.packet_num
                self.trajectory_pub.publish(batch)
                
                self.packet_num += 1
                self.last_send_idx += num_points
                sent_in_this_call += num_points

                import time
                if count_requested > 100:
                    time.sleep(0.05)
        
        else:
            redundant_batch = TrajectoryBatch()
            redundant_batch.trajectory_id = self.trajectory_id
            redundant_batch.trajectory_status = TrajectoryBatch.END
            
            if len(self.trajectory) > 0:
                redundant_batch.data = [self.trajectory[-1]]
            
            self.trajectory_pub.publish(redundant_batch)
            self.node.get_logger().debug("Resending last point with END status.")

    def cancel(self):
        self._is_running = False

    def is_done(self):
        return not self._is_running

    def is_success(self):
        return self._success
    
    def set_feedback_handler(self, callback):
        self._feedback_cb = callback

    def clear_feedback_handler(self):
        self._feedback_cb = None

