import time
from rclpy.node import Node
from robotarm_interface.srv import RequestAction

class RequestActionClient:
    def __init__(self, node: Node):
        self.node = node
        self.client = self.node.create_client(RequestAction, 'request_action')

    def send_request(self, action_id, request_type, blocking=True, controller_names=""):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error('Service not available')
            return False if blocking else None

        request = RequestAction.Request()
        request.node_name = self.node.get_name()
        request.action_id = action_id
        request.request_type = request_type
        request.controller_names = controller_names

        if blocking:
            # --- SYNCHRONER MODUS ---
            rate = self.node.create_rate(10)
            future = self.client.call_async(request)
            while not future.done(): rate.sleep()
            
            return self._process_response(future)
        else:
            # --- ASYNCHRONER MODUS ---
            future = self.client.call_async(request)
            future.add_done_callback(lambda f: self._process_response(f))
            return None

    def _process_response(self, future):
        try:
            response = future.result()
            
            if not response.success:
                self.node.get_logger().error(f"Service Error: {response.message}")
            
            return response.success
        except Exception as e:
            self.node.get_logger().error(f"Service call failed: {e}")
            return False