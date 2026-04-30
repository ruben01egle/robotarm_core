import rclpy
from rclpy.node import Node
from interface.srv import RequestAction

class RequestActionClient:
    def __init__(self, node: Node):
        self.node = node
        self.client = self.node.create_client(RequestAction, 'request_action')

    def send_request(self, action_id, request_type, blocking=True):
        """
        Sendet eine Anfrage.
        :param blocking: Wenn True, wartet die Funktion und gibt bool zurück.
                         Wenn False, läuft sie asynchron und loggt nur Fehler.
        """
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error('Service not available')
            return False if blocking else None

        request = RequestAction.Request()
        node_name_bytes = self.node.get_name().encode('utf-8')
        padded_bytes = node_name_bytes.ljust(32, b'\x00')[:32]
        request.node_name = list(padded_bytes)
        request.action_id = action_id
        request.request_type = request_type

        if blocking:
            # --- SYNCHRONER MODUS ---
            future = self.client.call_async(request)
            # Blockiert den aktuellen Thread, bis die Antwort da ist
            rclpy.spin_until_future_complete(self.node, future)
            
            return self._process_response(future)
        else:
            # --- ASYNCHRONER MODUS ---
            future = self.client.call_async(request)
            # Interner Callback ohne Rückgabewert an den Aufrufer
            future.add_done_callback(lambda f: self._process_response(f))
            return None

    def _process_response(self, future):
        """Zentrale Auswertung der Antwort."""
        try:
            response = future.result()
            error_msg = bytes(response.message).decode('utf-8').strip('\x00')
            
            if not response.success:
                self.node.get_logger().error(f"Service Error: {error_msg}")
            
            return response.success
        except Exception as e:
            self.node.get_logger().error(f"Service call failed: {e}")
            return False

    def _string_to_uint8_array(self, source_str, length):
        encoded_bytes = source_str.encode('utf-8')[:length]
        return encoded_bytes.ljust(length, b'\x00')