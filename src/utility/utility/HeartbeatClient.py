from interface.msg import HeartbeatQuery, Heartbeat

class HeartbeatClient():
    def __init__(self, node):
        self.node = node

        self.last_query_time = self.node.get_clock().now()

        self.sub = self.node.create_subscription(HeartbeatQuery, 'heartbeat/query', self.heartbeat_cb, 1)
        self.pub = self.node.create_publisher(Heartbeat, 'heartbeat/response', 1)

    def heartbeat_cb(self, heartbeat):
        self.last_query_time = self.node.get_clock().now()
        heartbeat_resp = Heartbeat()
        heartbeat_resp.query_id = heartbeat.query_id
        heartbeat_resp.original_sent_timestamp = heartbeat.sent_timestamp
        
        node_name_bytes = self.node.get_name().encode('utf-8')
        padded_bytes = node_name_bytes.ljust(32, b'\x00')[:32]
        heartbeat_resp.node_name = list(padded_bytes)

        self.pub.publish(heartbeat_resp)

    def destroy(self):
        if self.node:
            self.node.destroy_subscription(self.sub)
            self.node.destroy_publisher(self.pub)
            self.node.get_logger().info(f"HeartbeatClient for {self.node.get_name()} destroyed.")

    def get_last_query_time(self):
        return self.last_query_time

