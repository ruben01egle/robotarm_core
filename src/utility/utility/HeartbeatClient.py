from interface.msg import HeartbeatQuery, Heartbeat

class HeartbeatClient():
    def __init__(self, node):
        self.node = node

        self.node.create_subscription(HeartbeatQuery, 'heartbeat/query', self.heartbeat_cb, 10)
        self.pub = self.node.create_publisher(Heartbeat, 'heartbeat/response', 5)

    def heartbeat_cb(self, heartbeat):
        heartbeat_resp = Heartbeat()
        heartbeat_resp.query_id = heartbeat.query_id
        heartbeat_resp.original_sent_timestamp = heartbeat.sent_timestamp
        node_name_bytes = self.node.get_name().encode('utf-8')
        heartbeat_resp.node_name = node_name_bytes.ljust(32, b'\x00')[:32]

        self.pub.publish(heartbeat_resp)

