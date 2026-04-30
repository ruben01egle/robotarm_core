import rclpy
from rclpy.time import Time
from transitions import Machine
from rclpy.node import Node
from enum import IntEnum
import numpy as np


from interface.msg import SystemState, HardwareActions, HardwareCommand, HardwareFeedback
from interface.srv import RequestAction

from utility.HeartbeatClient import HeartbeatClient

class HardwareSimulationNode(Node):
    class State(IntEnum):
        IDLE = SystemState.IDLE
        CONNECTED = SystemState.CONNECTED
        ARMED = SystemState.ARMED
        MISSION = SystemState.MOTION
        CONFIG = SystemState.CONFIG
        ERROR = SystemState.ERROR
        EMERGENCY_HALT = SystemState.EMERGENCY_HALT

    state: State

    def __init__(self):
        super().__init__('control_center_node')

        self.heartbeat_client = HeartbeatClient(self)

        self.command_id = 0
        self.pending_command = False

        transitions = [
            {'trigger': 'connect', 'source': self.State.IDLE, 'dest': self.State.CONNECTED},
            {'trigger': 'enter_config', 'source': self.State.CONNECTED, 'dest': self.State.CONFIG},
            {'trigger': 'exit_config', 'source': self.State.CONFIG, 'dest': self.State.CONNECTED},
            {'trigger': 'arm', 'source': self.State.CONNECTED, 'dest': self.State.ARMED},
            {'trigger': 'disarm', 'source': self.State.ARMED, 'dest': self.State.CONNECTED},
            {'trigger': 'start_mission', 'source': self.State.ARMED, 'dest': self.State.MISSION},
            {'trigger': 'end_mission', 'source': self.State.MISSION, 'dest': self.State.ARMED},
            {'trigger': 'error', 'source': '*', 'dest': self.State.ERROR},
            {'trigger': 'emergency', 'source': '*', 'dest': self.State.EMERGENCY_HALT},
        ]
        self.machine = Machine(model=self, states=self.State, transitions=transitions, initial=self.State.CONNECTED)

        self.hardware_feedbaack_pub = self.create_publisher(HardwareFeedback, 'hardware/feedback', 1)

        self.create_subscription(HardwareCommand, 'hardware/command', self.hardware_command_cb, 1)

        self.get_logger().info('Hardware sim node running')

    def hardware_command_cb(self, msg):
        action_map = {
            (HardwareActions.ARM): ("arm"),
            (HardwareActions.DISARM):  ("disarm"),
            
            (HardwareActions.ENTER_CONFIG): ("enter_config"),
            (HardwareActions.EXIT_CONFIG):  ("exit_config"),
            
            (HardwareActions.START_MISSION): ("start_mission"),
            (HardwareActions.END_MISSION):  ("end_mission"),
        }
        response = HardwareFeedback()
        response.command_id = msg.command_id
        response.action = msg.action
        
        trigger = action_map[msg.action]
        try:
            getattr(self, trigger)()
            response.success = True
            response.current_state = self.state
        except:
            response.success = False
        self.hardware_feedbaack_pub.publish(response)

def main(args=None):
    rclpy.init(args=args)
    
    ros_node = HardwareSimulationNode()
    
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()