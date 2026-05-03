from interface.msg import SystemState

STATE_MAP = {
        SystemState.IDLE: ("IDLE"),
        SystemState.CONNECTED: ("CONNECTED"),
        SystemState.ARMED: ("ARMED"),
        SystemState.MISSION: ("MOTION"),
        SystemState.CONFIG: ("CONFIG"),
        SystemState.ERROR: ("ERROR"),
        SystemState.EMERGENCY_HALT: ("EMERGENCY")
    }