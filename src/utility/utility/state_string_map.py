from interface.msg import SystemState

STATE_MAP = {
        SystemState.IDLE: ("IDLE"),
        SystemState.CONNECTED: ("CONNECTED"),
        SystemState.ARMED: ("ARMED"),
        SystemState.MISSION: ("MOTION"),
        SystemState.CONFIG: ("CONFIG"),
        SystemState.SOFT_STOP: ("SOFT_STOP"),
        SystemState.HARD_STOP: ("HARD_STOP"),
        SystemState.ERROR: ("ERROR"),
        SystemState.EMERGENCY: ("EMERGENCY")
    }