class RobotArmConfig:
    TA_MS = 1
    TA = float(TA_MS) / 1000

    TRAJECTORY_BATCH_SIZE = 10
    TELEMETRY_BATCH_SIZE = 10

    COMP_POS_SCALE = 1.0
    COMP_VEL_SCALE = 1.0
    COMP_TRQ_SCALE = 1.0