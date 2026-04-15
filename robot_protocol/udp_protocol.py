from enum import IntEnum
import ctypes
from .data_types import PackedStructure, CTrajectoryData, CTelemetryData
from .robotarm_config import RobotArmConfig

class MessageType(IntEnum):
    HEARTBEAT = 0x01
    SYSTEM_COMMAND = 0x02
    SYSTEM_LOG = 0x03
    MSG_STATUS = 0x04
    TRAJECTORY = 0x05
    TELEMETRY = 0x06
    CANFD_TUNNEL = 0x07

class SystemMode(IntEnum):
    IDLE = 0x01
    CONNECTED = 0x02
    ARMED = 0x03
    MOTION = 0x04
    CONFIG = 0x05
    ERROR = 0x66
    EMERGENCY_HALT = 0xFF


UDP_MAGIC_BYTE = 0x112233FF

class CPacketHeader(PackedStructure):
    _fields_ = [
        ("mMagicByte", ctypes.c_uint32),
        ("mMsgType",   ctypes.c_uint32), # MessageType Enum
        ("mTimestamp", ctypes.c_uint32),
        ("mMode",      ctypes.c_uint32), # SystemMode Enum
    ]

class CRobotHeartbeatMsg(PackedStructure):
    _fields_ = [
        ("mHeader", CPacketHeader),
        ("mData",   CTrajectoryData),
    ]

class CSystemCMDMsg(PackedStructure):
    _fields_ = [
        ("mHeader", CPacketHeader),
        ("mMsgID",  ctypes.c_uint32),
        ("mCMD",    ctypes.c_uint32), # SystemCMD Enum
    ]

class CMsgStatusMsg(PackedStructure):
    _fields_ = [
        ("mHeader",         CPacketHeader),
        ("mInitialMsgType", ctypes.c_uint32),
        ("mInitialMsgID",   ctypes.c_uint32),
        ("mMsgStatus",      ctypes.c_uint32), # MsgStatus Enum
        ("mError",          ctypes.c_uint32), # ErrorCode Enum
        ("mInfo",           ctypes.c_char * 128),
    ]

class CSystemLogMsg(PackedStructure):
    _fields_ = [
        ("mHeader", CPacketHeader),
        ("mError",  ctypes.c_uint32),
        ("mInfo",   ctypes.c_char * 256),
    ]

class CTrajectoryMsg(PackedStructure):
    _fields_ = [
        ("mHeader",       CPacketHeader),
        ("mPacketNum",    ctypes.c_uint32),
        ("mNumDataPoints",ctypes.c_uint32),
        ("mData",         CTrajectoryData * RobotArmConfig.TRAJECTORY_BATCH_SIZE),
    ]

class CTelemetryMsg(PackedStructure):
    _fields_ = [
        ("mHeader",       CPacketHeader),
        ("mPacketNum",    ctypes.c_uint32),
        ("mNumDataPoints",ctypes.c_uint32),
        ("mData",         CTelemetryData * RobotArmConfig.TELEMETRY_BATCH_SIZE),
    ]

class CCANFDMsg(PackedStructure):
    _fields_ = [
        ("mHeader",    CPacketHeader),
        ("mMsgID",     ctypes.c_uint32),
        ("mDataLength",ctypes.c_uint32),
        ("mData",      ctypes.c_uint8 * 64),
    ]