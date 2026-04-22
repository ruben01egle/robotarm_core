import ctypes

# Like #pragma pack(push, 4)
class PackedStructure(ctypes.Structure):
    _pack_ = 4

class CAxisDataFloat(PackedStructure):
    _fields_ = [
        ("position", ctypes.c_float),
        ("velocity", ctypes.c_float),
        ("torque", ctypes.c_float),
    ]

class CTrajectoryData(PackedStructure):
    _fields_ = [
        ("idx", ctypes.c_uint32),
        ("axis1", CAxisDataFloat),
        ("axis2", CAxisDataFloat),
        ("axis3", CAxisDataFloat),
        ("axis4", CAxisDataFloat),
        ("axis5", CAxisDataFloat),
        ("axis6", CAxisDataFloat),
    ]

class CAxisDataInt16(PackedStructure):
    _fields_ = [
        ("position", ctypes.c_int16),
        ("velocity", ctypes.c_int16),
        ("torque", ctypes.c_int16),
    ]

class CTelemetryData(PackedStructure):
    _fields_ = [
        ("timeUs", ctypes.c_uint32),
        ("idx", ctypes.c_uint32),
        ("axis1", CAxisDataInt16),
        ("axis2", CAxisDataInt16),
        ("axis3", CAxisDataInt16),
        ("axis4", CAxisDataInt16),
        ("axis5", CAxisDataInt16),
        ("axis6", CAxisDataInt16),
    ]