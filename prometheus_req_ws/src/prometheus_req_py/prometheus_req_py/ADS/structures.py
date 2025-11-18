import ctypes

class ScrewSlot_ctype(ctypes.Structure):
    '''
    Structure representing a screw slot in the PLC.
    '''
    _pack_ = 1
    _fields_ = [
        ("MAX_IDX_X", ctypes.c_uint16),
        ("MAX_IDX_Y", ctypes.c_uint16),
        ("nextIdxX", ctypes.c_uint16),
        ("nextIdxY", ctypes.c_uint16),
    ]

class PLC_STRING_40(ctypes.Structure):
    '''
    Structure representing a PLC string of length 40.
    '''
    _pack_ = 1
    _fields_ = [
        ("data", ctypes.c_char * 41),      # string chars
    ]

class EquipmentStatus_ctype(ctypes.Structure):
    '''
    Structure representing the equipment status in the PLC.
    '''
    _pack_ = 1
    _fields_ = [
        ("emGeneral", ctypes.c_bool),
        ("emMR", ctypes.c_bool),
        ("emSR", ctypes.c_bool),
        ("temSensOk", ctypes.c_bool),
        ("airPressOk", ctypes.c_bool),
        ("emLaserScanner", ctypes.c_bool),
        ("emLaserScanner2", ctypes.c_bool),
        ("emLaserScanner3", ctypes.c_bool),
        ("automaticMode", ctypes.c_bool),
        ("mgseToConveyor", ctypes.c_bool),
        ("trolleyInBay", ctypes.c_bool),
        ("padding", ctypes.c_bool), #to compensate alignment issue.
        ("side2Robot", ctypes.c_uint16),
        ("positionerIsUp", ctypes.c_bool),
        ("positionerIsDown", ctypes.c_bool),
        ("palletIsInWrkPos", ctypes.c_bool),
        ("palletIsInEntryPos", ctypes.c_bool),
        ("rotaryAligned", ctypes.c_bool),
        ("holderCorrectionDone", ctypes.c_bool),
        ("screwBay", ScrewSlot_ctype * 6),
        ("trayIsGripped", ctypes.c_bool),
        ("activeStateFSMString", PLC_STRING_40), 
        ("activeStateMRFSMString", PLC_STRING_40),
        ("activeStateSRFSMString", PLC_STRING_40),
        ("activeStateConveyorString", PLC_STRING_40),
        ("activeStateSystemSafetyTest", PLC_STRING_40),
    ]
