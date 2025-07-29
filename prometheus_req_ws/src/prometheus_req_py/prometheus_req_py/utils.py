
from enum import Enum
class req_state(Enum):
    ST_ERROR_CHECK = -40
    ST_ERROR = -30
    ST_STOP = -20,
    ST_SYS_NOT_READY=-10,
    ST_READY = 0,
    ST_EXECUTING = 20,
    ST_REQ_COMPLETION=30,
    ST_REQ_PENDING=40,
    ST_EXECUTING_2=100,
    ST_EXECUTING_3=110,
    ST_EXECUTING_4=120

def get_req_state_result_msg(state: int) -> tuple[bool, str]:
    state_mapping = {
        0:     (True,  "Success! Robot Ready!"),
        40:    (True,  "Waiting for the client to take action..."),
        -20:   (False, "Stopped!"),
        -30:   (False, "ERROR!"),
        -40:   (False, "ERROR CHECK!")
    }

def get_req_type(val:str) -> int:
    block_mapping = {
        "loadTray": 0,
        "pickUpTray": 0,
        "depositTray": 0,
        "srHoming": 0,
        "mrHoming": 0,
        "mrTrolleyVCheck": 0,
        "gyroGrpRot": 0,
        "screwTight": 0,
        "screwPickup": 0,
        "presentToScrew": 0,
        "positionerRotate": 1,
        "stackTray": 0,
        "present2Op": 0,
        "setScrewBayState": 1
        }
    return block_mapping.get(val, 0)  # 0 = default value for unknown strings.
    