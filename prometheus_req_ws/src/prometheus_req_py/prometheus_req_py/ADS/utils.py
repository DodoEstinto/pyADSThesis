
from enum import IntEnum
import tkinter as tk
import time
from prometheus_req_interfaces.action import CallFunctionBlock
class reqState(IntEnum):
    '''
    Enum representing the possible states of the FSM.
    '''
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

class msgType(IntEnum):
    '''
    Enum representing the type of message sent via action feedback.
    0: NORMAL - A normal message.
    1: ERROR_CHECK - A message indicating an error check is required.
    2: ASK_PICTURE_SCREW - A message requesting a picture of a screw.
    3: ASK_PICTURE_VCHECK - A message requesting a picture for visual calibration.
    '''
    NORMAL = 0
    ERROR_CHECK = 1
    ASK_PICTURE_SCREW = 2
    ASK_PICTURE_VCHECK = 3

class inputType(IntEnum):
    '''
    Enum representing the type of input requested from the user.
    The input type is used to determine how the user should respond to a request.
    0: CALLBLOCK - The user is prompted to call a function block.
    1: ERROR_CHECK - The user is prompted to check for an error.
    2: YES_NO - The user is prompted to answer a yes/no question.
    3: INTEGER - The user is prompted to enter an integer value.
    4: FLOAT - The user is prompted to enter a float value.
    5: OK - The user is prompted to acknowledge with an OK.
    '''
    CALLBLOCK = 0
    ERROR_CHECK = 1
    YES_NO = 2
    INTEGER = 3
    FLOAT = 4
    OK = 5
    SCREW_SLOT = 6
    


class roiIds(IntEnum):
    '''
    Enum representing the ROI IDs.
    '''
    P0=0
    P1=1
    P2=2
    P3=3

class calibrationPlanes(IntEnum):
    '''
    Enum representing the calibration planes.
    '''
    Distance_10Cm=0
    Distance_13Cm=1
    Distance_15Cm=2

def getReqStateMsg(state: int) -> str:
    '''
    Returns a default message corresponding to the given state.
    Args:
        state (int): The state code to map to a message.
    Returns:
        str: A message corresponding to the state code.
    If the state code is not recognized, returns "State not mapped!".
    '''
    state_mapping = {
        0:     ("Success! Ready!"),
        40:    ("Waiting for the client to take action..."),
        30:    ("Request completed!"),
        -20:   ("Stopped!"),
        -30:   ("ERROR!"),
        -40:   ("ERROR CHECK!"),
        20:    ("Still running!"),
        100:    ("Still running!"),
        110:    ("Still running!"),
        120:    ("Still running!"),
        
    }
    return state_mapping.get(state,("State not mapped!"))


def publishFeedback(self,goalHandler, msg: str, actionTimerDelay: float = 0.5):
    '''
    Publishes feedback to the action server.
    Args:
        goalHandler: The goal handler for the action server.
        msg (str): The message to publish as feedback.
        actionTimerDelay (float): Delay between feedback messages in seconds.
    '''
    if(time.time()-self.lastTime>self.actionTimerDelay):
            self.lastTime=time.time()
            feedback_msg = CallFunctionBlock.Feedback()
            feedback_msg.msg_type=msgType.NORMAL
            feedback_msg.msg=msg
            goalHandler.publish_feedback(feedback_msg)

