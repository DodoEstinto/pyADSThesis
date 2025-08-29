
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
    '''
    NORMAL = 0
    ERROR_CHECK = 1
    ASKING_PICTURE = 2
class OkDialog(tk.Toplevel):
    def __init__(self, parent, title="Message", message=""):
        super().__init__(parent)
        self.result = False  
        self.title(title)
        self.geometry("300x120")
        self.resizable(False, False)

        self.label = tk.Label(self, text=message, wraplength=280)
        self.label.pack(pady=20)

        self.ok_button = tk.Button(self, text="OK", width=10, command=self.on_ok)
        self.ok_button.pack(pady=(0, 10))

        self.grab_set()     # Make this window modal
        self.transient(parent)  # Show on top of parent
        self.wait_window()  # Wait for this window to close

    def on_ok(self):
        self.result = True
        self.destroy()

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
            feedback_msg.msg_type=msg_type.NORMAL
            feedback_msg.msg=msg
            goalHandler.publish_feedback(feedback_msg)

