
from enum import IntEnum
import tkinter as tk
class req_state(IntEnum):
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

class msg_type(IntEnum):
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

def get_req_state_msg(state: int) -> str:
    state_mapping = {
        0:     ("Success! Robot Ready!"),
        40:    ("Waiting for the client to take action..."),
        -20:   ("Stopped!"),
        -30:   ("ERROR!"),
        -40:   ("ERROR CHECK!"),
        20:    ("Still running!"),
        100:    ("Still running!"),
        110:    ("Still running!"),
        120:    ("Still running!"),
        
    }
    return state_mapping.get(state,(False,"Invalid State!"))



def get_req_type(val:str) -> int:
    block_mapping = {
        "loadTray": 3,
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
    