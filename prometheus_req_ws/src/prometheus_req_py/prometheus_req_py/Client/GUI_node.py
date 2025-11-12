# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


'''
FOR THE MOMENT REFER TO client_GUI.PY FOR THE GUI IMPLEMENTATION.
'''

'''
from copy import deepcopy
from prometheus_req_interfaces.msg import EquipmentStatus, Offset,ScrewSlot
from prometheus_req_interfaces.action import CallFunctionBlock
from prometheus_req_py.ADS.utils import msgType
from prometheus_req_py.Client.utils import OkDialog,ScrewDialog,ScrewBayDialog
import prometheus_req_py.ADS.constants as ADS_CONSTANTS
import tkinter as tk
from tkinter import messagebox,simpledialog
from std_msgs.msg import Empty
from functools import partial
from prometheus_req_py.Client.GUI import client_GUI
from prometheus_req_interfaces.srv import SetScrewBayState
import requests
import urllib3
import ast

class GUI_node():
    '''
    This node is designed to be used directly by a human operator. 
    It provides a simplified interface for monitoring and controlling the system.
    '''

  

    def start_sequence(self):  
        '''
        Initiate a sequence of building block calls.
        '''
        self.inSequence=True

        if(not self.errorChecked):
            choosenSequence=SequenceDialog(self.root,title="Starting Sequence",options=ADS_CONSTANTS.BUILDING_BLOCKS).result
            if(choosenSequence is None):
                choosenSequence=[]
            self.sequence = iter(choosenSequence)
        else:

            self.updateResponseText("Sequence interrupted because error check, resuming sequence.", isResult=False)

        self.errorChecked=False
        self.sequenceAborted = False
        self.goNext=True
        if(not self.functionBlockCalled):

            self.call_next_block()
        else:

            self.updateResponseText("Cannot start sequence, another function block is running.", isResult=False)
            self.inSequence=False

    def call_next_block(self):
        '''
        Call the next block in the sequence.
        '''
        if self.sequenceAborted:

            self.sequenceAborted = False
            self.functionBlockCalled = False
            return
        while(not self.goNext):
            self.root.update()
        if(self.errorChecked):
            self.inSequence=False
            self.functionBlockCalled=False
            return
        self.goNext=False
        try:
            block = next(self.sequence)

            self.call_block(block, override=True)
            self.call_next_block()
        except StopIteration:
            self.inSequence=False
            self.functionBlockCalled = False

            self.updateResponseText("Sequence completed!", isResult=False)



    def call_block(self, name:str,override=False) -> None:
        pass

    def __init__(self,root):
        '''
        Initialize the client node.
        :param root: The root window of the Tkinter GUI.
        '''
        super().__init__()
        self.init_GUI = partial(client_GUI.init_GUI,self)
        self.initLabels = partial(client_GUI.initLabels,self)
        self.updateLabels = partial(client_GUI.updateLabels,self)
        self.updateResponseText = partial(client_GUI.updateResponseText,self)
        self.state=EquipmentStatus()

        self.functionBlockCalled=False
        self.functionBlockDone=False
        self.functionBlockState="N/A"
        self.functionBlockMsg="N/A"
        self.functionBlockResult=False
        self.inSequence=False
        self.sequenceAborted=False
        self.goNext=False
        self.errorChecked=False
        self.init_GUI(root)


    def state_update_callback(self, msg: EquipmentStatus) -> None:
        '''
        Called each time the plc's equipmentstate changes.
        It updates the local state variable and the labels of the GUI.
        :param msg: The new state of the equipment.
        '''
        #Testing code

        if(msg is None):

            return
        self.state=deepcopy(msg)
        self.updateLabels()


    def calculate_picture_offset(self,askedAction,calibrationPlane=0,roiId=0,findScrew=False) -> tuple[bool,float,float,float]:
            """
            Calculate the offset of the picture.
            :param askedAction: The type of action to perform (ASK_PICTURE_SCREW or ASK_PICTURE_VCHECK).
            :param calibrationPlane: The calibration plane to use (only for ASK_PICTURE_SCREW).
            :param roiId: The ROI ID to use (only for ASK_PICTURE_SCREW).
            :param findScrew: Whether to find the screw in the picture (only for ASK_PICTURE_SCREW).
            :return: A tuple containing a boolean indicating if the data is valid, and the x, y, theta offsets.
            """
            ATS_IP = '10.10.10.100'

            parameters={"calibrationPlane":calibrationPlane,"roiId":roiId,"findScrew":findScrew}
            if(askedAction==msgType.ASK_PICTURE_SCREW):
                Command="GetScrewCorrection"
            else:
                Command="GetTrayCorrection"


            urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
            try:
                response = requests.get(f'https://{ATS_IP}/{Command}',params=parameters,verify=False).json()
            except Exception as e:

                return False,0.0,0.0,0.0


            if(type(response) is str):
                response=ast.literal_eval(response.replace('false',"False").replace('true','True'))

            dataValid=response["DataValid"]
            if(dataValid is False):

                return False,0.0,0.0,0.0
            #Not used for now
            #circleFound=response["CircleFound"]
            #if(circleFound is False):

            #    return False,0.0,0.0,0.0
            translationX=response["TranslationX"]
            translationY=response["TranslationY"]
            rotation=response["Rotation"]




            return dataValid,translationX, translationY, rotation

def main(args=None):

    root=tk.Tk()
    node = GUI_node(root)


    def on_close():
        '''
        This function is called when the window is closed.
        '''


        node.destroy_node()
        root.destroy()  # closes the window and ends mainloop

    # Bind the close event to the on_close function
    root.protocol("WM_DELETE_WINDOW", on_close)  # handles window close
    root.mainloop()




#TODO: needed?
if __name__ == '__main__':
    main()

'''