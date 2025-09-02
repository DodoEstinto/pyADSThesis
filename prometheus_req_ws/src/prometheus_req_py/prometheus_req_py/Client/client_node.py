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

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from copy import deepcopy
from prometheus_req_interfaces.msg import EquipmentStatus, Offset
from prometheus_req_interfaces.action import CallFunctionBlock
from prometheus_req_py.ADS.utils import msgType
from prometheus_req_py.Client.utils import OkDialog,ScrewDialog
import tkinter as tk
from tkinter import messagebox,simpledialog
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from functools import partial
from prometheus_req_py.Client.GUI import client_GUI
import requests

class Client_Node(Node):
    '''
    This node is designed to be used directly by a human operator. 
    It provides a simplified interface for monitoring and controlling the system.
    '''

    def call_block(self, name:str) -> None:
        '''
        This function is called when the building block buttons are pressed.
        It sends an async request to the service server.
        :param name: The name of the function block to call.
        '''
        cancelAction=False
        #Prevent to call a second function call while the previous one is still executing.
        if not self.functionBlockCalled:
            self.functionBlockCalled=True
            self.get_logger().info(f"[Client_node] Calling Block {name}")
            self.req.function_block_name=name
            #based on the function called, a different parameter is asked.
            match(name):
                case "loadTray":
                    answer = messagebox.askyesnocancel("Load Tray", "Load(Yes) or Unload(No) the tray?")
                    self.req.bool_param1=answer
                    if(answer is None):
                        cancelAction=True
                case "positionerRotate":
                    answer = messagebox.askyesnocancel("Positioner Rotate", "Rotate clockwise(Yes) or anticlockwise(No)?")
                    self.req.bool_param1=answer
                    if(answer is None):
                        cancelAction=True
                case "screwPickup":
                    screw = simpledialog.askinteger("Pick Up Screw", "Enter the screw number to pick up (1-6):", minvalue=1, maxvalue=6)
                    self.req.int_param1=screw
                    if(screw is None):
                        cancelAction=True
                case "screwTight":
                    response=ScrewDialog(self.root, title="Screw Tightening Parameters")
                    if(response.result is None):
                        cancelAction=True
                    else:
                        self.req.float_param1=response.result["screwX"]
                        self.req.float_param2=response.result["screwY"]
                        self.req.float_param3=response.result["screwZ"]
                        self.req.int_param1=response.result["targetToUse"]
                        self.req.int_param2=response.result["focalPlane"]
                        self.req.int_param3=response.result["screwRecipeID"]
                        self.req.bool_param1=True if response.result["screwArea"]=="inside" else False
            if(cancelAction):
                self.functionBlockCalled=False
                self.get_logger().info(f"[Client_node] Action cancelled by the user.")
                self.updateResponseText("Action cancelled by the user.", isResult=False)
            else:
                #Ask the permission to run the function block.
                self.send_goal_future=self.client.send_goal_async(self.req,feedback_callback=self.goal_feedback_callback)
                #Tell where you are waiting for a response.
                self.send_goal_future.add_done_callback(self.goal_response_callback)
                self.get_logger().info(f"[Client_node] Function Block already called, waiting for response...")
                self.updateResponseText("A function Block has been already called, waiting for its response...", isResult=False)

    def goal_response_callback(self,future: rclpy.Future) -> None:
        '''
        This function is called when the action client receive a response.
        :future: The future containing the response of the action.
        '''
        goalHandler=future.result()
        self.get_logger().info(f"[CLIENT NODE] Action response:Entering")

        #If the action is refused. Even if the ads node never refuses,
        #this has been added for anomaly robusteness.
        if not goalHandler.accepted:
            self.get_logger().info(f"[CLIENT NODE] Action response:Not accepted")
            self.updateResponseText("Command not accepted", isResult=False)
            self.functionBlockCalled=False
            return
        
        self.get_logger().info(f"[CLIENT NODE] Action response: Accepted")
        self.updateResponseText("Command accepted", isResult=False)

        #Ask for the result.
        self.send_goal_future= goalHandler.get_result_async()
        #Tell where you are waiting for a response.
        self.send_goal_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self,future: rclpy.Future) -> None:
        '''
        This function is called when the action client receive a result.
        :future: The future containing the result of the action.
        '''
        self.functionBlockResult=future.result().result.success
        self.functionBlockState=future.result().result.state
        self.functionBlockMsg=future.result().result.msg
        self.functionBlockCalled=False
  
        self.updateLabels()  #TODO: needed?
        self.updateResponseText(self.functionBlockMsg, isResult=True)

    def goal_feedback_callback(self,feedback: CallFunctionBlock.Feedback) -> None:
        '''
        This function is called when the action client receive a feedback.
        :feedback: The feedback received from the action server.
        '''
        feedbackMsgType=feedback.feedback.msg_type
        self.functionBlockMsg=feedback.feedback.msg
        #Based on the msg type manage the feedback differently.
        #Refer to msgType documentation for more informations about it
        match feedbackMsgType:
            case msgType.ERROR_CHECK:
                self.updateResponseText("Error check in progress...", isResult=False)
                _=OkDialog(self.root, title="Error Check", message=self.functionBlockMsg)
                self.errorCheckPub.publish(Empty())
            case msgType.ASKING_PICTURE:
                self.updateResponseText("Asking a photo...", isResult=False)
                _=OkDialog(self.root, title="Take Picture", message="Press ok to take a picture!")
                #TODO: to set!
                offset=self.calculate_picture_offset(0,0,True)
                offsetMsg=Offset()
                offsetMsg.x=offset[0]
                offsetMsg.y=offset[1]
                offsetMsg.theta=offset[2]
                self.offsetPub.publish(offsetMsg)
                self.get_logger().info(f"[Client_node] Published offset: {offsetMsg}")
            case _:
                self.updateLabels() #TODO: needed?
                self.updateResponseText(self.functionBlockMsg, isResult=False)
        
    def __init__(self,root):
        super().__init__('client_node')
        self.init_GUI = partial(client_GUI.init_GUI,self)
        self.initLabels = partial(client_GUI.initLabels,self)
        self.updateLabels = partial(client_GUI.updateLabels,self)
        self.updateResponseText = partial(client_GUI.updateResponseText,self)
        self.state=EquipmentStatus()
        stateQos=QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.stateSub = self.create_subscription(
            EquipmentStatus,
            'state',
            self.state_update_callback,
            qos_profile=stateQos)
        
        #it's essential that we do not lost the acks due to a networking failure
        #so we guarantee that samples are delivered, also trying multiple times.
        qos = QoSProfile(
            depth=3,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.errorCheckPub= self.create_publisher(
            Empty,
            'errorCheckAck',
            qos_profile=qos
        )

        self.offsetPub = self.create_publisher(
            Offset,
            'offset',
            qos_profile=qos
        )

        self.functionBlockCalled=False
        self.functionBlockDone=False
        self.functionBlockState="N/A"
        self.functionBlockMsg="N/A"
        self.functionBlockResult=False
        self.init_GUI(root)
        self.client=ActionClient(self,CallFunctionBlock,"CallFunctionBlock")
        while not self.client.wait_for_server(timeout_sec=1):
            #self.get_logger().info('service not available, waiting again...')
            pass
        self.req=CallFunctionBlock.Goal()
        self.stateSub  # prevent unused variable warning

    def state_update_callback(self, msg: EquipmentStatus) -> None:
        '''
        Called each time the plc's equipmentstate changes.
        It updates the local state variable and the labels of the GUI.
        :param msg: The new state of the equipment.
        '''
        #Testing code
        #self.get_logger().info("[Client_node]Receiving:"+str(msg.active_state_fsm_string))
        if(msg is None):
            self.get_logger().error("[Client_node]Received None msg!")
            return
        self.state=deepcopy(msg)
        self.updateLabels()


    def calculate_picture_offset(self,calibrationPlane,roiId,findScrew) -> tuple[float,float,float]:
            """
            #TODO: test on real PLC!
            Dummy function, as the actual implementation depends on the specific requirements of the application.
            Calculate the offset of the picture.
            :return: The offset of the picture.
            """
            #params={"calibrationPlane":calibrationPlane,"roiId":roiId,"findScrew":findScrew}
            #response = requests.post(f"{ATC_IP}", json=params)
            #data = response.json()
            #circleFound=data["CircleFound"]
            #translationX=data["TranslationX"]
            #translationY=data["TranslationY"]
            #rotation=data["Rotation"]
            #dataValid=data["DataValid"]

            #if not dataValid or not circleFound:
            #    self.get_logger().error("[Client_node]Error in picture processing!")
            #    #this should trigger the PLC safety measures.
            #    #TODO:TEST!
            #    return 99999,99999,99999
            
            #return translationX,translationY,rotation

            #For testing purposes, we return a dummy offset.
            return 0.9, 0.0, 0.0  # x, y, theta

def main(args=None):
    rclpy.init(args=args)
    root=tk.Tk()
    client_node = Client_Node(root)

    '''
    Both ROS and Tkinker are liveservices that requires the complete control of the process to work.
    With this trick we embed the spinning of the ROS node inside the mainloop of Tkinker, forcing a
    pseudo-parallel execution of both.
    '''
    def update_ros():
        '''
        It spin once the ROS node and then schedule in Tkinker its next spin.
        '''
        rclpy.spin_once(client_node,timeout_sec=0.005)
        #client_node.get_logger().info("[Client_node]Spinning once...")
        root.after(1, update_ros)  # Schedule the next call 
    root.after(1,update_ros)

  

    def on_close():
        '''
        This function is called when the window is closed.
        It stops the ROS node and closes the window.'''


        client_node.get_logger().info("[Client_node] Shutting down...")
        client_node.destroy_node()
        rclpy.shutdown()
        root.destroy()  # closes the window and ends mainloop

    # Bind the close event to the on_close function
    root.protocol("WM_DELETE_WINDOW", on_close)  # handles window close
    root.mainloop()




#TODO: needed?
if __name__ == '__main__':
    main()
