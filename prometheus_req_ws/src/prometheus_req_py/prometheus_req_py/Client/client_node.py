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
from prometheus_req_interfaces.msg import EquipmentStatus
from prometheus_req_interfaces.action import CallFunctionBlock
from prometheus_req_py.ADS.utils import OkDialog,msg_type
import tkinter as tk
from tkinter import messagebox,simpledialog
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from functools import partial
from prometheus_req_py.Client.GUI import client_GUI

class Client_Node(Node):
    '''
    This node is designed to be used directly by a human operator. 
    It provides a simplified interface for monitoring and controlling the system.
    '''

    def call_block(self, name:str):
        '''
        This function is called when the building block buttons are pressed.
        It sends an async request to the service server.
        '''

        #Prevent to call a second function call while the previous one is still executing.
        if not self.functionBlockCalled:
            self.functionBlockCalled=True
            self.get_logger().info(f"[Client_node] Calling Block {name}")
            self.req.function_block_name=name
            #based on the function called, a different parameter is asked.
            match(name):
                case "loadTray":
                    answer = messagebox.askyesno("Load Tray", "Load(Yes) or Unload(No) the tray?")
                    self.req.bool_param1=answer
                case "positionerRotate":
                    answer = messagebox.askyesno("Positioner Rotate", "Rotate clockwise(Yes) or anticlockwise(No)?")
                    self.req.bool_param1=answer
                case "screwPickup":
                    screw = simpledialog.askinteger("Pick Up Screw", "Enter the screw number to pick up (1-6):", minvalue=1, maxvalue=6)
                    self.req.int_param1=screw

            #Ask the permission to run the function block.
            self.send_goal_future=self.client.send_goal_async(self.req,feedback_callback=self.goal_feedback_callback)
            #Tell where you are waiting for a response.
            self.send_goal_future.add_done_callback(self.goal_response_callback)
            self.get_logger().info(f"[Client_node] Done")
        else:
            self.get_logger().info(f"[Client_node] Function Block already called, waiting for response...")
            self.updateResponseText("A function Block has been already called, waiting for its response...", isResult=False)
             
    def goal_response_callback(self,future):
        '''
        This function is called when the action client receive a response.

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

    def goal_result_callback(self,future):
        '''
        This function is called when the action client receive a result.
        '''
        self.functionBlockResult=future.result().result.result
        self.functionBlockState=future.result().result.state
        self.functionBlockMsg=future.result().result.msg
        self.functionBlockCalled=False
  
        self.updateLabels()  #TODO: needed?
        self.updateResponseText(self.functionBlockMsg, isResult=True)
    
    def goal_feedback_callback(self,feedback):
        '''
        This function is called when the action client receive a feedback.
        '''
        msgType=feedback.feedback.msg_type
        self.functionBlockMsg=feedback.feedback.msg
        #Based on the msg type manage the feedback differently.
        #Refer to msg_type documentation for more informations about it
        match msgType:
            case msg_type.ERROR_CHECK:
                self.updateResponseText("Error check in progress...", isResult=False)
                _=OkDialog(self.root, title="Error Check", message=self.functionBlockMsg)
                self.errorCheckPub.publish(Empty())
            case msg_type.ASKING_PICTURE:
                self.updateResponseText("Asking a photo...", isResult=False)
                _=OkDialog(self.root, title="Take Picture", message="Press ok to take a picture!")
                self.imagePub.publish(Empty())
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
        self.subscription = self.create_subscription(
            EquipmentStatus,
            'state',
            self.state_update_callback,
            1)
        
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

        self.imagePub = self.create_publisher(
            Empty,
            'takePicture',
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
        self.subscription  # prevent unused variable warning

    def state_update_callback(self, msg):
        '''
        Called each time the plc's equipmentstate changes.
        It updates the local state variable and the labels of the GUI.
        '''
        #Testing code
        #self.get_logger().info("[Client_node]Receinving:"+str(msg.active_state_fsm_string))
        self.state=deepcopy(msg)
        self.updateLabels()



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
