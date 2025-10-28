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
from prometheus_req_interfaces.msg import EquipmentStatus, Offset,ScrewSlot,InputOutput
from prometheus_req_interfaces.action import CallFunctionBlock
from prometheus_req_py.ADS.utils import msgType,inputType,publishFeedback
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from prometheus_req_interfaces.srv import SetScrewBayState, RequestState
from rclpy.executors import MultiThreadedExecutor
import requests
import urllib3
import ast
import threading
import time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class API_node(Node):

    
    def __init__(self):
        super().__init__('api_node')

        # Define callback groups
        self.main_group = MutuallyExclusiveCallbackGroup() 
        self.input_group = ReentrantCallbackGroup()            
        self.action_group = ReentrantCallbackGroup()   
        self.state=EquipmentStatus()

        stateQos=QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.stateSub = self.create_subscription(
            EquipmentStatus,
            'state',
            self.state_update_callback,
            qos_profile=stateQos,
            callback_group=self.main_group)
        
        #it's essential that we do not lost the acks due to a networking failure
        #so we guarantee that samples are delivered, also trying multiple times.
        qos = QoSProfile(
            depth=3,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.stateRequestService= self.create_service(
            RequestState,
            'request_state',
            self.state_request_callback)

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
        inputQos=QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST
             )
        
        self.askInputPub = self.create_publisher(InputOutput,'askinput',qos_profile=inputQos)
        self.receiveInputSub= self.create_subscription(InputOutput,'receiveinput',self.receive_input_callback,qos_profile=inputQos,callback_group=self.input_group)
        self.inputReceived=False
        self.input=InputOutput()
        self.functionBlockCalled=False
        self.functionBlockDone=False
        self.functionBlockState="N/A"
        self.functionBlockMsg="N/A"
        self.functionBlockResult=False
        self.inSequence=False
        self.sequenceAborted=False
        self.goNext=False
        self.errorChecked=False

        self.functionBlockClient=ActionClient(self,CallFunctionBlock,"CallFunctionBlock",callback_group=self.action_group)
        while not self.functionBlockClient.wait_for_server(timeout_sec=1):
            self.get_logger().info('service not available, waiting again...')
            pass
        self.screwBayStateClient=self.create_client(SetScrewBayState,'setScrewBayState')
        while not self.screwBayStateClient.wait_for_service(timeout_sec=1):
            self.get_logger().info('service not available, waiting again...')
            pass
        self.screwBayReq=SetScrewBayState.Request()
        self.ActionReq=CallFunctionBlock.Goal()
        self.stateSub  # prevent unused variable warning

  
  
    def call_block(self, name:str,override=False) -> None:
        '''
        This function is called when the building block buttons are pressed.
        It sends an async request to the service server.
        :param name: The name of the function block to call.
        :param override: If True, it forces the call even if another function block is running.
        '''
        cancelAction=False
        #Prevent to call a second function call while the previous one is still executing.
        if override or not self.functionBlockCalled:
            self.functionBlockCalled=True
            self.get_logger().info(f"[client_API] Calling Block {name}")
            self.ActionReq.function_block_name=name
            inputMsg=InputOutput()
            #based on the function called, a different parameter is asked.
            match(name):
                case "loadTray":
                    self.get_logger().info("Asking for load tray...")
                    inputMsg.type=inputType.YES_NO
                    msg="Load Tray: Do you want to load(yes) or unload(no) a new tray? Yes/No/Cancel"
                    inputMsg.message=msg
                    self.askInputPub.publish(inputMsg)
                    while not self.inputReceived:
                        pass
                    self.inputReceived=False
                    self.get_logger().info(f"[client_API] Received input: {self.input.message} of type {self.input.type}")
                    match self.input.message.lower():
                        case "yes":
                            self.ActionReq.bool_param1=True
                        case "no":
                            self.ActionReq.bool_param1=False
                        case _:
                            cancelAction=True

                    
                case "positionerRotate":
                    self.get_logger().info("Asking for positioner rotation direction...")
                    inputMsg.type=inputType.YES_NO
                    msg="Positioner Rotate: Rotate clockwise(Yes) or anticlockwise(No)? Yes/No/Cancel"
                    inputMsg.message=msg
                    self.askInputPub.publish(inputMsg)
                    while not self.inputReceived:
                        pass
                    self.inputReceived=False
                    match self.input.message.lower():
                        case "yes":
                            self.ActionReq.bool_param1=True
                        case "no":
                            self.ActionReq.bool_param1=False
                        case _:
                            cancelAction=True
                case "stackTray":
                    self.get_logger().info("Asking for stack tray number...")
                    inputMsg.type=inputType.INTEGER
                    msg="Stack Tray: Enter the tray number to stack (1-4) or Cancel"
                    inputMsg.message=msg
                    self.askInputPub.publish(inputMsg)
                    while not self.inputReceived:
                        pass
                    self.inputReceived=False
                    try:
                        trayNumber=int(self.input.message)
                        if(trayNumber<1 or trayNumber>4):
                            raise ValueError("Tray number out of range")
                        self.ActionReq.int_param1=trayNumber
                    
                    except ValueError:
                        cancelAction=True




                case "gyroGrpRot":
                    self.get_logger().info("Asking for gyro gripper rotation direction...")
                    inputMsg.type=inputType.INTEGER
                    msg="Gyro Gripper Rotate: Enter the direction to rotate (1-2) or Cancel"
                    inputMsg.message=msg
                    self.askInputPub.publish(inputMsg)
                    while not self.inputReceived:
                        pass
                    self.inputReceived=False
                    try:
                        direction=int(self.input.message)
                        if(direction<1 or direction>2):
                            raise ValueError("Direction out of range")
                        self.ActionReq.int_param1=direction
                    except ValueError:
                        cancelAction=True

                case "screwPickup":
                    self.get_logger().info("Asking for screw number to pick up...")
                    inputMsg.type=inputType.INTEGER
                    msg="Pick Up Screw: Enter the screw number to pick up (1-6) or Cancel"
                    inputMsg.message=msg
                    self.askInputPub.publish(inputMsg)
                    while not self.inputReceived:
                        pass
                    self.inputReceived=False
                    try:
                        screw=int(self.input.message)
                        if(screw<1 or screw>6):
                            raise ValueError("Screw number out of range")
                        self.ActionReq.int_param1=screw
                    except ValueError:
                        cancelAction=True

                case "present2Op":
                    self.get_logger().info("Asking for side and face to present...")
                    inputMsg.type=inputType.INTEGER
                    msg="Present to Operator: Enter the side to present (1-2) or Cancel"
                    inputMsg.message=msg
                    self.askInputPub.publish(inputMsg)
                    while not self.inputReceived:
                        pass
                    self.inputReceived=False
                    try:
                        side=int(self.input.message)
                        if(side<1 or side>2):
                            raise ValueError("Side out of range")
                        self.ActionReq.int_param1=side
                    except ValueError:
                        cancelAction=True
                    if(not cancelAction): 
                        inputMsg.type=inputType.INTEGER
                        msg="Present to Operator: Enter the face to present (1-4) or Cancel"
                        inputMsg.message=msg
                        self.askInputPub.publish(inputMsg)
                        while not self.inputReceived:
                            pass
                        self.inputReceived=False
                        try:
                            face=int(self.input.message)
                            if(face<1 or face>4):
                                raise ValueError("Face out of range")
                            self.ActionReq.int_param2=face
                        except ValueError:
                            cancelAction=True
   
                #TODO: the Or create a crash. Investigate.
                case "presentToScrew":
                    inputMsg.type=inputType.INTEGER
                    msg="Present to Operator: Enter the side to present (1-2) or Cancel"
                    inputMsg.message=msg
                    self.askInputPub.publish(inputMsg)
                    while not self.inputReceived:
                        pass
                    self.inputReceived=False
                    try:
                        side=int(self.input.message)
                        if(side<1 or side>2):
                            raise ValueError("Side out of range")
                        self.ActionReq.int_param1=side
                    except ValueError:
                        cancelAction=True
                    if(not cancelAction): 
                        inputMsg.type=inputType.INTEGER
                        msg="Present to Operator: Enter the face to present (1-4) or Cancel"
                        inputMsg.message=msg
                        self.askInputPub.publish(inputMsg)
                        while not self.inputReceived:
                            pass
                        self.inputReceived=False
                        try:
                            face=int(self.input.message)
                            if(face<1 or face>4):
                                raise ValueError("Face out of range")
                            self.ActionReq.int_param2=face
                        except ValueError:
                            cancelAction=True
                

                case "screwTight":
                    inputMsg.type=inputType.FLOAT
                    msg="Insert the X offset for the screw (in mm) or Cancel"
                    inputMsg.message=msg
                    self.askInputPub.publish(inputMsg)
                    while not self.inputReceived:
                        pass
                    self.inputReceived=False
                    try:
                        screwX=float(self.input.message)
                    except ValueError:
                        cancelAction=True
                    if(not cancelAction):
                        inputMsg.type=inputType.FLOAT
                        msg="Insert the Y offset for the screw (in mm) or Cancel"
                        inputMsg.message=msg
                        self.askInputPub.publish(inputMsg)
                        while not self.inputReceived:
                            pass
                        self.inputReceived=False
                        try:
                            screwY=float(self.input.message)
                        except ValueError:
                            cancelAction=True
                    if(not cancelAction):
                        inputMsg.type=inputType.FLOAT
                        msg="Insert the Z offset for the screw (in mm) or Cancel"
                        inputMsg.message=msg
                        self.askInputPub.publish(inputMsg)
                        while not self.inputReceived:
                            pass
                        self.inputReceived=False
                        try:
                            screwZ=float(self.input.message)
                        except ValueError:
                            cancelAction=True
                    if(not cancelAction):
                        inputMsg.type=inputType.INTEGER
                        msg="Insert the target to use (1-4) or Cancel"
                        inputMsg.message=msg
                        self.askInputPub.publish(inputMsg)
                        while not self.inputReceived:
                            pass
                        self.inputReceived=False
                        try:
                            targetToUse=int(self.input.message)
                            if(targetToUse<1 or targetToUse>4):
                                raise ValueError("Target out of range")
                        except ValueError:
                            cancelAction=True
                    if(not cancelAction):
                        inputMsg.type=inputType.INTEGER
                        msg="Insert the focal plane to use (0-2) or Cancel"
                        inputMsg.message=msg
                        self.askInputPub.publish(inputMsg)
                        while not self.inputReceived:
                            pass
                        self.inputReceived=False
                        try:
                            focalPlane=int(self.input.message)
                            if(focalPlane<0 or focalPlane>2):
                                raise ValueError("Focal plane out of range")
                        except ValueError:
                            cancelAction=True
                    if(not cancelAction):
                        inputMsg.type=inputType.INTEGER
                        msg="Insert the screw recipe ID to use (1-5) or Cancel"
                        inputMsg.message=msg
                        self.askInputPub.publish(inputMsg)
                        while not self.inputReceived:
                            pass
                        self.inputReceived=False
                        try:
                            screwRecipeID=int(self.input.message)
                            if(screwRecipeID<1 or screwRecipeID>5):
                                raise ValueError("Screw recipe ID out of range")
                        except ValueError:
                            cancelAction=True
                    if(not cancelAction):
                        inputMsg.type=inputType.YES_NO
                        msg="Screw Area: Tighten the screw inside the area (Yes) or outside the area (No)? Yes/No/Cancel"
                        inputMsg.message=msg
                        self.askInputPub.publish(inputMsg)
                        while not self.inputReceived:
                            pass
                        self.inputReceived=False
                        match self.input.message:
                            case "Yes":
                                screwArea=True
                            case "No":
                                screwArea=False
                            case _:
                                cancelAction=True
                        if(not cancelAction):
                            self.ActionReq.float_param1=screwX
                            self.ActionReq.float_param2=screwY
                            self.ActionReq.float_param3=screwZ
                            self.ActionReq.int_param1=targetToUse
                            self.ActionReq.int_param2=focalPlane
                            self.ActionReq.int_param3=screwRecipeID
                            self.ActionReq.bool_param1=screwArea


                case "setScrewBayState":
                    #TODO: Implement the screw bay editor.
                    '''
                    self.screwBayReq.bay_number=6
                    response=ScrewBayDialog(self.root,screw_bays=self.state.screw_bay,num_slots=self.screwBayReq.bay_number,title="Screw Bay Editor")
                    if(response.result is None):
                        pass
                    else:
                        screwBay = []
                        for slot_dict in response.result:
                            if slot_dict is None:
                                continue  # skip invalid entries
                            slot_msg = ScrewSlot()
                            slot_msg.max_idx_x = slot_dict["MAX_IDX_X"]
                            slot_msg.max_idx_y = slot_dict["MAX_IDX_Y"]
                            slot_msg.next_idx_x  = slot_dict["nextIdxX"]
                            slot_msg.next_idx_y  = slot_dict["nextIdxY"]
                            screwBay.append(slot_msg)
                        self.screwBayReq.screw_bays = screwBay
                        self.get_logger().info(f"[client_API] Calling setScrewBayState for {self.screwBayReq.bay_number} bays with state: {self.screwBayReq.screw_bays}")
                        self.screwBayStateClient.call_async(self.screwBayReq)
                    self.functionBlockCalled=False
                    '''
                    return 
                                       
            if(cancelAction):
                self.functionBlockCalled=False
                self.get_logger().info(f"[client_API] Action cancelled by the user.")
                self.sequenceAborted=True
                return
            #Ask the permission to run the function block.
            self.send_goal_future=self.functionBlockClient.send_goal_async(self.ActionReq,feedback_callback=self.goal_feedback_callback)
            #Tell where you are waiting for a response.
            self.send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info(f"[client_API] Function Block already called, waiting for response...")

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
            self.sequenceAborted=True
            self.functionBlockCalled=False
            self.goNext=True
            return
        
        self.get_logger().info(f"[CLIENT NODE] Action response: Accepted")
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
        self.sequenceAborted= not self.functionBlockResult
        self.functionBlockCalled=False
        self.goNext=True
        self.get_logger().info(f"[CLIENT NODE] Action result: State:{self.functionBlockState}, Success:{self.functionBlockResult}, Msg:{self.functionBlockMsg}")


    def goal_feedback_callback(self,feedbackMsg: CallFunctionBlock.Feedback) -> None:
        '''
        This function is called when the action client receive a feedback.
        :feedback: The feedback received from the action server.
        '''
        feedbackMsgType=feedbackMsg.feedback.msg_type
        self.functionBlockMsg=feedbackMsg.feedback.msg
        #Based on the msg type manage the feedback differently.
        #Refer to msgType documentation for more informations about it
        match feedbackMsgType:
            case msgType.ERROR_CHECK:
                message=f"[client_API] Error check required: {self.functionBlockMsg}"
                self.get_logger().info(message)

                inputMsg=InputOutput()
                inputMsg.type=inputType.ERROR_CHECK
                inputMsg.message=self.functionBlockMsg
                self.askInputPub.publish(inputMsg)
                self.lastTime=time.time()
                while not self.inputReceived and self.input.type!=inputType.ERROR_CHECK:
                    if(time.time()-self.lastTime>1):
                        self.lastTime=time.time()
                        self.get_logger().info(f"Waiting for error check input...{self.inputReceived} {self.input.type} {inputType.ERROR_CHECK}")
                    pass
                self.inputReceived=False

                
                self.errorCheckPub.publish(Empty())
                if(self.inSequence):
                    self.get_logger().info("[client_API] Error check in sequence, aborting sequence!")
                    self.errorChecked=True
            case msgType.ASK_PICTURE_SCREW:
                self.get_logger().info("Asking a photo for screw...")
                inputMsg=InputOutput()
                inputMsg.type=inputType.OK
                inputMsg.message="Picture needed for screw detection! (OK)"
                self.askInputPub.publish(inputMsg)
                while not self.inputReceived and self.input.type!=inputType.OK:
                    pass
                self.inputReceived=False


                inputMsg.type=inputType.INTEGER
                msg="Select Focal Plane: Enter the focal plane to use (0-2)"
                inputMsg.message=msg
                self.askInputPub.publish(inputMsg)
                while not self.inputReceived:
                    pass
                self.inputReceived=False
                try:
                    focalPlane=int(self.input.message)
                    if(focalPlane<0 or focalPlane>2):
                        raise ValueError("Focal plane out of range")
                except ValueError:
                    focalPlane=0
                    self.get_logger().info("Invalid focal plane, using 0")

                msg="Select ROI ID: Enter the ROI ID to use (0-3)"
                inputMsg.message=msg
                self.askInputPub.publish(inputMsg)
                while not self.inputReceived:
                    pass
                self.inputReceived=False
                try:
                    roiID=int(self.input.message)
                    if(roiID<0 or roiID>3):
                        raise ValueError("ROI ID out of range")
                except ValueError:
                    roiID=0
                    self.get_logger().info("Invalid ROI ID, using 0")
                inputMsg.type=inputType.YES_NO
                msg="Find Screw: Find the screw in the image? Yes/No"
                inputMsg.message=msg
                self.askInputPub.publish(inputMsg)
                while not self.inputReceived and self.input.type!=inputType.YES_NO:
                    pass
                self.inputReceived=False
                findScrew = inputMsg.message=="Yes"

                self.get_logger().info(f"[client_API] Received picture request with focalPlane:{focalPlane}, roiID:{roiID}, findScrew:{findScrew}")
                self.sendOffsetData(feedbackMsgType,focalPlane,roiID,findScrew)
            case msgType.ASK_PICTURE_VCHECK:
                self.get_logger().info("Asking a photo for vision check...")
                inputMsg=InputOutput()
                inputMsg.type=inputType.OK
                inputMsg.message="Picture needed for vision check! (OK)"
                self.askInputPub.publish(inputMsg)
                while not self.inputReceived and self.input.type!=inputType.OK:
                    pass
                self.inputReceived=False
                self.sendOffsetData(feedbackMsgType)
            case _:
                self.get_logger().info(f"[client_API] Feedback: {self.functionBlockMsg}")


    def receive_input_callback(self, msg: InputOutput) -> None:
        '''
        This function is called when an input is received on the /receiveinput topic.
        :param msg: The input received.
        '''
        self.get_logger().info(f"[client_API] Received input: {self.input.message} of type {self.input.type}")
        if msg.type==inputType.CALLBLOCK:
            thread= threading.Thread(target=lambda: self.call_block(msg.message))
            thread.start()
            #self.call_block(msg.message)
            return

        self.input=deepcopy(msg)
        self.inputReceived=True
        self.get_logger().info(f"[client_API] Received input: {self.input.message} of type {self.input.type}")



    def sendOffsetData(self,msgType:CallFunctionBlock.Feedback.msg_type,focalPlane:int=0, roiID:int=0, findScrew:bool=False) -> None:
        '''
        Call the function to calculate the picture offset and publish it.
        :param msgType: The type of message to send.
        :param focalPlane: The focal plane to use for the picture.
        :param roiID: The ROI ID to use for the picture.
        :param findScrew: Whether to find the screw in the picture.
        '''
        offset=self.calculate_picture_offset(msgType,focalPlane,roiID,findScrew)
        offsetMsg=Offset()
        offsetMsg.data_valid=offset[0]
        offsetMsg.x=offset[1]
        offsetMsg.y=offset[2]
        offsetMsg.theta=offset[3]
        self.offsetPub.publish(offsetMsg)
        self.get_logger().info(f"[client_API] Published offset: {offsetMsg}")


    def state_update_callback(self, msg: EquipmentStatus) -> None:
        '''
        Called each time the plc's equipmentstate changes.
        It updates the local state variable and the labels of the GUI.
        :param msg: The new state of the equipment.
        '''
        #Testing code
        #self.get_logger().info("[client_API]Receiving:"+str(msg.active_state_fsm_string))
        if(msg is None):
            self.get_logger().error("[client_API]Received None msg!")
            return
        self.state=deepcopy(msg)

    def state_request_callback(self, request: RequestState, response: RequestState.Response) -> RequestState.Response:
        '''
        Handle a state request.
        '''
        response.state=deepcopy(self.state)
        return response

    def calculate_picture_offset(self,askedAction:msgType,calibrationPlane:int=0,roiId:int=0,findScrew:bool=False) -> tuple[bool,float,float,float]:
            """
            Calculate the offset of the picture.
            :param askedAction: The type of action to perform (ASK_PICTURE_SCREW or ASK_PICTURE_VCHECK).
            :param calibrationPlane: The calibration plane to use (only for ASK_PICTURE_SCREW).
            :param roiId: The ROI ID to use (only for ASK_PICTURE_SCREW).
            :param findScrew: Whether to find the screw in the picture (only for ASK_PICTURE_SCREW).
            :return: A tuple containing a boolean indicating if the data is valid, and the x, y, theta offsets.
            """
            ATS_IP = '10.10.10.100'
            self.get_logger().info(f"[client_API]Requesting picture offset from ATS at {ATS_IP}")
            parameters={"calibrationPlane":calibrationPlane,"roiId":roiId,"findScrew":findScrew}
            if(askedAction==msgType.ASK_PICTURE_SCREW):
                Command="GetScrewCorrection"
            else:
                Command="GetTrayCorrection"


            urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
            try:
                response = requests.get(f'https://{ATS_IP}/{Command}',params=parameters,verify=False).json()
            except Exception as e:
                self.get_logger().error(f"Error occurred: {e}")
                return False,0.0,0.0,0.0
            self.get_logger().info(f"[client_API]DATA IS: {response}")

            if(type(response) is str):
                response=ast.literal_eval(response.replace('false',"False").replace('true','True'))

            dataValid=response["DataValid"]
            if(dataValid is False):
                self.get_logger().error("[client_API]Error in picture processing!")
                return False,0.0,0.0,0.0
            #Not used for now
            #circleFound=response["CircleFound"]
            #if(circleFound is False):
            #    self.get_logger().error("[client_API]Error in picture processing: Circle not found!")
            #    return False,0.0,0.0,0.0
            translationX=response["TranslationX"]
            translationY=response["TranslationY"]
            rotation=response["Rotation"]

            self.get_logger().info(f"[client_API]Picture offset response: {response}")


            return dataValid,translationX, translationY, rotation


def main(args=None):
    rclpy.init(args=args)
    api_node =API_node()
    executor = MultiThreadedExecutor()
    executor.add_node(api_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    api_node.destroy_node()
    rclpy.shutdown()




#TODO: needed?
if __name__ == '__main__':
    main()
