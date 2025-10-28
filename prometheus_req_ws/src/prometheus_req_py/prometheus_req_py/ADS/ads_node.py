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
from getpass import getpass
import pyads # pyright: ignore[reportMissingImports]
import time
import ctypes
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from copy import deepcopy
from prometheus_req_interfaces.msg import EquipmentStatus,ScrewSlot,Offset
from prometheus_req_interfaces.srv import SetScrewBayState
from prometheus_req_interfaces.action import CallFunctionBlock
from prometheus_req_py.ADS.structures import EquipmentStatus_ctype,ScrewSlot_ctype
from prometheus_req_py.ADS.utils import reqState,getReqStateMsg,msgType,publishFeedback
from std_msgs.msg import Empty
from rclpy.action import ActionServer as rclpyActionServer
from prometheus_req_py.ADS.FunctionBlocks import *
import prometheus_req_py.ADS.checks as checks
from functools import partial
import threading

class ADS_Node(Node):
    """
    A pyADS node is responsible for establishing and mantaining the connection with the PLC using the pyADS library.
    """

    def __init__(self):
        '''
        Initialize the ADS Node.
        '''
        super().__init__('ads_node')
        self.init_s_time=time.time_ns()
        #always drop old msg in case of a slowdonw. Keep the newest.
        stateQos=QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.statePub = self.create_publisher(EquipmentStatus, 'state',qos_profile=stateQos)
        self.actionServer= rclpyActionServer(self,CallFunctionBlock,
                                             "CallFunctionBlock",
                                             self.block_execute_callback,)
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        # Create a subscription to the error check action
        self.ADSErrorCheckSub= self.create_subscription(
            Empty,
            'errorCheckAck',
            self.error_check_callback,
            qos_profile=qos,
        )
        # Create a subscription to the ask picture action
        self.ADSOffsetSub= self.create_subscription(
            Offset,
            'offset',
            self.askPicture_callback,
            qos_profile=qos,
        )

        self.setScrewBayStateService= self.create_service(
            SetScrewBayState,
            'setScrewBayState',
            self.setScrewBayState_callback
        )


        self.picture= None
        self.errorCheckSem = threading.Semaphore(0)
        self.askPictureSem = threading.Semaphore(0)
        self.askPictureEvent = False
        #timerPeriod = 1  # seconds
        #self.timer = self.create_timer(timerPeriod, self.timer_callback)
        self.lastTime=time.time()
        self.actionTimerDelay=3 #seconds
        self.lastStatus=None

        
        #Initialize the function blocks management methods. Treat them as methods of the ADS_Node class.
        self.manageScrew = partial(screwTemplate.manageScrew, self)
        self.managePositionerRotate = partial(positionerRotate.managePositionerRotate, self)
        self.manageLoadTray = partial(loadTray.manageLoadTray, self)
        self.manageDepositTray = partial(depositTray.manageDepositTray, self)
        self.manageMrTrolleyVCheck = partial(mrTrolleyVCheck.manageMrTrolleyVCheck, self)
        self.manageMrTrolleyVCheckErrorCheck = partial(mrTrolleyVCheck.manageMrTrolleyVCheckErrorCheck, self)
        self.managePresent = partial(presentTemplate.managePresent, self)
        self.publishFeedback= partial(publishFeedback, self)
        self.manageStackTray = partial(stackTray.manageStackTray, self)
        self.manageGyroGrpRotate = partial(gyroGrpRotate.manageGyroGrpRotate, self)
        
        

        # Initialize the checks methods. Treat them as methods of the ADS_Node class.
        self.checkPositionerRotate = partial(checks.checkPositionerRotate, self)
        self.checkLoadTray = partial(checks.checkLoadTray, self)
        self.checkScrewPickup = partial(checks.checkScrewPickup, self)

        #Get the parameters from the config file
        self.declare_parameter("CLIENT_NETID","None")
        self.declare_parameter("PLC_IP","None")
        self.declare_parameter("PLC_NET_ID","None")
        self.CLIENT_NETID = self.get_parameter('CLIENT_NETID').value
        PLC_IP= self.get_parameter('PLC_IP').value
        PLC_NET_ID = self.get_parameter('PLC_NET_ID').value
        
        pyads.open_port()
        pyads.set_local_address(self.CLIENT_NETID)
        
        #change based on the credential you are connecting to. To run only the first time.
        if(False):
            self.declare_parameter("CLIENT_IP","None")
            CLIENT_IP = self.get_parameter('CLIENT_IP').value
            pyads.add_route_to_plc(self.CLIENT_NETID,CLIENT_IP,PLC_IP,"Administrator","1",route_name="pyADS")
        pyads.close_port()

        self.plc= pyads.Connection(PLC_NET_ID, pyads.PORT_TC3PLC1, PLC_IP)
        self.plc.open()
        
        #Set a notification on the equipment status to publish it on a topic.
        statusMemory=pyads.NotificationAttrib(ctypes.sizeof(EquipmentStatus_ctype))#ctypes.sizeof(EquipmentStatus_ctype)
        self.plc.add_device_notification("GVL_ATS.equipmentState",statusMemory,self.status_callback)
        #self.plc.write_by_name("GVL_ATS.requests.loadTray.errorAck",True,pyads.PLCTYPE_BOOL)
        self.first_update()

        
    def block_execute_callback(self,goalHandler) -> CallFunctionBlock.Result:
        '''
        Callback for the action server. This function is called when a client sends a request to the action server.
        First it checks if the block is ready to be executed, then it runs the checks and manages the parameters.
        Later it sends the request to the PLC and waits for the state change from executing.
        Finally it manages the function block individual behaviour and returns the result.

        :param goalHandler: The goal handler to manage the request.
        :return: The result of the action.
        '''
        self.get_logger().info(f"[DEBUG]block_execute_callback")

        functionBlockName=goalHandler.request.function_block_name

        '''
        allowedFunctionBlocks=["positionerRotate","loadTray","mrTrolleyVCheck","screwPickup",
                               "screwTight","depositTray","pickUpTray","present2Op","presentToScrew",
                               "gyroGrpRot","stackTray"]
        if(functionBlockName    not in allowedFunctionBlocks):
            self.get_logger().info(f"[ADS_Node] Function Block {goalHandler.request.function_block_name} not allowed! Allowed function blocks are: {allowedFunctionBlocks}")
            goalHandler.abort()
            result=CallFunctionBlock.Result()
            result.success=False
            result.msg=f"Function Block {goalHandler.request.function_block_name} not allowed! Allowed function blocks are: {allowedFunctionBlocks}"
            result.state=999
            return result
        '''
        

        result=CallFunctionBlock.Result()

        actualState=self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT)
        msgState=getReqStateMsg(actualState)
        self.get_logger().info(f"[ADS_Node]State: {actualState}:{msgState}")
        self.publishFeedback(goalHandler, getReqStateMsg(actualState),0)
        if(actualState == reqState.ST_READY):
            check,msg=self.runChecks(functionBlockName)
            #if the preconditions are not met, the goal is aborted.
            if(not check):
                msgFeed=CallFunctionBlock.Feedback()
                msgFeed.msg_type=msgType.ERROR_CHECK
                msgFeed.msg=msg
                goalHandler.publish_feedback(msgFeed)
                goalHandler.abort()
                result.success=False
                result.msg=msg
                result.state=actualState
                self.get_logger().info(f"[ADS_Node] Goal aborted due to checks failure. -> {msg}")
                return result
            
            #If the checks are passed, we manage the parameters of the function block.
            self.manageParameters(goalHandler.request,functionBlockName)
            self.get_logger().info(f"[ADS_Node] Parameters managed for {functionBlockName}")
            #Send the request to the PLC
            self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.request",1,pyads.PLCTYPE_BOOL)
            counter=0
            while(self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT) == reqState.ST_READY):
                if(time.time()-self.lastTime>self.actionTimerDelay):
                    counter += 1
                    self.lastTime=time.time()
                    feedback_msg = CallFunctionBlock.Feedback()
                    feedback_msg.msg="Waiting for the function block state to start..."
                    goalHandler.publish_feedback(feedback_msg)
                    self.get_logger().info(f"[ADS_Node] Waiting for the function block state to start... ({counter} try of 3)")
                    if counter==3:
                        break
                pass
            if counter==3:
                self.get_logger().info(f"[ADS_Node] Function Block {functionBlockName} did not start, aborting goal.")
                goalHandler.abort()
                result.success=False
                result.msg=f"Function Block {functionBlockName} did not start, aborting goal."
                result.state=self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT)
                return result
            
            #Wait the completion of the task
            while(self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT) in (reqState.ST_EXECUTING,
                                                                                                             reqState.ST_EXECUTING_2,
                                                                                                             reqState.ST_EXECUTING_3,
                                                                                                             reqState.ST_EXECUTING_4)):
                self.publishFeedback(goalHandler, f"Function Block {functionBlockName} is executing...", self.actionTimerDelay)
            #self.get_logger().info(f"[DEBUG]Function Block {functionBlockName} executed, waiting for the result...")
            

            actualState=self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT)
            self.publishFeedback(goalHandler, f"Handling the function Block {functionBlockName}",0)
            self.get_logger().info(f"[ADS_Node]: Managing function block {functionBlockName} with state {actualState} and result {result.success}")
            match (functionBlockName):
                case "positionerRotate":
                        result.msg,result.state=self.managePositionerRotate(goalHandler)
                case "loadTray":
                        result.msg,result.state=self.manageLoadTray(goalHandler)
                case "depositTray":
                        result.msg,result.state=self.manageDepositTray(goalHandler)
                case "mrTrolleyVCheck":
                        result.msg,result.state=self.manageMrTrolleyVCheck(goalHandler)
                case "screwPickup":
                        result.msg,result.state=self.manageScrew(goalHandler)
                case "screwTight":
                        result.msg,result.state=self.manageScrew(goalHandler)
                case "present2Op":
                        result.msg,result.state=self.managePresent(goalHandler)
                case "presentToScrew":
                        result.msg,result.state=self.managePresent(goalHandler)
                case "gyroGrpRot":
                        result.msg,result.state=self.manageGyroGrpRotate(goalHandler)
                case "stackTray":
                        result.msg,result.state=self.manageStackTray(goalHandler)

            goalHandler.succeed()
            result.success=self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.Done",pyads.PLCTYPE_BOOL)
            self.get_logger().info(f"[DEBUG] Goal Finished {result.success} with message: {result.msg} and state: {result.state}.| Types: {type(result.success)}, {type(result.msg)}, {type(result.state)}")
            return result
        else:
            self.get_logger().info(f"[DEBUG]Not Ready")
            goalHandler.abort()
            result.success=False
            result.msg="Machine not ready! Please wait for the machine to be ready"
            result.state=actualState
            return result
    

    def manageParameters(self,req,functionBlockName:str)-> None:
        '''
        Manage the parameters of the function block.
        :param req: The request containing the parameters.
        :param functionBlockName: The name of the function block.
        '''
        match functionBlockName:
            case "positionerRotate":
                self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.rotateClockwise",req.bool_param1,pyads.PLCTYPE_BOOL)
            case "loadTray":
                self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.reqToLoad",req.bool_param1,pyads.PLCTYPE_BOOL)
            case "screwPickup":
                #screwType
                self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.screwType",req.int_param1,pyads.PLCTYPE_INT)
            case "screwTight":
                self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.screwX",req.float_param1,pyads.PLCTYPE_REAL)
                self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.screwY",req.float_param2,pyads.PLCTYPE_REAL)
                self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.screwZ",req.float_param3,pyads.PLCTYPE_REAL)
                self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.screwArea",2 if req.bool_param1 else 1,pyads.PLCTYPE_INT)
                self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.target2Use",req.int_param1,pyads.PLCTYPE_INT)
                self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.focalPlane2Use",req.int_param2,pyads.PLCTYPE_INT)
                self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.screwRecipeID",req.int_param3,pyads.PLCTYPE_BYTE)
            case "present2Op" | "presentToScrew":
                self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.sideToScrew",req.int_param1,pyads.PLCTYPE_INT)
                self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.faceToScrew",req.int_param2,pyads.PLCTYPE_INT)
            case "gyroGrpRot":
                self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.sideToRotate",req.int_param1,pyads.PLCTYPE_INT)
            case "stackTray":
                self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.stackLevel",req.int_param1,pyads.PLCTYPE_INT)
            case _:
                  pass
            


    def runChecks(self,functionBlockName:str) -> tuple[bool,str]:
        '''
        Check for the preconditions before executing a function block.
        :param functionBlockName: The name of the function block to check.
        :return: A tuple containing a boolean indicating if the checks passed and, in case of failure, a message explaining it.
        '''

        match (functionBlockName):
            case "positionerRotate":
                return self.checkPositionerRotate()
            case "loadTray":
                return self.checkLoadTray()
            case "screwPickup":
                return self.checkScrewPickup()
            case _:
                return (True,None) #No checks for other function blocks, return True and None message.
    
    
    def error_check_callback(self, _):
        """
        Callback for the error check action.
        This function is called when the client aknowledge the error check.
        """
        self.get_logger().info("[ADS]ERROR CHECK CALLBACK!")
        self.errorCheckSem.release()


    def error_check(self,errMsg:str,goalHandler) -> None:
        '''
        Handle the error check action.
        :param errMsg: The error message to send to the client.
        :param goalHandler: The goal handler to publish the feedback.
        '''
        msgFeed=CallFunctionBlock.Feedback()
        msgFeed.msg_type=msgType.ERROR_CHECK
        msgFeed.msg=errMsg
        goalHandler.publish_feedback(msgFeed)

        self.errorCheckSem.acquire()
        self.get_logger().info("[Debug]Error check acknowledged by the client.")



    def askPicture(self,goalHandler,msg):
        '''
        Handle the ask picture action.
        '''
        msg_feed=CallFunctionBlock.Feedback()
        if(goalHandler.request.function_block_name=="screwPickup" or goalHandler.request.function_block_name=="screwTight"):
            msg_feed.msg_type=msgType.ASK_PICTURE_SCREW
        else:
            msg_feed.msg_type=msgType.ASK_PICTURE_VCHECK

        msg_feed.msg=msg
        goalHandler.publish_feedback(msg_feed)
        self.get_logger().info("[Debug]Waiting for the picture...")
        self.askPictureSem.acquire()
        #we don't need data_valid for now
        return self.offset[1:]


    def askPicture_callback(self, offset: Offset):
        """
        Callback for the ask picture action.
        This function is called when the client sends a picture.
        """

        self.get_logger().info("[ADS]ASK PICTURE CALLBACK!")
        self.offset=(offset.data_valid,offset.x, offset.y, offset.theta)

        self.askPictureSem.release()
        
    def setScrewBayState_callback(self,request,response):
        '''
        Callback for the setScrewBayState service.
        This function is called when a client sends a request to the service.
        It sets the screw bay state in the PLC.
        :param request: The request containing the parameters.
        :param response: The response to send back to the client.
        :return: The response with the success status.
        '''
        bayNumber=request.bay_number
        screwBays=request.screw_bays
        if bayNumber<1:
            self.get_logger().info(f"[ADS_Node] Invalid bay number {bayNumber}. Must be positive.")
            response.success=False
            return response
        if len(screwBays)!=bayNumber:
            self.get_logger().info(f"[ADS_Node] Invalid number of screw bays {len(screwBays)}. Must be equal to bay number {bayNumber}.")
            response.success=False
            return response
        screwBays_ctype=[]
        for i in range(bayNumber):
            slot_ctype = ScrewSlot_ctype()
            slot_ctype.MAX_IDX_X = screwBays[i].max_idx_x
            slot_ctype.MAX_IDX_Y = screwBays[i].max_idx_y
            slot_ctype.nextIdxX  = screwBays[i].next_idx_x
            slot_ctype.nextIdxY  = screwBays[i].next_idx_y
            screwBays_ctype.append(slot_ctype)

            self.plc.write_by_name(f"GVL_ATS.equipmentState.screwBay[{i+1}]",screwBays_ctype[i],ScrewSlot_ctype)
        response.success=True
        return response
   

    def cpy_to_equipment_status_msg(self,src:EquipmentStatus_ctype) -> EquipmentStatus:
        """
        Copy the values from a EquipmentStatus_ctype to a EquipmentStatus message.
        :param src: The source EquipmentStatus_ctype.
        :return: A EquipmentStatus message with the copied values.
        """
        dst=EquipmentStatus()
        dst.em_general =  src.emGeneral
        dst.em_mr = src.emMR
        dst.em_sr = src.emSR
        dst.tem_sens_ok = src.temSensOk
        dst.air_press_ok = src.airPressOk
        dst.em_laser_scanner = src.emLaserScanner
        dst.em_laser_scanner2 = src.emLaserScanner2 
        dst.em_laser_scanner3 = src.emLaserScanner3 
        dst.automatic_mode = src.automaticMode
        dst.mgse_to_conveyor = src.mgseToConveyor
        dst.trolley_in_bay = src.trolleyInBay
        dst.side_2_robot = src.side2Robot
        dst.positioner_is_up = src.positionerIsUp
        dst.positioner_is_down = src.positionerIsDown
        dst.pallet_is_in_wrk_pos = src.palletIsInWrkPos
        dst.pallet_is_in_entry_pos = src.palletIsInEntryPos
        dst.rotary_aligned = src.rotaryAligned
        dst.holder_correction_done = src.holderCorrectionDone
        dst.screw_bay = []
        for i in range(6):
            src_slot = src.screwBay[i]
            slot = ScrewSlot()
            slot.max_idx_x = src_slot.MAX_IDX_X
            slot.max_idx_y = src_slot.MAX_IDX_Y
            slot.next_idx_x = src_slot.nextIdxX
            slot.next_idx_y = src_slot.nextIdxY
            dst.screw_bay.append(slot)
        dst.active_state_fsm_string = str(src.activeStateFSMString.data)
        dst.active_state_mr_fsm_string = str(src.activeStateMRFSMString.data)
        dst.active_state_sr_fsm_string = str(src.activeStateSRFSMString.data)
        dst.active_state_conveyor_string = str(src.activeStateConveyorString.data)
        dst.active_state_system_safety_test = str(src.activeStateSystemSafetyTest.data)
        return dst

    def status_callback(self,notification,_): #_=data
        '''
        This function is called each time equipementStatus on the plc changes.
        It parses the notification and publishes the status update on the state topic.
        :param notification: The notification received from the PLC.
        :param _: Unused paramenter.
        '''
        #WARNING: if you change the type of the notification, you have also to update it in the statusMemory init!
        _,_,value=self.plc.parse_notification(notification,EquipmentStatus_ctype)
        statusUpdate=self.cpy_to_equipment_status_msg(value)
        self.statePub.publish(statusUpdate)
        self.lastStatus=deepcopy(statusUpdate)

    def first_update(self):
         status=self.plc.read_by_name('GVL_ATS.equipmentState',EquipmentStatus_ctype)
         statusUpdate=self.cpy_to_equipment_status_msg(status)
         self.statePub.publish(statusUpdate)
         self.get_logger().info("[ADS_Node]First status update published.")

    def timer_callback(self):
        '''
        This function is called periodically. It publishes a status update on the state topic.
        '''
        if(self.lastStatus is None):
            status=self.plc.read_by_name('GVL_ATS.equipmentState',EquipmentStatus_ctype)
            self.get_logger().info("[ADS_Node]No status received yet, reading from PLC...")
            statusUpdate=self.cpy_to_equipment_status_msg(status)
        else:
            statusUpdate=self.lastStatus
        self.statePub.publish(statusUpdate)
        


def main(args=None):
    rclpy.init(args=args)
    pyads_node =  ADS_Node()
    executor = MultiThreadedExecutor()
    executor.add_node(pyads_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pyads_node.get_logger().info("Shutting down due to KeyboardInterrupt")
    finally:
        pyads_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
