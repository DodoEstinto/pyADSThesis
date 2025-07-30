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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


from copy import deepcopy

from prometheus_req_interfaces.msg import EquipmentStatus,ScrewSlot
from prometheus_req_interfaces.action import CallFunctionBlock
from prometheus_req_py.structures import ScrewSlot_ctype,EquipmentStatus_ctype,PLC_STRING_40
from prometheus_req_py.utils import req_state,get_req_type,get_req_state_msg
from std_msgs.msg import String,Empty
from rclpy.action import ActionServer as rclpyActionServer
from threading import Event
class ADS_Node(Node):
    """
    A pyADS node is responsible for establishing and mantaining the connection with the PLC using the pyADS library.
    """

    def __init__(self):
        super().__init__('ads_node')
        #self.init_s_time=time.time_ns()
        #always drop old msg in case of a slowdonw. Keep the newest.
        self.publisher_ = self.create_publisher(EquipmentStatus, 'state', 1)
        timer_period = 1  # seconds
        self.actionServer= rclpyActionServer(self,CallFunctionBlock,"CallFunctionBlock",self.block_execute_callback)
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.ads_error_check_sub= self.create_subscription(
            Empty,
            'errorCheckAck',
            self.error_check_callback,
            qos_profile=qos
        )

        self.errorCheckEvent = False
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lastTime=time.time
        self.actionTimerDelay=5 #seconds
        self.lastStatus=None
        self.declare_parameter("remote_ip","None")
        self.declare_parameter("remote_ads","None")
        self.declare_parameter("CLIENT_NETID","None")
        self.declare_parameter("CLIENT_IP","None")
        self.declare_parameter("PLC_IP","None")
        self.declare_parameter("PLC_NET_ID","None")
        CLIENT_NETID = self.get_parameter('CLIENT_NETID').value
        CLIENT_IP = self.get_parameter('CLIENT_IP').value
        PLC_IP= self.get_parameter('PLC_IP').value
        PLC_NET_ID = self.get_parameter('PLC_NET_ID').value

        pyads.open_port()
        pyads.set_local_address(CLIENT_NETID)

        #change  based on the credential you are connecting to.
        if(False):
            temp=pyads.add_route_to_plc(CLIENT_NETID,CLIENT_IP,PLC_IP,"Administrator","1",route_name="pyADS")
        pyads.close_port()

        self.plc= pyads.Connection(PLC_NET_ID, pyads.PORT_TC3PLC1, PLC_IP)
        self.plc.open()
     
        statusMemory=pyads.NotificationAttrib(ctypes.sizeof(EquipmentStatus_ctype))#ctypes.sizeof(EquipmentStatus_ctype)

        self.test= self.plc.add_device_notification("GVL_ATS.equipmentState",statusMemory,self.status_callback)
        self.plc.write_by_name("GVL_ATS.requests.loadTray.errorAck",1,pyads.PLCTYPE_BOOL)
        self.plc.write_by_name("GVL_ATS.requests.loadTray.errorAck",0,pyads.PLCTYPE_BOOL)



    def block_execute_callback(self,goalHandler):
        self.get_logger().info(f"[DEBUG]block_execute_callback")

        #functionBlockName="positionerRotate"
        #self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.rotateClockwise",0,pyads.PLCTYPE_BOOL)

        #functionBlockName="loadTray"

        functionBlockName=goalHandler.request.function_block_name

        allowedFunctionBlocks=["positionerRotate","loadTray","depositTray"]
        if(functionBlockName    not in allowedFunctionBlocks):
            self.get_logger().info(f"[ADS_Node] Function Block {goalHandler.request.function_block_name} not allowed! Allowed function blocks are: {allowedFunctionBlocks}")
            goalHandler.abort()
            result=CallFunctionBlock.Result()
            result.result=False
            result.msg=f"Function Block {goalHandler.request.function_block_name} not allowed! Allowed function blocks are: {allowedFunctionBlocks}"
            result.state=999
            return result

        
        feedback_msg = CallFunctionBlock.Feedback()
        
        result=CallFunctionBlock.Result()
        self.get_logger().info(f"[DEBUG]GVL_ATS.requests.{functionBlockName}.request")
        
        actualState=self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT)
        test=get_req_state_msg(actualState)
        self.get_logger().info(f"[ADS_NODE]State: {actualState}@@@{test}")
        feedback_msg.error_check=False
        feedback_msg.msg=get_req_state_msg(actualState)
        if(actualState == req_state.ST_READY):
            self.get_logger().info(f"[DEBUG]Ready")
            check,msg=self.runChecks(functionBlockName)
            if(not check):
                feedback_msg.msg=msg
                goalHandler.publish_feedback(feedback_msg)
                goalHandler.abort()
                result.result=False
                result.msg=msg
                result.state=actualState
                self.get_logger().info(f"[ADS_Node] Goal aborted due to checks failure. -> {msg}")
                return result
            
            self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.request",1,pyads.PLCTYPE_BOOL)
            test=self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.request",pyads.PLCTYPE_BOOL)
            #self.get_logger().info(f"[DEBUG]{test}@@@@GVL_ATS.requests.{functionBlockName}.request")


            #Wait the completion of the task
            while(self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT) in (req_state.ST_EXECUTING,
                                                                                                             req_state.ST_EXECUTING_2,
                                                                                                             req_state.ST_EXECUTING_3,
                                                                                                             req_state.ST_EXECUTING_4)):
                if(time.time()-self.lastTime>self.actionTimerDelay):
                    self.lastTime=time.time()
                    feedback_msg.msg="Executing..."
                    goalHandler.publish_feedback(feedback_msg)

            #Wait for busy to be false
            while(self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.Busy",pyads.PLCTYPE_BOOL)): #TODO: serve ancora con il while sopra? Testare!
                if(time.time()-self.lastTime>self.actionTimerDelay):
                    self.lastTime=time.time()
                    feedback_msg.msg="Busy! Waiting..."
                    goalHandler.publish_feedback(feedback_msg)
                

            actualState=self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT)
            result.result=self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.Done",pyads.PLCTYPE_BOOL)
            feedback_msg.msg="Handling the building block..."
            goalHandler.publish_feedback(feedback_msg)
            self.get_logger().info(f"[ADS_Node]: Managing function block {functionBlockName} with state {actualState} and result {result.result}")
            match (functionBlockName):
                case "positionerRotate":
                        result.msg,result.state=self.managePositionerRotate(goalHandler)
                        #result.state=actualState
                case "loadTray":
                        result.msg,result.state=self.manageLoadTray(goalHandler)
                case "depositTray":
                        result.msg=self.manageDepositTray(actualState)
                        result.state=actualState
            '''   
                case 2:#pending
                        
                        while(self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT) ==req_state.ST_REQ_PENDING):
                            if(time.time()-self.lastTime>self.actionTimerDelay):
                                self.lastTime=time.time()
                                feedback_msg.msg="Waiting for the client to take action..."
                                goalHandler.publish_feedback(feedback_msg)
            '''
            goalHandler.succeed()
            self.get_logger().info("[DEBUG] Goal Finished ")
            return result
        else:
            self.get_logger().info(f"[DEBUG]Not Ready")
            goalHandler.abort()
            result.result=False
            result.msg="Machine not ready! Please wait for the machine to be ready"
            result.state=actualState
            return result
    
    def runChecks(self,functionBlockName:str) -> tuple[bool,str]:
        """
        Run the checks before executing a function block.
        :param functionBlockName: The name of the function block to check.
        :return: A tuple containing a boolean indicating if the checks passed and a message.
        """
        match (functionBlockName):
            case "positionerRotate":
                #return self.checkPositionerRotate()
                return (True,"")
            case "loadTray":
                return self.checkLoadTray()
            case _:
                return (True,None) #No checks for other function blocks, return True and None message.
        
    def checkPositionerRotate(self) -> tuple[bool,str]:
        palletWrkPos=self.plc.read_by_name(f"GVL_ATS.equipmentState.palletIsInWrkPos",pyads.PLCTYPE_BOOL)
        msg=None
        if(not palletWrkPos):
            msg="Pallet not in working position! Please move the pallet to the working position before calling this function block."
        return (palletWrkPos,msg)

    def checkLoadTray(self) -> tuple[bool,str]:

        side2Robot=self.plc.read_by_name(f"GVL_ATS.equipmentState.side2Robot",pyads.PLCTYPE_INT)
        msg=None
        if(side2Robot != 0):
            msg="Tray not in the right position! Please move the tray to the right position before calling this function block."
        return (side2Robot == 0,msg)

    def error_check_callback(self, _):
        """
        Callback for the error check action.
        This function is called when the client aknowledge the error check.
        """
        self.get_logger().info("[ADS]ERROR CHECK CALLBACK!")
        self.errorCheckEvent=True


    def error_check(self,err_msg:str,goalHandler):
        msg_feed=CallFunctionBlock.Feedback()
        msg_feed.error_check=True
        msg_feed.msg=err_msg
        goalHandler.publish_feedback(msg_feed)
        self.get_logger().info("MANDATO!")
        while(not self.errorCheckEvent):
            rclpy.spin_once(self,timeout_sec=0.01)
            
        self.get_logger().info("Uscito!")
        self.errorCheckEvent=False

    def managePositionerRotate(self,goalHandler)->str:
        funcState=self.plc.read_by_name("GVL_ATS.requests.positionerRotate.State",pyads.PLCTYPE_INT)
        state=req_state.ST_ERROR_CHECK
        self.get_logger().info(f"funcState:{funcState}")
        if(funcState == int(req_state.ST_ERROR_CHECK)):
            self.get_logger().info("[ADS_Node]Checking Positioner Rotate...")

            self.error_check("PositionerRotate Error Check",goalHandler)
            self.plc.write_by_name("GVL_ATS.requests.positionerRotate.errorAck",1,pyads.PLCTYPE_BOOL)
            self.get_logger().info("[ADS_Node]ACK sent for Positioner Rotate Error Check!") 
            while(state==int(req_state.ST_ERROR_CHECK)):
                state=self.plc.read_by_name(f"GVL_ATS.requests.{goalHandler.request.function_block_name}.State",pyads.PLCTYPE_INT)
            msg="Error check solved"
        else:
            msg=get_req_state_msg(state)

        return msg,state
    
    def manageLoadTray(self,goalHandler)-> str:
        funcState=self.plc.read_by_name("GVL_ATS.requests.loadTray.State",pyads.PLCTYPE_INT)
        self.get_logger().info(f"funcState:{funcState}")

        if(funcState == req_state.ST_ERROR_CHECK):
            self.get_logger().info("[ADS_Node]Checking Load Tray...")

            self.error_check("Load Tray Error Check",goalHandler)
            self.plc.write_by_name("GVL_ATS.requests.loadTray.errorAck",1,pyads.PLCTYPE_BOOL)
            while(funcState==req_state.ST_ERROR_CHECK):
                funcState=self.plc.read_by_name(f"GVL_ATS.requests.{goalHandler.request.function_block_name}.state",pyads.PLCTYPE_INT)
            msg="Error check solved"
        else:
            msg=get_req_state_msg(funcState)

        return msg,funcState
    
    def manageDepositTray(self,state:int):
        return get_req_state_msg(state)

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
        dst.active_state_fsm_string = str(src.activeStateFSMString)
        dst.active_state_mr_fsm_string = str(src.activeStateMRFSMString)
        dst.active_state_sr_fsm_string = str(src.activeStateSRFSMString)
        dst.active_state_conveyor_string = str(src.activeStateConveyorString)
        dst.active_state_system_safety_test = str(src.activeStateSystemSafetyTest)
        return dst

    def status_callback(self,notification,_): #_=data
        '''
        This function is called each time equipementStatus on the plc changes.
        '''
        #WARNING: if you change the type of the notification, you have also to update it in the statusMemory init!
        handle,timestamp,value=self.plc.parse_notification(notification,EquipmentStatus_ctype)
        #handle,timestamp,value=self.plc.parse_notification(notification,ctypes.c_ubyte)
        
        statusUpdate=self.cpy_to_equipment_status_msg(value)
 

        self.publisher_.publish(statusUpdate)
        self.lastStatus=deepcopy(statusUpdate)
        #self.get_logger().info("[ADS_Node]Sending status update!")


    def timer_callback(self):
        if(self.lastStatus is None):
            status=self.plc.read_by_name('GVL_ATS.equipmentState',EquipmentStatus_ctype)
            self.get_logger().info("[ADS_Node]No status received yet, reading from PLC...")
            statusUpdate=self.cpy_to_equipment_status_msg(status)
        else:
            statusUpdate=self.lastStatus
        self.publisher_.publish(statusUpdate)
        self.get_logger().debug(f"[ADS_Node]Sending periodic status update!")
        #rclpy.spin_once(self,timeout_sec=0.01)
        

def main(args=None):
    rclpy.init(args=args)
    pyads_node =  ADS_Node()
    rclpy.spin(pyads_node)
    pyads_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
