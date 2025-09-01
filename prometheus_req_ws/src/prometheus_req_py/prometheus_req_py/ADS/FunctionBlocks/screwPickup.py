from prometheus_req_py.ADS.utils import reqState,getReqStateMsg
import pyads
import time
from prometheus_req_interfaces.action import CallFunctionBlock

def manageScrewPickupErrorCheck(self,goalHandler):
    '''
    Manage the error check for the screw pickup function block.
    :param goalHandler: The goal handler to manage the request.
    :return: A tuple containing the message and the state of the function block.
    '''
    funcState=reqState.ST_ERROR_CHECK
    self.get_logger().info("[ADS_Node]Checking pickupScrew Error Check...")
    #this refers to the ads_node.error_check(...) method
    self.error_check("pickupScrew Error Check",goalHandler)
    self.plc.write_by_name("GVL_ATS.requests.screwPickup.errorAck",1,pyads.PLCTYPE_BOOL)
    self.get_logger().info("[ADS_Node]ACK sent for pickupScrew Error Check!") 
    while(funcState==reqState.ST_ERROR_CHECK):
        funcState=self.plc.read_by_name(f"GVL_ATS.requests.screwPickup.State",pyads.PLCTYPE_INT)
    msg="Error check solved" 
    return msg,funcState

def manageScrewPickupLogic(self,goalHandler,funcState):
    '''
    Manage the logic of the screw pickup function block.
    :param goalHandler: The goal handler to manage the request.
    :param funcState: The current state of the function block.
    '''

    self.get_logger().info(f"[DEBUG]Logic funcState:{funcState}")
    if funcState == reqState.ST_REQ_PENDING | reqState.ST_READY:
        self.get_logger().info("[Debug]Waiting for the picture request...")
        while(not self.plc.read_by_name(f"GVL_ATS.requests.screwPickup.takePicture",pyads.PLCTYPE_BOOL)):
            pass
        self.get_logger().info("[Debug]Picture request received, asking for the picture...")
        #this refers to the ads_node.askPicture(...) method
        x,y,theta=self.askPicture("Asking Picture",goalHandler)
        self.get_logger().info(f"[Debug]Picture received with offsets: x={x}, y={y}, theta={theta}")
        self.plc.write_by_name("GVL_ATS.requests.screwPickup.xVisCorrTray",x,pyads.PLCTYPE_REAL)
        self.plc.write_by_name("GVL_ATS.requests.screwPickup.yVisCorrTray",y,pyads.PLCTYPE_REAL)
        self.plc.write_by_name("GVL_ATS.requests.screwPickup.thetaVisCorrTray",theta,pyads.PLCTYPE_REAL)
        self.plc.write_by_name("GVL_ATS.requests.screwPickup.pictureAvailable",1,pyads.PLCTYPE_BOOL)

        
        #waiting for the pickupScrew state to update
        while(self.plc.read_by_name(f"GVL_ATS.requests.screwPickup.State",pyads.PLCTYPE_INT)== reqState.ST_REQ_PENDING):
            if(time.time()-self.lastTime>self.actionTimerDelay):
                    self.lastTime=time.time()
                    feedback_msg = CallFunctionBlock.Feedback()
                    feedback_msg.msg="Waiting for pickupScrew state to update..."
                    goalHandler.publish_feedback(feedback_msg)

        #waiting for the pickup operation to finish
        while(self.plc.read_by_name(f"GVL_ATS.requests.screwPickup.State",pyads.PLCTYPE_INT) in (
                                                                                                reqState.ST_EXECUTING,
                                                                                                reqState.ST_EXECUTING_2,
                                                                                                reqState.ST_EXECUTING_3,
                                                                                                reqState.ST_EXECUTING_4)):
            if(time.time()-self.lastTime>self.actionTimerDelay):
                    self.lastTime=time.time()
                    feedback_msg = CallFunctionBlock.Feedback()
                    feedback_msg.msg="Waiting for the screw pickup..."
                    goalHandler.publish_feedback(feedback_msg)


        funcState=self.plc.read_by_name(f"GVL_ATS.requests.screwPickup.State",pyads.PLCTYPE_INT)
        msg=getReqStateMsg(funcState)
        self.get_logger().info(f"[DEBUG]PickupScrew completed with msg: {msg}")


def manageScrewPickup(self,goalHandler) -> tuple[str,int]:
    '''
    Manage the individual behaviour of the screw pickup function block.
    :param goalHandler: The goal handler to manage the request.
    :return: A tuple containing the message and the state of the function block.
    '''

    funcState=self.plc.read_by_name("GVL_ATS.requests.screwPickup.State",pyads.PLCTYPE_INT)
    self.get_logger().info(f"[DEBUG]funcState:{funcState}")

    if(funcState == reqState.ST_READY):
        self.get_logger().info(f"[DEBUG]Inside the if")
        self.manageScrewPickupLogic(goalHandler,funcState)

    funcState=self.plc.read_by_name("GVL_ATS.requests.screwPickup.State",pyads.PLCTYPE_INT)
    self.get_logger().info(f"[DEBUG]funcState:{funcState}")

    while(funcState in (reqState.ST_REQ_PENDING,
                        reqState.ST_EXECUTING,
                        reqState.ST_EXECUTING_2,
                        reqState.ST_EXECUTING_3,
                        reqState.ST_EXECUTING_4
                        )):
        self.get_logger().info(f"[DEBUG]Inside while funcState:{funcState}")
        
        if(funcState == reqState.ST_REQ_PENDING):
            self.get_logger().info(f"[DEBUG]Inside if funcState:{funcState}")
            self.manageScrewPickupLogic(goalHandler,funcState)
            self.get_logger().info(f"[DEBUG]Exiting if funcState:{funcState}")
        funcState= self.plc.read_by_name("GVL_ATS.requests.screwPickup.State",pyads.PLCTYPE_INT)
    
    #there is a transaction in this state also in case of error check, we wait for it to finish and analyze the final state.
    while (funcState == reqState.ST_REQ_COMPLETION):
        funcState=self.plc.read_by_name("GVL_ATS.requests.screwPickup.State",pyads.PLCTYPE_INT)

    if(funcState == reqState.ST_ERROR_CHECK):
        msg,funcState=self.manageScrewPickupErrorCheck(goalHandler)
        self.get_logger().info(f"[DEBUG]Exiting  pickupScrew...")
    else:
        msg=getReqStateMsg(funcState)
    return msg,funcState

