from prometheus_req_py.ADS.utils import reqState,getReqStateMsg
import pyads
import time
from prometheus_req_interfaces.action import CallFunctionBlock

def manageScrewErrorCheck(self,goalHandler):
    '''
    Manage the error check for the screw pickup and tight functions block.
    :return: A tuple containing the message and the state of the function block.
    '''
    funcState=reqState.ST_ERROR_CHECK
    functionBlockName=goalHandler.request.function_block_name
    self.get_logger().info(f"[ADS_Node]Checking {functionBlockName} Error Check...")
    #this refers to the ads_node.error_check(...) method
    self.error_check(f"{functionBlockName} Error Check",goalHandler)
    self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.errorAck",1,pyads.PLCTYPE_BOOL)
    self.get_logger().info(f"[ADS_Node]ACK sent for {functionBlockName} Error Check!") 
    while(funcState==reqState.ST_ERROR_CHECK):
        funcState=self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT)
    msg="Error check solved" 
    return msg,funcState

#TODO: once the photo is taken, the plc do a correction and then ask for a new photo, and so on. Remember to handle this.
def manageScrewLogic(self,goalHandler,funcState):
    '''
    Manage the logic of the screw pickup and tight functions block.
    :param funcState: The current state of the function block.
    '''

    self.get_logger().info(f"[DEBUG]Logic funcState:{funcState}")
    functionBlockName=goalHandler.request.function_block_name
    if funcState == reqState.ST_REQ_PENDING | reqState.ST_READY:
        self.get_logger().info("[Debug]Waiting for the picture request...")
        while(not self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.takePicture",pyads.PLCTYPE_BOOL)):
            pass
        self.get_logger().info("[Debug]Picture request received, asking for the picture...")
        #this refers to the ads_node.askPicture(...) method
        x,y,theta=self.askPicture(goalHandler,"Asking Picture")
        self.get_logger().info(f"[Debug]Picture received with offsets: x={x}, y={y}, theta={theta}")
        self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.xVisCorrTray",x,pyads.PLCTYPE_REAL)
        self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.yVisCorrTray",y,pyads.PLCTYPE_REAL)
        self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.thetaVisCorrTray",theta,pyads.PLCTYPE_REAL)
        self.plc.write_by_name(f"GVL_ATS.requests.{functionBlockName}.pictureAvailable",1,pyads.PLCTYPE_BOOL)


        
        #waiting for the pickupScrew state to update
        while(self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT)== reqState.ST_REQ_PENDING):
            if(time.time()-self.lastTime>self.actionTimerDelay):
                    self.lastTime=time.time()
                    feedback_msg = CallFunctionBlock.Feedback()
                    feedback_msg.msg="Waiting for pickupScrew state to update..."
                    goalHandler.publish_feedback(feedback_msg)
                    self.get_logger().info(f"[DEBUG]Still waiting for pickupScrew state to update...")

        self.get_logger().info(f"[DEBUG]{functionBlockName} state updated, waiting for the pickup to finish...")

        #waiting for the pickup operation to finish
        while(self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT) in (
                                                                                                reqState.ST_EXECUTING,
                                                                                                reqState.ST_EXECUTING_2,
                                                                                                reqState.ST_EXECUTING_3,
                                                                                                reqState.ST_EXECUTING_4)):
            #TODO: swap with the util function.
            if(time.time()-self.lastTime>self.actionTimerDelay):
                    self.lastTime=time.time()
                    feedback_msg = CallFunctionBlock.Feedback()
                    feedback_msg.msg="Waiting for the screw pickup..."
                    goalHandler.publish_feedback(feedback_msg)
                    self.get_logger().info(f"[DEBUG]Still executing pickupScrew...")


        funcState=self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT)
        msg=getReqStateMsg(funcState)
        self.get_logger().info(f"[DEBUG]{functionBlockName} completed with msg: {msg}")


def manageScrew(self,goalHandler) -> tuple[str,int]:
    '''
    Manage the individual behaviour of the screw pickup and tight functions block.
    :return: A tuple containing the message and the state of the function block.
    '''

    functionBlockName=goalHandler.request.function_block_name
    funcState=self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT)
    self.get_logger().info(f"[DEBUG]funcState:{funcState}")

    #if(funcState == reqState.ST_READY):
        #self.get_logger().info(f"[DEBUG]Inside the ready if")
        #manageScrewPickupLogic(self,goalHandler,funcState)

    funcState=self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT)
    self.get_logger().info(f"[DEBUG]funcState:{funcState}")

    while(funcState in (reqState.ST_REQ_PENDING,
                        reqState.ST_EXECUTING,
                        reqState.ST_EXECUTING_2,
                        reqState.ST_EXECUTING_3,
                        reqState.ST_EXECUTING_4
                        )):
        
        if(funcState == reqState.ST_REQ_PENDING):
            self.get_logger().info(f"[DEBUG]Inside pending if funcState:{funcState}")
            manageScrewLogic(self,goalHandler,funcState)
            self.get_logger().info(f"[DEBUG]Exiting if funcState:{funcState}")
        funcState= self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT)

    #there is a transaction in this state also in case of error check, we wait for it to finish and analyze the final state.
    if(functionBlockName == "screwPickup"):
        while (funcState == reqState.ST_REQ_COMPLETION):
            funcState=self.plc.read_by_name(f"GVL_ATS.requests.{functionBlockName}.State",pyads.PLCTYPE_INT)

    if(funcState == reqState.ST_ERROR_CHECK):
        msg,funcState=manageScrewErrorCheck(self,goalHandler)
        self.get_logger().info(f"[DEBUG]Exiting  {functionBlockName}...")
    else:
        msg=getReqStateMsg(funcState)
    return msg,funcState

