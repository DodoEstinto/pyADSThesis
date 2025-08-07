from prometheus_req_py.ADS.utils import req_state,get_req_state_msg,msg_type
import pyads  
import time
from prometheus_req_interfaces.action import CallFunctionBlock

#TODO: pratically identical to manageScrewPickup, consider merging them.
#TODO: 
def manageMrTrolleyVCheck(self,goalHandler):
    '''
    Manage the individual behaviour of the MR Trolley V Check function block.
    :param goalHandler: The goal handler to manage the request.
    :return: A tuple containing the message and the state of the function block.
    '''
    funcState=self.plc.read_by_name("GVL_ATS.requests.mrTrolleyVCheck.State",pyads.PLCTYPE_INT)
    self.get_logger().info(f"funcState:{funcState}")
    match funcState:
        case req_state.ST_ERROR_CHECK:
            msg,funcState=self.manageMrTrolleyVCheckErrorCheck(goalHandler)
        #TODO: controllare se serve veramente lo state ready
        case req_state.ST_REQ_PENDING | req_state.ST_READY:
            self.get_logger().info("[Debug]Waiting for the picture request...")
            #TODO: sostituire il goalHandler con la stringa.
            # Wait for the picture request to be sent.
            while(not self.plc.read_by_name(f"GVL_ATS.requests.{goalHandler.request.function_block_name}.takePicture",pyads.PLCTYPE_BOOL)):
                pass
            self.get_logger().info("[Debug]Picture request received, asking for the picture...")
            x,y,theta=self.askPicture("Asking Picture",goalHandler)
            self.get_logger().info(f"[Debug]Picture received with offsets: x={x}, y={y}, theta={theta}")
            self.plc.write_by_name("GVL_ATS.requests.mrTrolleyVCheck.xVisCorrTray",x,pyads.PLCTYPE_REAL)
            self.plc.write_by_name("GVL_ATS.requests.mrTrolleyVCheck.yVisCorrTray",y,pyads.PLCTYPE_REAL)
            self.plc.write_by_name("GVL_ATS.requests.mrTrolleyVCheck.thetaVisCorrTray",theta,pyads.PLCTYPE_REAL)
            self.plc.write_by_name("GVL_ATS.requests.mrTrolleyVCheck.pictureAvailable",1,pyads.PLCTYPE_BOOL)

            while(self.plc.read_by_name(f"GVL_ATS.requests.{goalHandler.request.function_block_name}.State",pyads.PLCTYPE_INT) == req_state.ST_REQ_PENDING):
                if(time.time()-self.lastTime>self.actionTimerDelay):
                        self.lastTime=time.time()
                        feedback_msg = CallFunctionBlock.Feedback()
                        feedback_msg.msg="Wait for MR Trolley V Check state to update..."
                        goalHandler.publish_feedback(feedback_msg)

            #TODO: technically this should never happen, but just in case.
            while(self.plc.read_by_name(f"GVL_ATS.requests.{goalHandler.request.function_block_name}.State",pyads.PLCTYPE_INT) in (
                                                                                                                                req_state.ST_EXECUTING,
                                                                                                                                req_state.ST_EXECUTING_2,
                                                                                                                                req_state.ST_EXECUTING_3,
                                                                                                                                req_state.ST_EXECUTING_4)):
                if(time.time()-self.lastTime>self.actionTimerDelay):
                    self.lastTime=time.time()
                    feedback_msg = CallFunctionBlock.Feedback()
                    feedback_msg.msg="Executing MR Trolley V Check..."
                    goalHandler.publish_feedback(feedback_msg)
            funcState=self.plc.read_by_name(f"GVL_ATS.requests.{goalHandler.request.function_block_name}.State",pyads.PLCTYPE_INT)
            msg=get_req_state_msg(funcState)
            self.get_logger().info(f"[DEBUG]MR Trolley V Check completed with msg: {msg}")
            #After the execution, we check if there is an error check to be managed.
            if(funcState == req_state.ST_ERROR_CHECK):
                msg,funcState=self.manageMrTrolleyVCheckErrorCheck(goalHandler)
            self.get_logger().info(f"[DEBUG]Exiting V check...")
        case _:
            msg=get_req_state_msg(funcState)

    return msg,funcState#TODO: pratically identical to manageScrewPickup, consider merging them.
#TODO: 
def manageMrTrolleyVCheck(self,goalHandler):
    '''
    Manage the individual behaviour of the MR Trolley V Check function block.
    :param goalHandler: The goal handler to manage the request.
    :return: A tuple containing the message and the state of the function block.
    '''
    funcState=self.plc.read_by_name("GVL_ATS.requests.mrTrolleyVCheck.State",pyads.PLCTYPE_INT)
    self.get_logger().info(f"funcState:{funcState}")
    match funcState:
        case req_state.ST_ERROR_CHECK:
            msg,funcState=self.manageMrTrolleyVCheckErrorCheck(goalHandler)
        #TODO: controllare se serve veramente lo state ready
        case req_state.ST_REQ_PENDING | req_state.ST_READY:
            self.get_logger().info("[Debug]Waiting for the picture request...")
            #TODO: sostituire il goalHandler con la stringa.
            # Wait for the picture request to be sent.
            while(not self.plc.read_by_name(f"GVL_ATS.requests.{goalHandler.request.function_block_name}.takePicture",pyads.PLCTYPE_BOOL)):
                pass
            self.get_logger().info("[Debug]Picture request received, asking for the picture...")
            x,y,theta=self.askPicture("Asking Picture",goalHandler)
            self.get_logger().info(f"[Debug]Picture received with offsets: x={x}, y={y}, theta={theta}")
            self.plc.write_by_name("GVL_ATS.requests.mrTrolleyVCheck.xVisCorrTray",x,pyads.PLCTYPE_REAL)
            self.plc.write_by_name("GVL_ATS.requests.mrTrolleyVCheck.yVisCorrTray",y,pyads.PLCTYPE_REAL)
            self.plc.write_by_name("GVL_ATS.requests.mrTrolleyVCheck.thetaVisCorrTray",theta,pyads.PLCTYPE_REAL)
            self.plc.write_by_name("GVL_ATS.requests.mrTrolleyVCheck.pictureAvailable",1,pyads.PLCTYPE_BOOL)

            while(self.plc.read_by_name(f"GVL_ATS.requests.{goalHandler.request.function_block_name}.State",pyads.PLCTYPE_INT) == req_state.ST_REQ_PENDING):
                if(time.time()-self.lastTime>self.actionTimerDelay):
                        self.lastTime=time.time()
                        feedback_msg = CallFunctionBlock.Feedback()
                        feedback_msg.msg="Wait for MR Trolley V Check state to update..."
                        goalHandler.publish_feedback(feedback_msg)

            #TODO: technically this should never happen, but just in case.
            while(self.plc.read_by_name(f"GVL_ATS.requests.{goalHandler.request.function_block_name}.State",pyads.PLCTYPE_INT) in (
                                                                                                                                req_state.ST_EXECUTING,
                                                                                                                                req_state.ST_EXECUTING_2,
                                                                                                                                req_state.ST_EXECUTING_3,
                                                                                                                                req_state.ST_EXECUTING_4)):
                if(time.time()-self.lastTime>self.actionTimerDelay):
                    self.lastTime=time.time()
                    feedback_msg = CallFunctionBlock.Feedback()
                    feedback_msg.msg="Executing MR Trolley V Check..."
                    goalHandler.publish_feedback(feedback_msg)
            funcState=self.plc.read_by_name(f"GVL_ATS.requests.{goalHandler.request.function_block_name}.State",pyads.PLCTYPE_INT)
            msg=get_req_state_msg(funcState)
            self.get_logger().info(f"[DEBUG]MR Trolley V Check completed with msg: {msg}")
            #After the execution, we check if there is an error check to be managed.
            if(funcState == req_state.ST_ERROR_CHECK):
                msg,funcState=self.manageMrTrolleyVCheckErrorCheck(goalHandler)
            self.get_logger().info(f"[DEBUG]Exiting V check...")
        case _:
            msg=get_req_state_msg(funcState)

    return msg,funcState


def manageMrTrolleyVCheckErrorCheck(self,goalHandler):
    '''
    Manage the error check for the MR Trolley V Check fucntion block.
    :param goalHandler: The goal handler to manage the request.
    :return: A tuple containing the message and the state of the function block.
    '''
    funcState=req_state.ST_ERROR_CHECK
    self.get_logger().info("[ADS_Node]Checking MR Trolley V Error Check...")
    self.error_check("MR Trolley V Check Error Check",goalHandler)
    self.plc.write_by_name("GVL_ATS.requests.mrTrolleyVCheck.errorAck",1,pyads.PLCTYPE_BOOL)
    self.get_logger().info("[ADS_Node]ACK sent for MR Trolley V Check Error Check!") 
    while(funcState==req_state.ST_ERROR_CHECK):
        funcState=self.plc.read_by_name(f"GVL_ATS.requests.{goalHandler.request.function_block_name}.State",pyads.PLCTYPE_INT)
    msg="Error check solved" 
    return msg,funcState

def askPicture(self,msg,goalHandler):
    '''
    Handle the ask picture action.
    '''
    msg_feed=CallFunctionBlock.Feedback()
    msg_feed.msg_type=msg_type.ASKING_PICTURE
    msg_feed.msg=msg
    goalHandler.publish_feedback(msg_feed)
    self.get_logger().info("[Debug]Waiting for the picture...")
    while(not self.askPictureEvent):
        #rclpy.spin_once(self)
        pass
    self.askPictureEvent=False
    return self.calculate_picture_offset(self.picture)
    