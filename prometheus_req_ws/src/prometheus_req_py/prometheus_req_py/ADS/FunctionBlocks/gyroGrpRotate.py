from prometheus_req_py.ADS.utils import reqState,getReqStateMsg
import pyads

def manageGyroGrpRotate(self,goalHandler) -> tuple[str,int]:
    '''
    Manage the individual behaviour of the gyro gripper rotate function block.
    :goalHandler: The goal handler for the request.
    :return: A tuple containing the message and the state of the function block.
    '''
    
    funcState=self.plc.read_by_name(f"GVL_ATS.requests.gyroGrpRot.State",pyads.PLCTYPE_INT)
    self.get_logger().info(f"funcState:{funcState}")

    if(funcState == reqState.ST_ERROR_CHECK):
        self.get_logger().info("[ADS_Node]Checking Gyro Gripper Rotate...")

        self.error_check("Gyro Gripper Rotate Error Check",goalHandler)
        self.plc.write_by_name(f"GVL_ATS.requests.gyroGrpRot.errorAck",1,pyads.PLCTYPE_BOOL)
        while(funcState==reqState.ST_ERROR_CHECK):
            funcState=self.plc.read_by_name(f"GVL_ATS.requests.gyroGrpRot.state",pyads.PLCTYPE_INT)
        msg="Error check solved"
    else:
        msg=getReqStateMsg(funcState)
    return msg,funcState