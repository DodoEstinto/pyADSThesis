from prometheus_req_py.ADS.utils import reqState,getReqStateMsg
import pyads

def managePositionerRotate(self) -> tuple[str,int]:
    '''
    Manage the individual behaviour of the positioner rotate function block.
    :return: A tuple containing the message and the state of the function block.
    '''
    #TODO:controllare che state serva veramente (non penso)
    funcState=self.plc.read_by_name("GVL_ATS.requests.positionerRotate.State",pyads.PLCTYPE_INT)
    self.get_logger().info(f"funcState:{funcState}")
    #After the execution, we check if there is an error check to be managed.
    if(funcState == reqState.ST_ERROR_CHECK):
        self.get_logger().info("[ADS_Node]Checking Positioner Rotate...")

        self.error_check("PositionerRotate Error Check")
        self.plc.write_by_name("GVL_ATS.requests.positionerRotate.errorAck",1,pyads.PLCTYPE_BOOL)
        self.get_logger().info("[ADS_Node]ACK sent for Positioner Rotate Error Check!") 
        #Wait for the error check to be solved.
        while(funcState==reqState.ST_ERROR_CHECK):
            funcState=self.plc.read_by_name(f"GVL_ATS.requests.positionerRotate.State",pyads.PLCTYPE_INT)
        msg="Error check solved"
    else:
        msg=getReqStateMsg(funcState)

    return msg,funcState
