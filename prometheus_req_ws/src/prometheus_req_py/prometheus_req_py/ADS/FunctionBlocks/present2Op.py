from prometheus_req_py.ADS.utils import reqState,getReqStateMsg
import pyads

#TODO: da implementare
def managePresent2Op(self) -> tuple[str,int]:
    '''
    Manage the individual behaviour of the present2Op function block.
    :return: A tuple containing the message and the state of the function block.
    '''

    funcState=self.plc.read_by_name("GVL_ATS.requests.present2Op.State",pyads.PLCTYPE_INT)
    self.get_logger().info(f"funcState:{funcState}")

    if(funcState == reqState.ST_ERROR_CHECK):
        self.get_logger().info("[ADS_Node]Checking Present2Op...")

        self.error_check("Present2Op Error Check",)
        self.plc.write_by_name("GVL_ATS.requests.present2Op.errorAck",1,pyads.PLCTYPE_BOOL)
        while(funcState==reqState.ST_ERROR_CHECK):
            funcState=self.plc.read_by_name(f"GVL_ATS.requests.present2Op.state",pyads.PLCTYPE_INT)
        msg="Error check solved"
    else:
        msg=getReqStateMsg(funcState)
        return msg,funcState