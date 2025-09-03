from prometheus_req_py.ADS.utils import reqState,getReqStateMsg
import pyads

def manageLoadTray(self) -> tuple[str,int]:
    '''
    Manage the individual behaviour of the load tray function block.
    :return: A tuple containing the message and the state of the function block.
    '''
    funcState=self.plc.read_by_name("GVL_ATS.requests.loadTray.State",pyads.PLCTYPE_INT)
    self.get_logger().info(f"funcState:{funcState}")

    #after the esecution, we check if there is an error check to be managed.
    if(funcState == reqState.ST_ERROR_CHECK):
        self.get_logger().info("[ADS_Node]Checking Load Tray...")
        self.error_check("Load Tray Error Check")
        self.plc.write_by_name("GVL_ATS.requests.loadTray.errorAck",1,pyads.PLCTYPE_BOOL)
        #Wait for the error check to be solved.
        while(funcState==reqState.ST_ERROR_CHECK):
            funcState=self.plc.read_by_name(f"GVL_ATS.requests.loadTray.state",pyads.PLCTYPE_INT)
        msg="Error check solved"
    else:
        msg=getReqStateMsg(funcState)

    return msg,funcState
