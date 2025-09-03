from prometheus_req_py.ADS.utils import reqState,getReqStateMsg
import pyads

#TODO: da implementare
def manageDepositTray(self) -> tuple[str,int]:
    '''
    Manage the individual behaviour of the deposit tray function block.
    :return: A tuple containing the message and the state of the function block.
    '''

    funcState=self.plc.read_by_name("GVL_ATS.requests.depositTray.State",pyads.PLCTYPE_INT)
    self.get_logger().info(f"funcState:{funcState}")

    if(funcState == reqState.ST_ERROR_CHECK):
        self.get_logger().info("[ADS_Node]Checking Load Tray...")

        self.error_check("Deposit Tray Error Check")
        self.plc.write_by_name("GVL_ATS.requests.depositTray.errorAck",1,pyads.PLCTYPE_BOOL)
        while(funcState==reqState.ST_ERROR_CHECK):
            funcState=self.plc.read_by_name(f"GVL_ATS.requests.depositTray.state",pyads.PLCTYPE_INT)
        msg="Error check solved"
    else:
        msg=getReqStateMsg(funcState)
        return msg,funcState