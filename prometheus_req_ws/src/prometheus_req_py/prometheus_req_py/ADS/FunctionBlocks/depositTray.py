from prometheus_req_py.ADS.utils import req_state,get_req_state_msg
import pyads

#TODO: da implementare
def manageDepositTray(self,goalHandler) -> tuple[str,int]:

    funcState=self.plc.read_by_name("GVL_ATS.requests.depositTray.State",pyads.PLCTYPE_INT)
    self.get_logger().info(f"funcState:{funcState}")

    if(funcState == req_state.ST_ERROR_CHECK):
        self.get_logger().info("[ADS_Node]Checking Load Tray...")

        self.error_check("Deposit Tray Error Check",goalHandler)
        self.plc.write_by_name("GVL_ATS.requests.depositTray.errorAck",1,pyads.PLCTYPE_BOOL)
        while(funcState==req_state.ST_ERROR_CHECK):
            funcState=self.plc.read_by_name(f"GVL_ATS.requests.{goalHandler.request.function_block_name}.state",pyads.PLCTYPE_INT)
        msg="Error check solved"
    else:
        msg=get_req_state_msg(funcState)
        return msg,funcState