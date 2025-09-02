from prometheus_req_py.ADS.utils import reqState,getReqStateMsg
import pyads

def manageStackTray(self,goalHandler) -> tuple[str,int]:
    '''
    Manage the individual behaviour of the stack tray function block.
    :param goalHandler: The goal handler to manage the request.
    :return: A tuple containing the message and the state of the function block.
    '''

    funcState=self.plc.read_by_name(f"GVL_ATS.requests.{goalHandler.request.function_block_name}.State",pyads.PLCTYPE_INT)
    self.get_logger().info(f"funcState:{funcState}")

    if(funcState == reqState.ST_ERROR_CHECK):
        self.get_logger().info("[ADS_Node]Checking Stack Tray...")

        self.error_check("Stack Tray Error Check",goalHandler)
        self.plc.write_by_name(f"GVL_ATS.requests.{goalHandler.request.function_block_name}.errorAck",1,pyads.PLCTYPE_BOOL)
        while(funcState==reqState.ST_ERROR_CHECK):
            funcState=self.plc.read_by_name(f"GVL_ATS.requests.{goalHandler.request.function_block_name}.state",pyads.PLCTYPE_INT)
        msg="Error check solved"
    else:
        msg=getReqStateMsg(funcState)
        return msg,funcState