
def checkPositionerRotate(self) -> tuple[bool,str]:
    '''
    Check if the positioner is in the right position.
    :return: A tuple containing a boolean indicating if the positioner is in the right position and, in case of failure, a message explaining it.
    '''
    palletWrkPos=self.plc.read_by_name(f"GVL_ATS.equipmentState.palletIsInWrkPos",pyads.PLCTYPE_BOOL)
    msg=None
    if(not palletWrkPos):
        msg="Pallet not in working position! Please move the pallet to the working position before calling this function block."
    return (palletWrkPos,msg)

def checkLoadTray(self) -> tuple[bool,str]:
    '''
    Check if the tray is in the right position.
    :return: A tuple containing a boolean indicating if the tray is in the right position and, in case of failure, a message explaining it.
    '''
    side2Robot=self.plc.read_by_name(f"GVL_ATS.equipmentState.side2Robot",pyads.PLCTYPE_INT)
    msg=None
    if(side2Robot != 2):
        msg="Tray not in the right position! Please move the tray to the right position before calling this function block."
    return (side2Robot == 2,msg)

def checkScrewPickup(self) -> tuple[bool,str]:
    '''
    Check if the holder correction is done.
    :return: A tuple containing a boolean indicating if the holder correction is done and, in case of failure, a message explaining it.
    '''
    holderCorrectionDone=self.plc.read_by_name(f"GVL_ATS.equipmentState.holderCorrectionDone",pyads.PLCTYPE_BOOL)
    msg=None
    if(not holderCorrectionDone):
        msg= "Holder correction not done! Please perform the offset calibration before calling this function block."
    return (holderCorrectionDone,msg)