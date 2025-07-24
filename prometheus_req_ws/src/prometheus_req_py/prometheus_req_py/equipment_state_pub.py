# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from getpass import getpass
import pyads
import time
import ctypes

from prometheus_req_interfaces.msg import EquipmentStatus
from prometheus_req_py.structures import ScrewSlot_ctype,EquipmentStatus_ctype,PLC_STRING_40

#TODO:still old name, to change
class Equipment_State_Pub(Node):

    def __init__(self):
        super().__init__('equipment_state_pub')
        self.init_s_time=time.time_ns()
        #always drop old msg in case of a slowdonw. Keep the newest.
        self.publisher_ = self.create_publisher(EquipmentStatus, 'state', 1)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter("remote_ip","None")
        self.declare_parameter("remote_ads","None")
        self.declare_parameter("CLIENT_NETID","None")
        self.declare_parameter("CLIENT_IP","None")
        self.declare_parameter("PLC_IP","None")
        self.declare_parameter("PLC_NET_ID","None")
        remote_ip = self.get_parameter('remote_ip').value
        remote_ads = self.get_parameter('remote_ads').value
        CLIENT_NETID = self.get_parameter('CLIENT_NETID').value
        CLIENT_IP = self.get_parameter('CLIENT_IP').value

        PLC_IP= self.get_parameter('PLC_IP').value
        PLC_NET_ID = self.get_parameter('PLC_NET_ID').value

        pyads.open_port()
        pyads.set_local_address(CLIENT_NETID)
        #adr=pyads.AmsAddr(CLIENT_NETID,851)
        #pyads.add_route(adr,remote_ip)

        #change dave0 based on the credential you are connecting to.
        pyads.add_route_to_plc(CLIENT_NETID,CLIENT_IP,PLC_IP,"dave0",getpass())
        pyads.close_port()

        self.plc= pyads.Connection(remote_ads, pyads.PORT_TC3PLC1, remote_ip)
        self.plc.open()
        statusMemory=pyads.NotificationAttrib(ctypes.sizeof(EquipmentStatus_ctype))#ctypes.sizeof(EquipmentStatus_ctype)
        #statusMemory.trans_mode=pyads.ADSTRANS_SERVERCYCLE
        #statusMemory.nCycleTime=1

        test= self.plc.add_device_notification("GVL_ATS.equipmentState",statusMemory,self.status_callback)

    def status_callback(self,notification,data):

        #WARNING: if you change the type of the notification, you have also to update it in the statusMemory init!
        handle,timestamp,value=self.plc.parse_notification(notification,EquipmentStatus_ctype)
        #handle,timestamp,value=self.plc.parse_notification(notification,ctypes.c_ubyte)

        statusUpdate=EquipmentStatus()
        statusUpdate.em_general =  value.emGeneral
        statusUpdate.em_mr = value.emMR
        statusUpdate.em_sr = value.emSR
        statusUpdate.tem_sens_ok = value.temSensOk
        statusUpdate.air_press_ok = value.airPressOk
        statusUpdate.em_laser_scanner = value.emLaserScanner
        statusUpdate.automatic_mode = value.automaticMode
        statusUpdate.mgse_to_conveyor = value.mgseToConveyor
        statusUpdate.trolley_in_bay = value.trolleyInBay
        statusUpdate.side_2_robot = value.side2Robot
        statusUpdate.positioner_is_up = value.positionerIsUp
        statusUpdate.positioner_is_down = value.positionerIsDown
        statusUpdate.pallet_is_in_wrk_pos = value.palletIsInWrkPos
        statusUpdate.pallet_is_in_entry_pos = value.palletIsInEntryPos
        statusUpdate.rotary_aligned = value.rotaryAligned
        statusUpdate.holder_correction_done = value.holderCorrectionDone
        
        #statusUpdate.screw_bay = value.screwBay
        
        statusUpdate.active_state_fsm_string = value.activeStateFSMString.data.decode('utf-8')
        statusUpdate.active_state_mr_fsm_string = value.activeStateMRFSMString.data.decode('utf-8')
        statusUpdate.active_state_sr_fsm_string = value.activeStateSRFSMString.data.decode('utf-8')
        statusUpdate.active_state_conveyor_string = value.activeStateConveyorString.data.decode('utf-8')
        statusUpdate.active_state_system_safety_test = value.activeStateSystemSafetyTest.data.decode('utf-8')

        self.publisher_.publish(statusUpdate)
        #self.get_logger().info("[pyADS_node]Sending status update!")


    def timer_callback(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    equipment_state_pub =  Equipment_State_Pub()
    rclpy.spin(equipment_state_pub)
    equipment_state_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
