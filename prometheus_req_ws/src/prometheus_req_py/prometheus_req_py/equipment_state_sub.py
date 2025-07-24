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
from prometheus_req_interfaces.msg import EquipmentStatus

#TODO:still old name, to change
class Equipment_State_Sub(Node):

    def __init__(self):
        super().__init__('equipment_state_sub')
        self.subscription = self.create_subscription(
            EquipmentStatus,
            'state',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #Testing code
        self.get_logger().info("[Operator_node]Receinving:"+str(msg.em_mr))



def main(args=None):
    rclpy.init(args=args)

    equipment_state_sub = Equipment_State_Sub()

    rclpy.spin(equipment_state_sub)
    equipment_state_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
