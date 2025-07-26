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
from copy import deepcopy
from std_msgs.msg import String
from prometheus_req_interfaces.msg import EquipmentStatus
from prometheus_req_interfaces.srv import CallFunctionBlock
import tkinter as tk


class Operator_Node(Node):
    '''
    This node is designed to be used directly by a human operator. 
    It provides a simplified interface for monitoring and controlling the system.
    '''

    def call_block(self, name:str):
        '''
        This function is called when the building block buttons are pressed.
        It sends an async request to the service server.
        '''
        self.get_logger().info(f"[Operator_node] Calling Block {name}")
        self.req.function_block_name=name
        self.client_request_response=self.client.call_async(self.req)
        self.client_request_response.add_done_callback(self.update_callback)
    
    def update_callback(self,data):
        '''
        This function is called when the service client receive a response.
        It updates the GUI.
        '''
        #self.get_logger().info(f'CALLBACK: {data}')
        self.update_labels()
        


    def init_labels(self):
        '''
        Init all the labels with default values.
        '''
    # Reset all label fields to "N/A" and neutral background
        for field, label in self.labels.items():
            label.config(text="N/A", bg="#ddd")

        # Reset screw bay text box
        self.screw_text.configure(state='normal')
        self.screw_text.delete('1.0', tk.END)
        self.screw_text.insert('1.0', "No ScrewBay data available.\n")
        self.screw_text.configure(state='disabled')

    def init_GUI(self, root:tk.Tk):
        '''
        Create the GUI
        '''
        self.root = root
        self.root.title("Prometheus Operator Interface")
        self.labels = {}

        label_font = ("Segoe UI", 10)
        header_font = ("Segoe UI", 12, "bold")

        # --- Main window layout: split left (sections) and right (buttons) ---
        outer_frame = tk.Frame(self.root)
        outer_frame.pack(fill='both', expand=True)

        # Left: content sections
        content_frame = tk.Frame(outer_frame, padx=10, pady=10)
        content_frame.pack(side='left', fill='both', expand=True)

        # Right: buttons
        button_frame = tk.Frame(outer_frame, padx=10, pady=10)
        button_frame.pack(side='right', fill='y')

        def make_section(parent:tk.Frame, title:str) -> tk.LabelFrame:
            frame = tk.LabelFrame(parent, text=title, font=header_font, padx=10, pady=10)
            frame.pack(fill='x', pady=5)
            return frame

        def add_label(frame, field, row):
            label = tk.Label(frame, text=f"{field.replace('_', ' ').capitalize()}:", font=label_font, anchor='w')
            value = tk.Label(frame, text="N/A", font=label_font, width=15, bg="#ddd", anchor='w')
            label.grid(row=row, column=0, sticky='w')
            value.grid(row=row, column=1, sticky='w')
            self.labels[field] = value

        # --- Section 1: Checks ---
        checks_frame = make_section(content_frame, "Checks")
        checks_fields = [
            'em_general', 'em_mr', 'em_sr',
            'tem_sens_ok', 'air_press_ok',
            'em_laser_scanner', 'automatic_mode'
        ]
        for i, field in enumerate(checks_fields):
            add_label(checks_frame, field, i)

        # --- Section 2: Positions ---
        positions_frame = make_section(content_frame, "Positions")
        positions_fields = [
            'mgse_to_conveyor', 'trolley_in_bay', 'side_2_robot',
            'positioner_is_up', 'positioner_is_down',
            'pallet_is_in_wrk_pos', 'pallet_is_in_entry_pos',
            'rotary_aligned', 'holder_correction_done'
        ]
        for i, field in enumerate(positions_fields):
            add_label(positions_frame, field, i)

        # --- Section 3: ScrewBay ---
        screwbay_frame = make_section(content_frame, "ScrewBay")
        self.screw_text = tk.Text(screwbay_frame, height=10, width=50, font=("Courier", 10))
        self.screw_text.pack()
        self.screw_text.insert('1.0', "Waiting for data...\n")
        self.screw_text.configure(state='disabled')

        # --- Section 4: States ---
        states_frame = make_section(content_frame, "States")
        state_fields = [
            'active_state_fsm_string',
            'active_state_mr_fsm_string',
            'active_state_sr_fsm_string',
            'active_state_conveyor_string',
            'active_state_system_safety_test'
        ]
        for i, field in enumerate(state_fields):
            add_label(states_frame, field, i)

        # --- Button column: "Calling Block" ---
        button_header = tk.Label(button_frame, text="Blocks", font=header_font)
        button_header.pack(pady=(0, 10))

        building_blocks = [
            "loadTray",
            "pickUpTray",
            "depositTray",
            "srHoming",
            "mrHoming",
            "mrTrolleyVCheck",
            "gyroGrpRot",
            "screwTight",
            "screwPickup",
            "presentToScrew",
            "positionerRotate",
            "stackTray",
            "present2Op",
            "setScrewBayState"
        ]

        for i in range(len(building_blocks)):  # Adjust number of blocks here
            btn = tk.Button(button_frame,width=40 ,text=f"Calling Block {i}: {building_blocks[i]}", command=lambda i=building_blocks[i]: self.call_block(i))
            btn.pack(pady=5)

        # --- Service Response Text Box ---
        response_label = tk.Label(button_frame, text="Service Response:", font=header_font)
        response_label.pack(pady=(20, 5))

        self.response_text = tk.Text(button_frame, height=4, width=50, font=("Courier", 10))
        self.response_text.pack()
        self.response_text.insert('1.0', "Awaiting service call...\n")
        self.response_text.configure(state='disabled')

        self.init_labels()

    def update_labels(self):
        '''
        Update every value of every field based on the current state.
        '''
        def color(val):
            return "#c8e6c9" if val else "#ffcdd2"

        # Update checks and positions
        all_fields = [
            'em_general', 'em_mr', 'em_sr', 'tem_sens_ok', 'air_press_ok',
            'em_laser_scanner', 'automatic_mode', 'mgse_to_conveyor',
            'trolley_in_bay', 'side_2_robot', 'positioner_is_up',
            'positioner_is_down', 'pallet_is_in_wrk_pos', 'pallet_is_in_entry_pos',
            'rotary_aligned', 'holder_correction_done'
        ]
        for field in all_fields:
            val = getattr(self.state, field)
            bg = color(val) if isinstance(val, bool) else "#ddd"
            self.labels[field].config(text=str(val), bg=bg)

        # Update FSM state strings
        for field in [
            'active_state_fsm_string',
            'active_state_mr_fsm_string',
            'active_state_sr_fsm_string',
            'active_state_conveyor_string',
            'active_state_system_safety_test'
        ]:
            val = getattr(self.state, field)
            self.labels[field].config(text=val, bg="#eee")

        # Update ScrewBay Text Box
        self.screw_text.configure(state='normal')
        self.screw_text.delete('1.0', tk.END)
        for i, slot in enumerate(self.state.screw_bay):
            self.screw_text.insert(tk.END, f"ScrewBay[{i}]: ")
            self.screw_text.insert(tk.END, f"max=({slot.max_idx_x},{slot.max_idx_y}) ")
            self.screw_text.insert(tk.END, f"next=({slot.next_idx_x},{slot.next_idx_y})\n")
        self.screw_text.configure(state='disabled')
        
        if self.client_request_response==None:
            self.get_logger().info(f"client_request_response:N/A")
        else:
            self.get_logger().info(f"client_request_response:{self.client_request_response.result().result}")

        
        if(self.client_request_response!=None):
            self.response_text.configure(state='normal')
            self.response_text.delete('1.0', tk.END)
            if(self.client_request_response.done()):
                try:
                    response = self.client_request_response.result()
                    msg = f"[Success! Message: {response.result}"
                except Exception as e:
                    msg = f"Service call failed: {e}"
                self.client_request_response=None
            else:
                msg="Waiting for a response..."
            self.response_text.insert("1.0",msg)
            self.response_text.configure(state="disabled")
            



    def __init__(self,root):
        super().__init__('operator_node')
        self.state=EquipmentStatus()
        self.subscription = self.create_subscription(
            EquipmentStatus,
            'state',
            self.state_update_callback,
            1)
        self.client_request_response=None
        self.init_GUI(root)
        self.client=self.create_client(CallFunctionBlock,"CallFunctionBlock")
        while not self.client.wait_for_service(timeout_sec=1):
            #self.get_logger().info('service not available, waiting again...')
            pass
        self.req=CallFunctionBlock.Request()
        self.subscription  # prevent unused variable warning

    def state_update_callback(self, msg):
        '''
        Called each time the plc's equipmentstate changes
        '''
        #Testing code
        #self.get_logger().info("[Operator_node]Receinving:"+str(msg.em_mr))
        self.state=deepcopy(msg)
        self.update_labels()



def main(args=None):
    rclpy.init(args=args)
    root=tk.Tk()
    operator_node = Operator_Node(root)

    def update_ros():
        rclpy.spin_once(operator_node,timeout_sec=0.005)
        #operator_node.get_logger().info("[Operator_node]Spinning once...")
        root.after(1, update_ros)  # Schedule the next call

    
    root.after(1,update_ros)

  

    def on_close():
        operator_node.get_logger().info("[Operator_node] Shutting down...")
        operator_node.destroy_node()
        rclpy.shutdown()
        root.destroy()  # closes the window and ends mainloop

    root.protocol("WM_DELETE_WINDOW", on_close)  # handles window close
    root.mainloop()

if __name__ == '__main__':
    main()
