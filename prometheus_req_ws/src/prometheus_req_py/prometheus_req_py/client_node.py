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
from rclpy.action import ActionClient
from copy import deepcopy
from prometheus_req_interfaces.msg import EquipmentStatus
from prometheus_req_interfaces.action import CallFunctionBlock
from prometheus_req_py.utils import OkDialog
import tkinter as tk
from tkinter import messagebox,simpledialog
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from prometheus_req_py.utils import req_state,msg_type
from typing import Literal



class Client_Node(Node):
    '''
    This node is designed to be used directly by a human operator. 
    It provides a simplified interface for monitoring and controlling the system.
    '''

    def call_block(self, name:str):
        '''
        This function is called when the building block buttons are pressed.
        It sends an async request to the service server.
        '''
        if not self.functionBlockCalled:
            self.functionBlockCalled=True
            self.get_logger().info(f"[Client_node] Calling Block {name}")
            self.req.function_block_name=name
        
            match(name):
                case "loadTray":
                    answer = messagebox.askyesno("Load Tray", "Load(Yes) or Unload(No) the tray?")
                    self.req.bool_param1=answer
                case "positionerRotate":
                    answer = messagebox.askyesno("Positioner Rotate", "Rotate clockwise(Yes) or anticlockwise(No)?")
                    self.req.bool_param1=answer
                case "screwPickup":
                    screw = simpledialog.askinteger("Pick Up Screw", "Enter the screw number to pick up (1-6):", minvalue=1, maxvalue=6)
                    self.req.int_param1=screw

            self.send_goal_future=self.client.send_goal_async(self.req,feedback_callback=self.goal_feedback_callback)
            self.send_goal_future.add_done_callback(self.goal_response_callback)
            self.get_logger().info(f"[Client_node] Done")
        else:
            self.get_logger().info(f"[Client_node] Function Block already called, waiting for response...")
            self.update_response_text("A function Block has been already called, waiting for its response...", isResult=False)
        
        
                
        
    def goal_response_callback(self,future):
        goalHandler=future.result()
        self.get_logger().info(f"[CLIENT NODE] Action response:Entering")

        if not goalHandler.accepted:
            self.get_logger().info(f"[CLIENT NODE] Action response:Not accepted")
            self.update_response_text("Command not accepted", isResult=False)
            self.functionBlockCalled=False
            return
        
        self.get_logger().info(f"[CLIENT NODE] Action response: Accepted")
        self.update_response_text("Command accepted", isResult=False)

        self.send_goal_future= goalHandler.get_result_async()
        self.send_goal_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self,future):
        self.get_logger().info(f"[CLIENT NODE] Action result:Entering")
        self.functionBlockResult=future.result().result.result
        self.functionBlockState=future.result().result.state
        self.functionBlockMsg=future.result().result.msg
        self.functionBlockCalled=False
        self.update_labels()
        self.update_response_text(self.functionBlockMsg, isResult=True)

        self.get_logger().info(f"[CLIENT NODE] Action result:{self.functionBlockState},{self.functionBlockMsg}")


    
    def goal_feedback_callback(self,feedback):
        #self.get_logger().info(f"[CLIENT NODE]FEEDBACK!")
        msgType=feedback.feedback.msg_type
        self.functionBlockMsg=feedback.feedback.msg
        self.get_logger().info(f"[CLIENT NODE]msgType:{msgType}")
        match msgType:
            case msg_type.ERROR_CHECK:
                self.update_response_text("Error check in progress...", isResult=False)
                _=OkDialog(self.root, title="Error Check", message=self.functionBlockMsg)
                self.get_logger().info(f"[CLIENT] SENDIND ACK!")
                self.errorCheckPub.publish(Empty())
            case msg_type.ASKING_PICTURE:
                self.update_response_text("Asking a photo...", isResult=False)
                _=OkDialog(self.root, title="Take Picture", message="Press ok to take a picture!")
                self.imagePub.publish(Empty())
            case _:
                self.update_labels()
                self.update_response_text(self.functionBlockMsg, isResult=False)
    

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
            'em_general', 
            'em_mr', 
            'em_sr',
            'tem_sens_ok', 
            'air_press_ok',
            'em_laser_scanner', 
            'em_laser_scanner2',
            'em_laser_scanner3',
            'automatic_mode'
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

        # --- Section 4: States (as a text box like ScrewBay) ---
        states_frame = make_section(content_frame, "States")
        self.states_text = tk.Text(states_frame, height=10, width=50, font=("Courier", 10))
        self.states_text.pack()
        self.states_text.insert('1.0', "Waiting for state data...\n")
        self.states_text.configure(state='disabled')

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

        # --- ACtion Response Text Box ---
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
            'em_general', 
            'em_mr', 
            'em_sr', 
            'tem_sens_ok', 
            'air_press_ok',
            'em_laser_scanner', 
            'em_laser_scanner2',
            'em_laser_scanner3',
            'automatic_mode', 
            'mgse_to_conveyor',
            'trolley_in_bay', 
            'side_2_robot', 
            'positioner_is_up',
            'positioner_is_down', 
            'pallet_is_in_wrk_pos', 
            'pallet_is_in_entry_pos',
            'rotary_aligned', 
            'holder_correction_done'
        ]
        for field in all_fields:
            val = getattr(self.state, field)
            bg = color(val) if isinstance(val, bool) else "#ddd"
            self.labels[field].config(text=str(val), bg=bg)

        # --- Update States Text Box (like ScrewBay) ---
        self.states_text.configure(state='normal')
        self.states_text.delete('1.0', tk.END)
        state_field=[
            'active_state_fsm_string',
            'active_state_mr_fsm_string',
            'active_state_sr_fsm_string',
            'active_state_conveyor_string',
            'active_state_system_safety_test'
        ]
        for field in state_field:
            val = getattr(self.state, field)
            self.states_text.insert(tk.END, f"{field}: {val}\n")
        self.states_text.configure(state='disabled')

        # Update ScrewBay Text Box
        self.screw_text.configure(state='normal')
        self.screw_text.delete('1.0', tk.END)
        for i, slot in enumerate(self.state.screw_bay,1):
            self.screw_text.insert(tk.END, f"ScrewBay[{i}]: ")
            self.screw_text.insert(tk.END, f"max=({slot.max_idx_x},{slot.max_idx_y}) ")
            self.screw_text.insert(tk.END, f"next=({slot.next_idx_x},{slot.next_idx_y})\n")
        self.screw_text.configure(state='disabled')
        
        #if self.client_request_response==None:
        #    self.get_logger().info(f"client_request_response:N/A")
        #else:
            #self.get_logger().info(f"client_request_response:{self.client_request_response.result().result}")

        '''
        if(self.functionBlockCalled):
            self.response_text.configure(state='normal')
            self.response_text.delete('1.0', tk.END)
            if(self.functionBlockDone):
                self.update_response_text(self.functionBlockMsg, isMsg=True)
            else:
                msg="Waiting for a response..."
            self.response_text.insert("1.0",msg)
            self.response_text.configure(state="disabled")
        '''
            
    def update_response_text(self, text:str, isResult:bool=False):
        '''
        Update the response text box with a new message.
        If isMsg is True, it will be treated as a message, otherwise as a status update.
        '''
        self.response_text.configure(state='normal')
        self.response_text.delete('1.0', tk.END)
        if isResult:
            if(self.functionBlockResult):
                msg = f"[Success!] Message: {text}"
            else:
                msg = f"[Fail] Message: {text}"
            self.response_text.insert('1.0', f" {msg}")
        else:
            self.response_text.insert('1.0', f"[Status] {text}")
        self.response_text.configure(state='disabled')


    def __init__(self,root):
        super().__init__('client_node')
        self.state=EquipmentStatus()
        self.subscription = self.create_subscription(
            EquipmentStatus,
            'state',
            self.state_update_callback,
            1)
        qos = QoSProfile(
            depth=3,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.errorCheckPub= self.create_publisher(
            Empty,
            'errorCheckAck',
            qos_profile=qos
        )

        self.imagePub = self.create_publisher(
            Empty,
            'takePicture',
            qos_profile=qos
        )

        self.functionBlockCalled=False
        self.functionBlockDone=False
        self.functionBlockState="N/A"
        self.functionBlockMsg="N/A"
        self.functionBlockResult=False
        self.init_GUI(root)
        self.client=ActionClient(self,CallFunctionBlock,"CallFunctionBlock")
        while not self.client.wait_for_server(timeout_sec=1):
            #self.get_logger().info('service not available, waiting again...')
            pass
        self.req=CallFunctionBlock.Goal()
        self.subscription  # prevent unused variable warning

    def state_update_callback(self, msg):
        '''
        Called each time the plc's equipmentstate changes
        '''
        #Testing code
        #self.get_logger().info("[Client_node]Receinving:"+str(msg.active_state_fsm_string))
        self.state=deepcopy(msg)
        self.update_labels()



def main(args=None):
    rclpy.init(args=args)
    root=tk.Tk()
    client_node = Client_Node(root)

    def update_ros():
        rclpy.spin_once(client_node,timeout_sec=0.005)
        #client_node.get_logger().info("[Client_node]Spinning once...")
        root.after(1, update_ros)  # Schedule the next call

    
    root.after(1,update_ros)

  

    def on_close():
        client_node.get_logger().info("[Client_node] Shutting down...")
        client_node.destroy_node()
        rclpy.shutdown()
        root.destroy()  # closes the window and ends mainloop

    root.protocol("WM_DELETE_WINDOW", on_close)  # handles window close
    root.mainloop()

if __name__ == '__main__':
    main()
