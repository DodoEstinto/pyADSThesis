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
import tkinter as tk


#TODO:still old name, to change
class Equipment_State_Sub(Node):

    def init_labels(self):
    # Reset all label fields to "N/A" and neutral background
        for field, label in self.labels.items():
            label.config(text="N/A", bg="#ddd")

        # Reset screw bay text box
        self.screw_text.configure(state='normal')
        self.screw_text.delete('1.0', tk.END)
        self.screw_text.insert('1.0', "No ScrewBay data available.\n")
        self.screw_text.configure(state='disabled')
    def init_GUI(self, root):
        self.root = root
        self.root.title("Prometheus Operator Interface")
        self.labels = {}

        label_font = ("Segoe UI", 10)
        header_font = ("Segoe UI", 12, "bold")

        main_frame = tk.Frame(self.root, padx=10, pady=10)
        main_frame.pack(fill='both', expand=True)

        def make_section(parent, title):
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
        checks_frame = make_section(main_frame, "Checks")
        checks_fields = [
            'em_general', 'em_mr', 'em_sr',
            'tem_sens_ok', 'air_press_ok',
            'em_laser_scanner', 'automatic_mode'
        ]
        for i, field in enumerate(checks_fields):
            add_label(checks_frame, field, i)

        # --- Section 2: Positions ---
        positions_frame = make_section(main_frame, "Positions")
        positions_fields = [
            'mgse_to_conveyor', 'trolley_in_bay', 'side_2_robot',
            'positioner_is_up', 'positioner_is_down',
            'pallet_is_in_wrk_pos', 'pallet_is_in_entry_pos',
            'rotary_aligned', 'holder_correction_done'
        ]
        for i, field in enumerate(positions_fields):
            add_label(positions_frame, field, i)

        # --- Section 3: ScrewBay ---
        screwbay_frame = make_section(main_frame, "ScrewBay")

        # Use a Text widget as a read-only display box
        self.screw_text = tk.Text(screwbay_frame, height=10, width=50, font=("Courier", 10))
        self.screw_text.pack()
        self.screw_text.insert('1.0', "Waiting for data...\n")
        self.screw_text.configure(state='disabled')

        # --- Section 4: States ---
        states_frame = make_section(main_frame, "States")
        state_fields = [
            'active_state_fsm_string',
            'active_state_mr_fsm_string',
            'active_state_sr_fsm_string',
            'active_state_conveyor_string',
            'active_state_system_safety_test'
        ]
        for i, field in enumerate(state_fields):
            add_label(states_frame, field, i)
        self.init_labels()

    def update_labels(self):
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

    def __init__(self,root):
        super().__init__('equipment_state_sub')
        self.state=EquipmentStatus()
        self.subscription = self.create_subscription(
            EquipmentStatus,
            'state',
            self.state_update_callback,
            1)
        self.init_GUI(root)
        self.update_labels()
        self.subscription  # prevent unused variable warning

    def state_update_callback(self, msg):
        #Testing code
        self.get_logger().info("[Operator_node]Receinving:"+str(msg.em_mr))
        self.state=deepcopy(msg)
        self.update_labels()



def main(args=None):
    rclpy.init(args=args)
    root=tk.Tk()
    equipment_state_sub = Equipment_State_Sub(root)

    def update_ros():
        rclpy.spin_once(equipment_state_sub,timeout_sec=0.005)
        #equipment_state_sub.get_logger().info("[Operator_node]Spinning once...")
        root.after(1, update_ros)  # Schedule the next call

    
    root.after(1,update_ros)

    root.mainloop()
    equipment_state_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
