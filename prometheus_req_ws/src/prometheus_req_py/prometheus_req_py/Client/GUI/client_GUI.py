
import tkinter as tk

def init_labels(self):
    '''
    Initialize all labels to "N/A" and neutral background.
    This is called at the start of the GUI to ensure all fields are reset.
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
        ''' 
        Create a labeled section frame.
        '''
        frame = tk.LabelFrame(parent, text=title, font=header_font, padx=10, pady=10)
        frame.pack(fill='x', pady=5)
        return frame

    def add_label(frame, field, row):
        '''
        Add a label to the given frame for the specified field.
        '''
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
    ''' Update all labels with the current state values.
    It uses a color coding scheme:
    - Green for True values
    - Red for False values
    - Grey for N/A or unknown values
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

