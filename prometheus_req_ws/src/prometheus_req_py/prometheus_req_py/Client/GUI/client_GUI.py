
import tkinter as tk

def initLabels(self):
    '''
    Initialize all labels to "N/A" and neutral background.
    This is called at the start of the GUI to ensure all fields are reset.
    '''
# Reset all label fields to "N/A" and neutral background
    for field, label in self.labels.items():
        label.config(text="N/A", bg="#ddd")

    # Reset screw bay text box
    self.screwText.configure(state='normal')
    self.screwText.delete('1.0', tk.END)
    self.screwText.insert('1.0', "No ScrewBay data available.\n")
    self.screwText.configure(state='disabled')

def init_GUI(self, root:tk.Tk):
    '''
    Create the GUI
    '''
    self.root = root
    self.root.title("Prometheus Operator Interface")
    self.labels = {}

    labelFont = ("Segoe UI", 10)
    headerFont = ("Segoe UI", 12, "bold")

    # --- Main window layout: split left (sections) and right (buttons) ---
    outerFrame = tk.Frame(self.root)
    outerFrame.pack(fill='both', expand=True)

    # Left: content sections
    contentFrame = tk.Frame(outerFrame, padx=10, pady=10)
    contentFrame.pack(side='left', fill='both', expand=True)

    # Right: buttons
    buttonFrame = tk.Frame(outerFrame, padx=10, pady=10)
    buttonFrame.pack(side='right', fill='y')

    def makeSection(parent:tk.Frame, title:str) -> tk.LabelFrame:
        ''' 
        Create a labeled section frame.
        '''
        frame = tk.LabelFrame(parent, text=title, font=headerFont, padx=10, pady=10)
        frame.pack(fill='x', pady=5)
        return frame

    def addLabel(frame, field, row):
        '''
        Add a label to the given frame for the specified field.
        '''
        label = tk.Label(frame, text=f"{field.replace('_', ' ').capitalize()}:", font=labelFont, anchor='w')
        value = tk.Label(frame, text="N/A", font=labelFont, width=15, bg="#ddd", anchor='w')
        label.grid(row=row, column=0, sticky='w')
        value.grid(row=row, column=1, sticky='w')
        self.labels[field] = value

    # --- Section 1: Checks ---
    checksFrame = makeSection(contentFrame, "Checks")
    checksFields = [
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
    for i, field in enumerate(checksFields):
        addLabel(checksFrame, field, i)

    # --- Section 2: Positions ---
    positionsFrame = makeSection(contentFrame, "Positions")
    positionsFields = [
        'mgse_to_conveyor', 'trolley_in_bay', 'side_2_robot',
        'positioner_is_up', 'positioner_is_down',
        'pallet_is_in_wrk_pos', 'pallet_is_in_entry_pos',
        'rotary_aligned', 'holder_correction_done'
    ]
    for i, field in enumerate(positionsFields):
        addLabel(positionsFrame, field, i)

    # --- Section 3: ScrewBay ---
    screwbayFrame = makeSection(contentFrame, "ScrewBay")
    self.screwText = tk.Text(screwbayFrame, height=10, width=50, font=("Courier", 10))
    self.screwText.pack()
    self.screwText.insert('1.0', "Waiting for data...\n")
    self.screwText.configure(state='disabled')

    # --- Section 4: States (as a text box like ScrewBay) ---
    statesFrame = makeSection(contentFrame, "States")
    self.statesText = tk.Text(statesFrame, height=10, width=50, font=("Courier", 10))
    self.statesText.pack()
    self.statesText.insert('1.0', "Waiting for state data...\n")
    self.statesText.configure(state='disabled')

    # --- Button column: "Calling Block" ---
    buttonHeader = tk.Label(buttonFrame, text="Blocks", font=headerFont)
    buttonHeader.pack(pady=(0, 10))

    buildingBlocks = [
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

    for i in range(len(buildingBlocks)):  # Adjust number of blocks here
        btn = tk.Button(buttonFrame,width=40 ,text=f"Calling Block {i}: {buildingBlocks[i]}", command=lambda i=buildingBlocks[i]: self.call_block(i))
        btn.pack(pady=5)

    # --- ACtion Response Text Box ---
    response_label = tk.Label(buttonFrame, text="Service Response:", font=headerFont)
    response_label.pack(pady=(20, 5))

    self.responseText = tk.Text(buttonFrame, height=4, width=50, font=("Courier", 10))
    self.responseText.pack()
    self.responseText.insert('1.0', "Awaiting service call...\n")
    self.responseText.configure(state='disabled')

    self.initLabels()

def updateLabels(self):
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
    self.statesText.configure(state='normal')
    self.statesText.delete('1.0', tk.END)
    state_field=[
        'active_state_fsm_string',
        'active_state_mr_fsm_string',
        'active_state_sr_fsm_string',
        'active_state_conveyor_string',
        'active_state_system_safety_test'
    ]
    for field in state_field:
        val = getattr(self.state, field)
        self.statesText.insert(tk.END, f"{field}: {val}\n")
    self.statesText.configure(state='disabled')

    # Update ScrewBay Text Box
    self.screwText.configure(state='normal')
    self.screwText.delete('1.0', tk.END)
    for i, slot in enumerate(self.state.screw_bay,1):
        self.screwText.insert(tk.END, f"ScrewBay[{i}]: ")
        self.screwText.insert(tk.END, f"max=({slot.max_idx_x},{slot.max_idx_y}) ")
        self.screwText.insert(tk.END, f"next=({slot.next_idx_x},{slot.next_idx_y})\n")
    self.screwText.configure(state='disabled')
        
def updateResponseText(self, text:str, isResult:bool=False):
    '''
    Update the response text box with a new message.
    If isMsg is True, it will be treated as a message, otherwise as a status update.
    '''
    self.responseText.configure(state='normal')
    self.responseText.delete('1.0', tk.END)
    if isResult:
        if(self.functionBlockResult):
            msg = f"[Success!] Message: {text}"
        else:
            msg = f"[Fail] Message: {text}"
        self.responseText.insert('1.0', f" {msg}")
    else:
        self.responseText.insert('1.0', f"[Status] {text}")
    self.responseText.configure(state='disabled')

