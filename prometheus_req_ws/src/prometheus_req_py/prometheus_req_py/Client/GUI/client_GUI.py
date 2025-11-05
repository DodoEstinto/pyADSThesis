import tkinter as tk
from tkinter import ttk

def initLabels(self):
    '''
    Initialize all labels to "N/A" and neutral background.
    '''
    for field, label in self.labels.items():
        label.config(text="N/A")

    # Reset ScrewBay text
    self.screwText.configure(state='normal')
    self.screwText.delete('1.0', tk.END)
    self.screwText.insert('1.0', "No ScrewBay data available.\n")
    self.screwText.configure(state='disabled')

def init_GUI(self, root: tk.Tk):
    '''
    Create GUI with dark modern style and white-bordered frames including titles inside.
    '''
    self.root = root
    self.root.title("Prometheus Operator Interface")
    self.labels = {}

    # ========== STYLING ==========
    style = ttk.Style()
    self.root.configure(bg="#1e1e1e")

    style.theme_use("clam")

    # Generale
    style.configure("TFrame", background="#1e1e1e")
    style.configure("TLabel", background="#1e1e1e", foreground="#ffffff", font=("Segoe UI", 10))
    style.configure("TButton", background="#3a7afe", foreground="white",
                    font=("Segoe UI", 10, "bold"), padding=6)
    style.map("TButton",
        background=[("active", "#1d4ed8"), ("disabled", "#3b3b3b")],
        relief=[("pressed", "sunken"), ("!pressed", "flat")]
    )

    # Frame con bordo bianco
    style.configure("WhiteBorder.TFrame", background="#1e1e1e")  # sfondo scuro, bordo visibile tramite relief

    # ========== MAIN LAYOUT ==========
    outerFrame = ttk.Frame(self.root, padding=10)
    outerFrame.pack(fill='both', expand=True)

    contentFrame = ttk.Frame(outerFrame, padding=(10, 0))
    contentFrame.pack(side='left', fill='both', expand=True)

    buttonFrame = ttk.Frame(outerFrame, padding=10)
    buttonFrame.pack(side='right', fill='y')

    # ---------- Helper: Section ----------
    def makeSection(parent, title: str) -> ttk.Frame:
        # Frame esterno con padding
        outer = ttk.Frame(parent, style="WhiteBorder.TFrame", padding=5)
        outer.pack(fill='x', pady=8)

        # Frame interno con bordino bianco
        inner = ttk.Frame(outer, padding=10, borderwidth=1, relief="solid", style="WhiteBorder.TFrame")
        inner.pack(fill='x')

        # Titolo all'interno del bordo
        title_label = ttk.Label(inner, text=title, font=("Segoe UI", 12, "bold"), background="#1e1e1e", foreground="white")
        title_label.pack(anchor='w', pady=(0,10))

        return inner

    def addLabel(frame, field):
        label_frame = ttk.Frame(frame, style="WhiteBorder.TFrame")
        label_frame.pack(fill='x', pady=2)
        label = ttk.Label(label_frame, text=f"{field.replace('_', ' ').capitalize()}:")
        label.pack(side='left', padx=(0,5))
        value = ttk.Label(label_frame, text="N/A", width=18, anchor='w', background="#1e1e1e", foreground="white")
        value.pack(side='left')
        self.labels[field] = value

    # ---------- Sections ----------
    checksFields = [
        'em_general','em_mr','em_sr','tem_sens_ok','air_press_ok',
        'em_laser_scanner','em_laser_scanner2','em_laser_scanner3','automatic_mode'
    ]
    checksFrame = makeSection(contentFrame, "Checks")
    for f in checksFields: addLabel(checksFrame, f)

    positionsFields = [
        'mgse_to_conveyor','trolley_in_bay','side_2_robot','positioner_is_up',
        'positioner_is_down','pallet_is_in_wrk_pos','pallet_is_in_entry_pos',
        'rotary_aligned','holder_correction_done'
    ]
    positionsFrame = makeSection(contentFrame, "Positions")
    for f in positionsFields: addLabel(positionsFrame, f)

    screwbayFrame = makeSection(contentFrame, "ScrewBay")
    self.screwText = tk.Text(screwbayFrame, height=10, width=50, font=("Consolas", 10),
                             bg="#252526", fg="#e0e0e0", insertbackground="white", relief="flat", wrap='none')
    self.screwText.pack(fill='x')
    self.screwText.insert('1.0', "Waiting for data...\n")
    self.screwText.configure(state='disabled')

    statesFrame = makeSection(contentFrame, "States")
    self.statesText = tk.Text(statesFrame, height=10, width=50, font=("Consolas", 10),
                              bg="#252526", fg="#e0e0e0", insertbackground="white", relief="flat", wrap='none')
    self.statesText.pack(fill='x')
    self.statesText.insert('1.0', "Waiting for state data...\n")
    self.statesText.configure(state='disabled')

    # ---------- Buttons ----------
    ttk.Label(buttonFrame, text="Blocks", font=("Segoe UI", 12, "bold"),
              background="#1e1e1e", foreground="#ffffff").pack(pady=(0,10))

    def makeButton(text, cmd):
        btn = ttk.Button(buttonFrame, text=text, command=cmd, width=40)
        btn.pack(pady=4)
        return btn

    buildingBlocks = [
        "loadTray","pickUpTray","depositTray","srHoming","mrHoming","mrTrolleyVCheck",
        "gyroGrpRot","screwTight","screwPickup","presentToScrew","positionerRotate",
        "stackTray","present2Op","setScrewBayState"
    ]
    for block in buildingBlocks:
        makeButton(block, lambda b=block: self.call_block(b))

    makeButton("Sequence", self.start_sequence)

    # ---------- Response ----------
    ttk.Label(buttonFrame, text="Service Response:", font=("Segoe UI", 12, "bold"),
              background="#1e1e1e", foreground="#ffffff").pack(pady=(20,5))

    self.responseText = tk.Text(buttonFrame, height=4, width=50, font=("Consolas", 10),
                                bg="#252526", fg="#e0e0e0", insertbackground="white", relief="flat", wrap='word')
    self.responseText.pack()
    self.responseText.insert('1.0', "Awaiting service call...\n")
    self.responseText.configure(state='disabled')

    self.initLabels()

def updateLabels(self):
    '''
    Update labels with color coding for True/False.
    '''
    def color(val):
        if val is True:
            return "#155724"  # green
        elif val is False:
            return "#7f1d1d"  # red
        return "#3c3c3c"     # neutral

    all_fields = [
        'em_general','em_mr','em_sr','tem_sens_ok','air_press_ok',
        'em_laser_scanner','em_laser_scanner2','em_laser_scanner3',
        'automatic_mode','mgse_to_conveyor','trolley_in_bay','side_2_robot',
        'positioner_is_up','positioner_is_down','pallet_is_in_wrk_pos',
        'pallet_is_in_entry_pos','rotary_aligned','holder_correction_done'
    ]

    for field in all_fields:
        val = getattr(self.state, field)
        self.labels[field].config(text=str(val), background=color(val))

    # Update state text
    self.statesText.configure(state='normal')
    self.statesText.delete('1.0', tk.END)
    for f in ['active_state_fsm_string','active_state_mr_fsm_string',
              'active_state_sr_fsm_string','active_state_conveyor_string',
              'active_state_system_safety_test']:
        self.statesText.insert(tk.END, f"{f}: {getattr(self.state, f)}\n")
    self.statesText.configure(state='disabled')

    # Update ScrewBay
    self.screwText.configure(state='normal')
    self.screwText.delete('1.0', tk.END)
    for i, slot in enumerate(self.state.screw_bay, 1):
        self.screwText.insert(tk.END, f"ScrewBay[{i}]: max=({slot.max_idx_x},{slot.max_idx_y}) next=({slot.next_idx_x},{slot.next_idx_y})\n")
    self.screwText.configure(state='disabled')


def updateResponseText(self, text: str, isResult: bool=False):
    self.responseText.configure(state='normal')
    self.responseText.delete('1.0', tk.END)
    if isResult:
        color = "#4caf50" if self.functionBlockResult else "#f44336"
        msg = f"[Success] {text}" if self.functionBlockResult else f"[Fail] {text}"
        self.responseText.insert('1.0', msg)
        self.responseText.tag_add("status", "1.0", "end")
        self.responseText.tag_config("status", foreground=color)
    else:
        self.responseText.insert('1.0', f"[Status] {text}")
        self.responseText.tag_add("status", "1.0", "end")
        self.responseText.tag_config("status", foreground="#ffffff")
    self.responseText.configure(state='disabled')
