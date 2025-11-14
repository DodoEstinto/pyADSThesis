import tkinter as tk
from tkinter import ttk
import tkinter.simpledialog as simpledialog


# Shared styling setup
def apply_dark_theme(widget):

    from tkinter import ttk

    style = ttk.Style(widget)
    style.theme_use("clam")

    bg = "#1e1e1e"
    fg = "#e0e0e0"
    accent = "#3a7afe"

    try:
        widget.configure(bg=bg)
    except tk.TclError:
        pass

    # --- Base ---
    style.configure("TFrame", background=bg)
    style.configure("TLabel", background=bg, foreground=fg)
    style.configure("TCheckbutton", background=bg, foreground=fg)

    # --- Entries ---
    style.configure("Dark.TEntry",
                    fieldbackground="#252526",
                    foreground=fg,
                    insertcolor=fg,
                    relief="flat",
                    bordercolor="#1e1e1e")
    style.configure("Readonly.TEntry",
                    fieldbackground="#333333",
                    foreground=fg,
                    relief="flat",
                    bordercolor="#1e1e1e")

    # --- Buttons ---
    style.configure("TButton",
                    background=accent,
                    foreground="white",
                    font=("Segoe UI", 10, "bold"),
                    padding=6,
                    borderwidth=0)
    style.map("TButton",
              background=[("active", "#1d4ed8"), ("disabled", "#3b3b3b")])

    # --- Scrollbars ---
    style.configure("Vertical.TScrollbar",
                    background="#2e2e2e",
                    troughcolor=bg,
                    arrowcolor=fg)
    style.configure("Horizontal.TScrollbar",
                    background="#2e2e2e",
                    troughcolor=bg,
                    arrowcolor=fg)


# ---------- YES/NO/CANCEL DIALOG ----------
class YesNoCancelDialog(tk.Toplevel):
    def __init__(self, parent, title, message):
        super().__init__(parent)
        apply_dark_theme(self)
        self.result = None
        self.title(title)
        self.geometry("450x160")
        self.resizable(False, False)

        frame = ttk.Frame(self, padding=20)
        frame.pack(expand=True, fill="both")

        label = ttk.Label(frame, text=message, wraplength=380, justify="center", font=("Segoe UI", 11))
        label.pack(pady=10)

        button_frame = ttk.Frame(frame)
        button_frame.pack(pady=10)

        yes_button = ttk.Button(button_frame, text="Yes", command=self.on_yes)
        yes_button.pack(side="left", padx=10)

        no_button = ttk.Button(button_frame, text="No", command=self.on_no)
        no_button.pack(side="left", padx=10)

        cancel_button = ttk.Button(button_frame, text="Cancel", command=self.on_cancel)
        cancel_button.pack(side="left", padx=10)

        self.grab_set()
        self.transient(parent)
        self.wait_window()

    def on_yes(self):
        self.result = True
        self.destroy()

    def on_no(self):
        self.result = False
        self.destroy()

    def on_cancel(self):
        self.result = None
        self.destroy()

# --------- ASK INTEGER DIALOG ----------

class AskIntegerDialog(simpledialog.Dialog):
    def __init__(self, parent, title="Input Required", prompt="Please enter an integer:", minvalue=None, maxvalue=None):
        self.prompt = prompt
        self.minvalue = minvalue
        self.maxvalue = maxvalue
        self.value = None
        super().__init__(parent, title)

    def body(self, master):
        apply_dark_theme(master)
        ttk.Label(master, text=self.prompt).pack(pady=10)

        self.entry = ttk.Entry(master, width=20, style="Dark.TEntry",
                               font=("Consolas", 10), justify="center")
        self.entry.pack(pady=5)
        return self.entry  # initial focus

    def apply(self):
        self.result = self.value

            
    def buttonbox(self):
        apply_dark_theme(self)

        box = ttk.Frame(self)
        box.pack(side="bottom", fill="x", pady=15)

        btn_frame = ttk.Frame(box)
        btn_frame.pack(anchor="center")

        btn_ok = ttk.Button(btn_frame, text="OK", command=self.check_ok)
        btn_ok.pack(side="left", padx=10)

        btn_cancel = ttk.Button(btn_frame, text="Cancel", command=self.cancel)
        btn_cancel.pack(side="left", padx=10)

        self.bind("<Return>", self.ok)
        self.bind("<Escape>", self.cancel)    

    def check_ok(self):
        try:
            self.value = int(self.entry.get())
            if(self.minvalue is not None):
                if(self.value < self.minvalue):
                    raise ValueError("Value below minimum")
            if(self.maxvalue is not None):
                if(self.value > self.maxvalue):
                    raise ValueError("Value above maximum")
            self.ok()
        except ValueError:
            #get actual background value
            bg = self.entry.cget("background")
            #flash red background
            self.entry.delete(0, tk.END)
            self.entry.insert(0, "Invalid input")


            self.result = None
        
 

# ---------- OK DIALOG ----------
class OkDialog(tk.Toplevel):
    def __init__(self, parent, title="Message", message=""):
        super().__init__(parent)
        apply_dark_theme(self)
        self.result = False
        self.title(title)
        self.geometry("450x200")
        self.resizable(False, False)

        frame = ttk.Frame(self, padding=20)
        frame.pack(expand=True, fill="both")

        label = ttk.Label(frame, text=message, wraplength=380, justify="center", font=("Segoe UI", 11))
        label.pack(pady=10)

        ok_button = ttk.Button(frame, text="OK", command=self.on_ok)
        ok_button.pack(pady=10)

        self.grab_set()
        self.transient(parent)
        self.wait_window()

    def on_ok(self):
        self.result = True
        self.destroy()


# ---------- SCREW DIALOG ----------
class ScrewDialog(simpledialog.Dialog):
    def body(self, master):
        apply_dark_theme(master)
        ttk.Label(master, text="Screw X:").grid(row=0, column=0, sticky="w")
        ttk.Label(master, text="Screw Y:").grid(row=1, column=0, sticky="w")
        ttk.Label(master, text="Screw Z:").grid(row=2, column=0, sticky="w")
        ttk.Label(master, text="Screw Area:").grid(row=3, column=0, sticky="w")
        ttk.Label(master, text="Target to Use:").grid(row=4, column=0, sticky="w")
        ttk.Label(master, text="Focal Plane:").grid(row=5, column=0, sticky="w")
        ttk.Label(master, text="Screw Recipe ID:").grid(row=6, column=0, sticky="w")

        self.entry_x = ttk.Entry(master, width=10)
        self.entry_y = ttk.Entry(master, width=10)
        self.entry_z = ttk.Entry(master, width=10)
        self.entry_target = ttk.Entry(master, width=10)
        self.entry_focal_plane = ttk.Entry(master, width=10)
        self.entry_screw_recipe_id = ttk.Entry(master, width=10)

        self.entry_x.grid(row=0, column=1, padx=5, pady=2)
        self.entry_y.grid(row=1, column=1, padx=5, pady=2)
        self.entry_z.grid(row=2, column=1, padx=5, pady=2)
        self.entry_target.grid(row=4, column=1, padx=5, pady=2)
        self.entry_focal_plane.grid(row=5, column=1, padx=5, pady=2)
        self.entry_screw_recipe_id.grid(row=6, column=1, padx=5, pady=2)

        self.area_var = tk.StringVar(value="inside")

        # Dark OptionMenu styling
        option_style = ttk.Style()
        option_style.configure(
            "Dark.TMenubutton",
            background="#252526",
            foreground="#e0e0e0",
            arrowcolor="#e0e0e0",
            font=("Segoe UI", 10),
            padding=6
        )
        option_style.map("Dark.TMenubutton",
                         background=[("active", "#3a7afe")],
                         foreground=[("active", "#ffffff")])

        self.option_menu = ttk.OptionMenu(master, self.area_var, "inside","inside","outside")
        self.option_menu.grid(row=3, column=1, padx=5, pady=2)
        self.option_menu.configure(style="Dark.TMenubutton")

        # Make dropdown menu dark too
        menu = self.option_menu.nametowidget(self.option_menu["menu"])
        menu.configure(bg="#252526", fg="#e0e0e0", activebackground="#3a7afe", activeforeground="white", borderwidth=0)


        return self.entry_x  # focus

    def apply(self):
        try:
            self.result = {
                "screwX": float(self.entry_x.get()),
                "screwY": float(self.entry_y.get()),
                "screwZ": float(self.entry_z.get()),
                "screwArea": self.area_var.get(),
                "targetToUse": int(self.entry_target.get()),
                "focalPlane": int(self.entry_focal_plane.get()),
                "screwRecipeID": int(self.entry_screw_recipe_id.get())
            }
        except ValueError:
            self.result = None

    def buttonbox(self):
            apply_dark_theme(self)

            box = ttk.Frame(self)
            box.pack(side="bottom", fill="x", pady=15)

            btn_frame = ttk.Frame(box)
            btn_frame.pack(anchor="center")

            btn_ok = ttk.Button(btn_frame, text="OK", command=self.ok)
            btn_ok.pack(side="left", padx=10)

            btn_cancel = ttk.Button(btn_frame, text="Cancel", command=self.cancel)
            btn_cancel.pack(side="left", padx=10)

            self.bind("<Return>", self.ok)
            self.bind("<Escape>", self.cancel)


# ---------- SCREW BAY DIALOG ----------
class ScrewBayDialog(simpledialog.Dialog):
    def __init__(self, parent, screw_bays=None, num_slots=6, title="Screw Bay Editor"):
        self.num_slots = num_slots
        self.entries = []
        self.screw_bays = screw_bays or []  # default values from your state
        super().__init__(parent, title)

    def body(self, master):
        apply_dark_theme(master)
        master.configure(bg="#1e1e1e")

        # Column headers
        header_font = ("Segoe UI", 10, "bold")
        labels = ["ScrewBay", "Max X", "Max Y", "Next X", "Next Y", "Edit Max?"]
        for j, text in enumerate(labels):
            tk.Label(master, text=text, bg="#1e1e1e", fg="#ffffff", font=header_font).grid(row=0, column=j, padx=5, pady=4)

        # Rows
        for i in range(self.num_slots):
            tk.Label(master, text=f"{i+1}", bg="#1e1e1e", fg="#ffffff").grid(row=i+1, column=0, padx=5, pady=3)

            entry_x = ttk.Entry(master, width=6, style="Readonly.TEntry",
                                font=("Consolas", 10), justify="center")
            entry_y = ttk.Entry(master, width=6, style="Readonly.TEntry",
                                font=("Consolas", 10), justify="center")
            next_x = ttk.Entry(master, width=6, style="Dark.TEntry",
                            font=("Consolas", 10), justify="center")
            next_y = ttk.Entry(master, width=6, style="Dark.TEntry",
                            font=("Consolas", 10), justify="center")


            # Pre-fill values
            if i < len(self.screw_bays):
                slot = self.screw_bays[i]
                entry_x.insert(0, slot.max_idx_x)
                entry_y.insert(0, slot.max_idx_y)
                next_x.insert(0, slot.next_idx_x)
                next_y.insert(0, slot.next_idx_y)

            # Lock Max X/Y initially
            entry_x.config(state="readonly")
            entry_y.config(state="readonly")

            entry_x.grid(row=i+1, column=1, padx=5)
            entry_y.grid(row=i+1, column=2, padx=5)
            next_x.grid(row=i+1, column=3, padx=5)
            next_y.grid(row=i+1, column=4, padx=5)

            # Checkbox for editing Max
            edit_var = tk.IntVar(value=0)
            checkbox = tk.Checkbutton(master, variable=edit_var, bg="#1e1e1e",
                                      activebackground="#1e1e1e", highlightthickness=0,
                                      command=lambda v=edit_var, e_x=entry_x, e_y=entry_y:
                                          self.toggle_edit(v, e_x, e_y))
            checkbox.grid(row=i+1, column=5)
            self.entries.append((entry_x, entry_y, next_x, next_y, edit_var))

        return self.entries[0][2]  # focus on first next_x entry
    
    def toggle_edit(self, var, e_x, e_y):
        if var.get():  # unlocked
            e_x.config(state="normal", style="Dark.TEntry")
            e_y.config(state="normal", style="Dark.TEntry")
        else:  # readOnly
            e_x.config(state="readonly", style="Readonly.TEntry")
            e_y.config(state="readonly", style="Readonly.TEntry")


    def buttonbox(self):
            apply_dark_theme(self)

            box = ttk.Frame(self)
            box.pack(side="bottom", fill="x", pady=15)

            btn_frame = ttk.Frame(box)
            btn_frame.pack(anchor="center")

            btn_ok = ttk.Button(btn_frame, text="OK", command=self.ok)
            btn_ok.pack(side="left", padx=10)

            btn_cancel = ttk.Button(btn_frame, text="Cancel", command=self.cancel)
            btn_cancel.pack(side="left", padx=10)

            self.bind("<Return>", self.ok)
            self.bind("<Escape>", self.cancel)


    def apply(self):
        self.result = []
        for entry_x, entry_y, next_x, next_y, _ in self.entries:
            try:
                self.result.append({
                    "MAX_IDX_X": int(entry_x.get()),
                    "MAX_IDX_Y": int(entry_y.get()),
                    "nextIdxX": int(next_x.get()),
                    "nextIdxY": int(next_y.get())
                })
            except ValueError:
                self.result.append(None)

# ---------- SEQUENCE DIALOG ----------
class SequenceDialog(simpledialog.Dialog):
    def __init__(self, parent, title="Sequence Editor", options=[]):
        self.options = options
        super().__init__(parent, title)

    def body(self, root):
        apply_dark_theme(root)

        main_frame = ttk.Frame(root, padding=10)
        main_frame.pack(fill="both", expand=True)

        # --- Left: listbox ---
        frame_left = ttk.Frame(main_frame)
        frame_left.pack(side="left", fill="both", expand=True, padx=5, pady=5)

        scrollbar = ttk.Scrollbar(frame_left)
        scrollbar.pack(side="right", fill="y")

        self.listbox = tk.Listbox(
            frame_left,
            yscrollcommand=scrollbar.set,
            width=45, height=20,
            bg="#252526", fg="#e0e0e0",
            selectbackground="#3a7afe", selectforeground="white",
            highlightthickness=0, relief="flat",
            borderwidth=0, font=("Consolas", 10)
        )
        self.listbox.pack(side="left", fill="both", expand=True, padx=2, pady=2)
        scrollbar.config(command=self.listbox.yview)

        # --- Right: dropdown + add ---
        frame_right = ttk.Frame(main_frame)
        frame_right.pack(side="right", fill="y", padx=5, pady=5)

        ttk.Label(frame_right, text="Select an option:").pack(pady=5)

        self.selected = tk.StringVar(value=self.options[0] if self.options else "")

        # Dark OptionMenu styling
        option_style = ttk.Style()
        option_style.configure(
            "Dark.TMenubutton",
            background="#252526",
            foreground="#e0e0e0",
            arrowcolor="#e0e0e0",
            font=("Segoe UI", 10),
            padding=6
        )
        option_style.map("Dark.TMenubutton",
                         background=[("active", "#3a7afe")],
                         foreground=[("active", "#ffffff")])

        self.dropdown = ttk.OptionMenu(frame_right, self.selected, self.selected.get(), *self.options)
        self.dropdown.configure(style="Dark.TMenubutton")
        self.dropdown.pack(pady=5, fill="x")

        # Make dropdown menu dark too
        menu = self.dropdown.nametowidget(self.dropdown["menu"])
        menu.configure(bg="#252526", fg="#e0e0e0", activebackground="#3a7afe", activeforeground="white", borderwidth=0)

        ttk.Button(frame_right, text="Add", command=self.add_to_list).pack(pady=5)

    def add_to_list(self):
        item = self.selected.get()
        if item:
            self.listbox.insert("end", item)

    def buttonbox(self):
        apply_dark_theme(self)

        box = ttk.Frame(self)
        box.pack(side="bottom", fill="x", pady=15)

        btn_frame = ttk.Frame(box)
        btn_frame.pack(anchor="center")

        btn_ok = ttk.Button(btn_frame, text="OK", command=self.ok)
        btn_ok.pack(side="left", padx=10)

        btn_cancel = ttk.Button(btn_frame, text="Cancel", command=self.cancel)
        btn_cancel.pack(side="left", padx=10)

        self.bind("<Return>", self.ok)
        self.bind("<Escape>", self.cancel)

    def apply(self):
        self.result = self.listbox.get(0, tk.END)
