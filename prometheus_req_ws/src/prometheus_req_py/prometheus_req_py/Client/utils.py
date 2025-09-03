import tkinter as tk
import tkinter.simpledialog as simpledialog
class OkDialog(tk.Toplevel):
    def __init__(self, parent, title="Message", message=""):
        super().__init__(parent)
        self.result = False  
        self.title(title)
        self.geometry("450x160")
        self.resizable(False, False)

        self.label = tk.Label(self, text=message, wraplength=280)
        self.label.pack(pady=20)

        self.ok_button = tk.Button(self, text="OK", width=10, command=self.on_ok)
        self.ok_button.pack(pady=(0, 10))

        self.grab_set()     # Make this window modal
        self.transient(parent)  # Show on top of parent
        self.wait_window()  # Wait for this window to close

    def on_ok(self):
        self.result = True
        self.destroy()


class ScrewDialog(simpledialog.Dialog):
    def body(self, master):
        tk.Label(master, text="Screw X:").grid(row=0, column=0, sticky="w")
        tk.Label(master, text="Screw Y:").grid(row=1, column=0, sticky="w")
        tk.Label(master, text="Screw Z:").grid(row=2, column=0, sticky="w")
        tk.Label(master, text="Screw Area:").grid(row=3, column=0, sticky="w")
        tk.Label(master, text="Target to Use:").grid(row=4, column=0, sticky="w")
        tk.Label(master, text="Focal Plane:").grid(row=5, column=0, sticky="w")
        tk.Label(master, text="Screw Recipe ID:").grid(row=6, column=0, sticky="w")

        self.entry_target = tk.Entry(master)
        self.entry_target.grid(row=4, column=1)

        self.entry_x = tk.Entry(master)
        self.entry_y = tk.Entry(master)
        self.entry_z = tk.Entry(master)
        self.entry_target = tk.Entry(master)
        self.entry_focal_plane = tk.Entry(master)
        self.entry_screw_recipe_id = tk.Entry(master)

        self.entry_x.grid(row=0, column=1)
        self.entry_y.grid(row=1, column=1)
        self.entry_z.grid(row=2, column=1)
        self.entry_target.grid(row=4, column=1)
        self.entry_focal_plane.grid(row=5, column=1)
        self.entry_screw_recipe_id.grid(row=6, column=1)

        self.area_var = tk.StringVar(value="inside")
        self.option_menu = tk.OptionMenu(master, self.area_var, "inside", "outside")
        self.option_menu.grid(row=3, column=1)

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

class ScrewBayDialog(simpledialog.Dialog):
    def __init__(self, parent, screw_bays=None, num_slots=6, title="Screw Bay Editor"):
        self.num_slots = num_slots
        self.entries = []
        self.screw_bays = screw_bays or []  # default values from your state
        super().__init__(parent, title)

    def body(self, master):
        # Column headers
        tk.Label(master, text="ScrewBay").grid(row=0, column=0)
        tk.Label(master, text="Max X").grid(row=0, column=1)
        tk.Label(master, text="Max Y").grid(row=0, column=2)
        tk.Label(master, text="Next X").grid(row=0, column=3)
        tk.Label(master, text="Next Y").grid(row=0, column=4)
        tk.Label(master, text="Edit Max?").grid(row=0, column=5)  # checkbox column

        for i in range(self.num_slots):
            tk.Label(master, text=f"{i+1}").grid(row=i+1, column=0)

            # Entry widgets
            entry_x = tk.Entry(master, width=5)
            entry_y = tk.Entry(master, width=5)
            next_x = tk.Entry(master, width=5)
            next_y = tk.Entry(master, width=5)

            # Pre-fill values
            if i < len(self.screw_bays):
                slot = self.screw_bays[i]
                entry_x.insert(0, slot.max_idx_x)
                entry_y.insert(0, slot.max_idx_y)
                next_x.insert(0, slot.next_idx_x)
                next_y.insert(0, slot.next_idx_y)

            # Make Max X/Y read-only initially
            entry_x.config(state="readonly")
            entry_y.config(state="readonly")

            # Grid placement
            entry_x.grid(row=i+1, column=1)
            entry_y.grid(row=i+1, column=2)
            next_x.grid(row=i+1, column=3)
            next_y.grid(row=i+1, column=4)

            # Checkbox to unlock Max X/Y
            edit_var = tk.IntVar(value=0)
            def toggle_edit(var=edit_var, e_x=entry_x, e_y=entry_y):
                state = "normal" if var.get() else "readonly"
                e_x.config(state=state)
                e_y.config(state=state)
            checkbox = tk.Checkbutton(master, variable=edit_var, command=toggle_edit)
            checkbox.grid(row=i+1, column=5)

            self.entries.append((entry_x, entry_y, next_x, next_y, edit_var))
        
        return self.entries[0][2]  # focus on first Next X field



    def apply(self):
        self.result = []
        for entry_x, entry_y, next_x, next_y, _ in self.entries:
            try:
                slot = {
                    "MAX_IDX_X": int(entry_x.get()),
                    "MAX_IDX_Y": int(entry_y.get()),
                    "nextIdxX": int(next_x.get()),
                    "nextIdxY": int(next_y.get())
                }
                self.result.append(slot)
            except ValueError:
                self.result.append(None)  # handle invalid entries