import tkinter as tk
import tkinter.simpledialog as simpledialog
class OkDialog(tk.Toplevel):
    def __init__(self, parent, title="Message", message=""):
        super().__init__(parent)
        self.result = False  
        self.title(title)
        self.geometry("300x120")
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
