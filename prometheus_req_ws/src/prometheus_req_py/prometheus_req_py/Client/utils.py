import tkinter as tk
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
