import tkinter as tk
from tkinter import scrolledtext
import serial
import threading
import time
from datetime import datetime

# === Serial Config ===
PORT = 'COM4'  # Update this with your Arduino's COM port
BAUD = 9600

# === Serial Reader Thread ===
def read_serial(log_widgets):
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)

        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            timestamp = datetime.now().strftime("[%H:%M:%S] ")
            entry = timestamp + line + '\n'

            if   line.startswith("[POS]"):      write_colored(log_widgets["pos"],     entry, "green")
            elif line.startswith("[STATE]"):    write_colored(log_widgets["state"],   entry, "blue")
            elif line.startswith("[ULTRA]"):    write_colored(log_widgets["ultra"],   entry, "purple")
            elif line.startswith("[INCLINE]"):  write_colored(log_widgets["incline"], entry, "red")
            elif line.startswith("[CURRENT]"):  write_colored(log_widgets["current"], entry, "orange")
            # all other lines are dropped

    except Exception as e:
        write_colored(log_widgets["state"], f"[ERROR] {e}\n", "black")

# === Helper Function for Colored Text ===
def write_colored(widget, text, color):
    widget.configure(state='normal')
    widget.insert(tk.END, text)
    widget.tag_add(color, "end-1l", "end-1c")
    widget.tag_config(color, foreground=color)
    widget.see(tk.END)
    widget.configure(state='disabled')

# === GUI Setup ===
root = tk.Tk()
root.title("Door Control Serial Monitor")

# Make the window fullscreen
root.attributes("-fullscreen", True)
# Allow exit from fullscreen with Esc
root.bind("<Escape>", lambda e: root.attributes("-fullscreen", False))

# Add in-GUI title
title_label = tk.Label(
    root,
    text="STABILUS_POWER Door Control Serial Monitor",
    font=('Segoe UI', 18, 'bold'),
    pady=10
)
title_label.grid(row=0, column=0, columnspan=3)

# === Create Labeled Scrollable Sections ===
def create_log_frame(parent, title, row, column, label_color):
    outer_frame = tk.Frame(
        parent,
        bg="#f0f0f0",
        highlightbackground="black",
        highlightthickness=2
    )
    outer_frame.grid(row=row, column=column, padx=10, pady=10, sticky="nsew")

    label = tk.Label(
        outer_frame,
        text=title,
        font=('Segoe UI', 11, 'bold'),
        bg="#f0f0f0",
        fg=label_color,
        anchor="w"
    )
    label.pack(fill='x', padx=8, pady=(6, 2))

    log = scrolledtext.ScrolledText(
        outer_frame,
        height=10,
        font=('Consolas', 10),
        bg="white",
        fg="black",
        relief="flat",
        state='disabled'
    )
    log.pack(expand=True, fill='both', padx=8, pady=(0, 8))

    return log

# === Grid Layout ===
root.grid_rowconfigure(1, weight=1)
root.grid_rowconfigure(2, weight=1)
for j in range(3):
    root.grid_columnconfigure(j, weight=1)

widgets = {
    "pos":     create_log_frame(root, "[POS] Encoder Position",       1, 0, "green"),
    "state":   create_log_frame(root, "[STATE] Door State",           1, 1, "blue"),
    "ultra":   create_log_frame(root, "[ULTRA] Ultrasonic Distance",  1, 2, "purple"),
    "incline": create_log_frame(root, "[INCLINE] Inclinometer",      2, 0, "red"),
    "current": create_log_frame(root, "[CURRENT] Motor Current (A)", 2, 1, "orange"),
}

threading.Thread(target=read_serial, args=(widgets,), daemon=True).start()
root.mainloop()
