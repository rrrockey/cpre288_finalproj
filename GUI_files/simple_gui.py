import tkinter as tk
import socket

# CyBot IP and port
CYBOT_IP = "192.168.1.1"
CYBOT_PORT = 288

# connect to CyBot
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((CYBOT_IP, CYBOT_PORT))

# track which buttons are being held
buttons_pressed = set()

#send keys repeatedly
def send_keys():
    for key in buttons_pressed:
        sock.send(key.encode())
    if buttons_pressed:  # keep looping if any button is held
        root.after(5, send_keys)  # repeat every 5 ms

#button press/release handlers
def button_press(key):
    buttons_pressed.add(key)
    if len(buttons_pressed) == 1:  # start loop if first button pressed
        send_keys()

def button_release(key):
    buttons_pressed.discard(key)

# GUI setup
root = tk.Tk()
root.geometry("300x300")


frame = tk.Frame(root)
frame.pack()

# W button (forward)
btn_w = tk.Button(frame, text="W", width=5, height=2)
btn_w.grid(row=0, column=1)
btn_w.bind("<ButtonPress-1>", lambda e: button_press('w'))
btn_w.bind("<ButtonRelease-1>", lambda e: button_release('w'))

# A button (left)
btn_a = tk.Button(frame, text="A", width=5, height=2)
btn_a.grid(row=1, column=0)
btn_a.bind("<ButtonPress-1>", lambda e: button_press('a'))
btn_a.bind("<ButtonRelease-1>", lambda e: button_release('a'))

# S button (backward)
btn_s = tk.Button(frame, text="S", width=5, height=2)
btn_s.grid(row=1, column=1)
btn_s.bind("<ButtonPress-1>", lambda e: button_press('s'))
btn_s.bind("<ButtonRelease-1>", lambda e: button_release('s'))

# D button (right)
btn_d = tk.Button(frame, text="D", width=5, height=2)
btn_d.grid(row=1, column=2)
btn_d.bind("<ButtonPress-1>", lambda e: button_press('d'))
btn_d.bind("<ButtonRelease-1>", lambda e: button_release('d'))

root.mainloop()
