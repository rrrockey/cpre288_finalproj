import tkinter as tk
from tkinter import scrolledtext
import socket
import math

# Configuration
HOST = "192.168.1.1" 
PORT = 288
CANVAS_W = 900
CANVAS_H = 600
SCALE = 1.5 # pixels per cm
GRID_SIZE = 50 # cm
MAX_X_CM = 450 # max horizontal
MAX_Y_CM = 300 # max vertical
OFFSET = 17.5 # Bot radius

class CyBotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("CyBot GUI")
        
        # Bot State
        self.x = OFFSET
        self.y = OFFSET
        self.dir = 0 # 0=xpos(right), 1=ypos(up), 2=xneg(left), 3=yneg(down)
        self.scan_data = []
        self.scanning = False
        self.sock = None
        self.cybot = None

        # UI Setup
        top = tk.Frame(root, pady=5)
        top.pack()
        self.btn_conn = tk.Button(top, text="Connect", command=self.connect, bg="#ddd")
        self.btn_conn.pack(side=tk.LEFT)

        main = tk.Frame(root)
        main.pack(fill=tk.BOTH, expand=True, padx=10)
        
        left = tk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.Y)
        
        tk.Label(left, text="Manual Control").pack(pady=(10,0))
        ctrl = tk.Frame(left)
        ctrl.pack(pady=5)
        
        tk.Button(ctrl, text="^", command=lambda: self.send('w'), bg="lime", width=5).grid(row=0, column=1)
        tk.Button(ctrl, text="<", command=lambda: self.send('a'), bg="green", width=5).grid(row=1, column=0)
        tk.Button(ctrl, text="v", command=lambda: self.send('s'), bg="green", width=5).grid(row=1, column=1)
        tk.Button(ctrl, text=">", command=lambda: self.send('d'), bg="green", width=5).grid(row=1, column=2)
        tk.Button(ctrl, text="Scan", command=lambda: self.send('h'), bg="cyan", width=15).grid(row=2, column=0, columnspan=3, pady=5)
        
        tk.Label(left, text="Autonomous").pack(pady=(10,0))
        tk.Button(left, text="Start Auto (m)", command=lambda: self.send('m'), bg="orange", width=20).pack()

        tk.Label(left, text="Log").pack(pady=(10,0))
        self.log_box = scrolledtext.ScrolledText(left, width=30, height=15, state='disabled')
        self.log_box.pack(fill=tk.Y, expand=True)

        self.canvas = tk.Canvas(main, bg="white", width=CANVAS_W, height=CANVAS_H)
        self.canvas.pack(side=tk.RIGHT, padx=10, pady=10)
        
        self.draw_grid()
        self.bot_dot = self.canvas.create_oval(0,0,0,0, fill="blue")
        self.update_bot_pos()

    def draw_grid(self):
        #grid lines in canvas
        for x in range(0, int(MAX_X_CM) + 1, GRID_SIZE):
            x0, y0 = self.to_screen(x, 0)
            x1, y1 = self.to_screen(x, MAX_Y_CM)
            self.canvas.create_line(x0, y0, x1, y1, fill="#eee")
        for y in range(0, int(MAX_Y_CM) + 1, GRID_SIZE):
            x0, y0 = self.to_screen(0, y)
            x1, y1 = self.to_screen(MAX_X_CM, y)
            self.canvas.create_line(x0, y0, x1, y1, fill="#eee")
        
        x0, y0 = self.to_screen(0, 0)
        x1, y1 = self.to_screen(MAX_X_CM, MAX_Y_CM)
        self.canvas.create_rectangle(x0, y0, x1, y1, outline="gray", width=2)

    def connect(self):
        #connect to cybot
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((HOST, PORT))
            self.cybot = self.sock.makefile("rbw", buffering=0)
            self.sock.setblocking(False)
            self.btn_conn.config(text="Connected", bg="lime", state="disabled")
            self.log("Connected!")
            self.root.after(100, self.poll_data)
        except Exception as e:
            self.log(f"Error: {e}")

    def send(self, char):
        #send command to cybot
        if self.cybot:
            try:
                self.cybot.write(char.encode())
                self.log(f"Sent: {char}")
            except: pass

    def poll_data(self):
        #poll data
        if not self.cybot: return
        try:
            line = self.cybot.readline()
            if line:
                decoded = line.decode('utf-8', errors='ignore').strip()
                if decoded: self.parse(decoded)
        except: pass
        self.root.after(50, self.poll_data)

    def parse(self, line):
        #split and interpret data
        self.log(f"Rx: {line}")
        parts = line.replace(',', ' ').split()
        if not parts: return
        cmd = parts[0].upper()
        
        try:
            if cmd == "MOVE": # MOVE x y dir dist
                x, y, d, dist = map(float, parts[1:])
                # Draw line from old pos to new pos
                sx1, sy1 = self.to_screen(self.x, self.y)
                self.x, self.y, self.dir = x + OFFSET, y + OFFSET, int(d)
                sx2, sy2 = self.to_screen(self.x, self.y)
                self.canvas.create_line(sx1, sy1, sx2, sy2, fill="blue", width=2, arrow=tk.LAST)
                self.update_bot_pos()
                
            elif cmd == "TURN": # TURN start end
                s, e = map(int, parts[1:])
                self.dir = e
                
            elif cmd == "SCAN": # SCAN x y dir
                x, y, d = map(float, parts[1:])
                self.x, self.y, self.dir = x + OFFSET, y + OFFSET, int(d)
                self.update_bot_pos()
                self.scan_data = []
                self.scanning = True
                
            elif cmd == "DATA" and self.scanning: # DATA angle ir ping
                a, ir, ping = map(float, parts[1:])
                self.scan_data.append({'a': a, 'ir': ir, 'ping': ping})
                
            elif cmd == "ENDSCAN":
                self.scanning = False
                self.draw_scan()
        except: pass

    def draw_scan(self):
        # Group contiguous scan points into objects based on distance 
        objects = []
        curr = []
        for p in self.scan_data:
            # Use IR if close <50cm, otherwise Ping
            dist = p['ir'] if p['ir'] < 50 else p['ping']
            
            if dist < 200: curr.append(p)
            elif curr:
                # End of an object, add to list
                objects.append(curr)
                curr = []
        if curr: objects.append(curr)
        
        # Draw each detected object
        for obj in objects:
            # Calculate average angle and distance for the object
            avg_a = sum(p['a'] for p in obj) / len(obj)
            valid_ir = [p['ir'] for p in obj if p['ir'] < 50] #filters list for only within range
            valid_ping = [p['ping'] for p in obj if p['ping'] < 200]
            
            # Convert robot-relative polar coordinates to screen coordinates
            # Robot's direction (0-3) * 90 degrees + object angle - 90 offset
            bot_deg = self.dir * 90
            rad = math.radians(bot_deg + (avg_a - 90))
            
            # Draw Ping detection (Blue) - Larger circle
            if valid_ping:
                d = sum(valid_ping)/len(valid_ping)
                sx, sy = self.to_screen(self.x + d*math.cos(rad), self.y + d*math.sin(rad))
                r = (d * 0.15) * SCALE # Radius scales with distance
                self.canvas.create_oval(sx-r, sy-r, sx+r, sy+r, outline="blue", width=2)
            
            # Draw IR detection (Red) - Smaller circle, drawn on top
            if valid_ir:
                d = sum(valid_ir)/len(valid_ir)
                sx, sy = self.to_screen(self.x + d*math.cos(rad), self.y + d*math.sin(rad))
                self.canvas.create_oval(sx-3, sy-3, sx+3, sy+3, outline="red", width=2)

    def to_screen(self, x, y):
        margin = 50
        return margin + x*SCALE, CANVAS_H - margin - y*SCALE

    def update_bot_pos(self):
        sx, sy = self.to_screen(self.x, self.y)
        self.canvas.coords(self.bot_dot, sx-5, sy-5, sx+5, sy+5)

    def log(self, msg):
        self.log_box.config(state='normal')
        self.log_box.insert(tk.END, msg + "\n")
        self.log_box.see(tk.END)
        self.log_box.config(state='disabled')

if __name__ == "__main__":
    root = tk.Tk()
    CyBotGUI(root)
    root.mainloop()
