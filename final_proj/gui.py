import tkinter as tk
from tkinter import scrolledtext
import socket
import math

# Configuration
HOST = "192.168.1.1" 
PORT = 288
CANVAS_W = 1000
CANVAS_H = 800
SCALE = 1 # pixels per cm
GRID_SIZE = 50 # cm
MAX_X_CM = 800 # max horizontal
MAX_Y_CM = 600 # max vertical
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
        self.horizontal_edge = None
        self.vertical_edge = None
        self.boundary_rect = None

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
        tk.Button(left, text="Rickroll", command=lambda: self.send('1'), bg="#ff69b4", width=20).pack(pady=5)
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
        
        #x0, y0 = self.to_screen(0, 0)
       # x1, y1 = self.to_screen(MAX_X_CM, MAX_Y_CM)
        #self.canvas.create_rectangle(x0, y0, x1, y1, outline="gray", width=2)
   
    def try_draw_boundary(self):
        """Draw the boundary rectangle once both edges are known."""

        # We need both edges to exist
        if self.horizontal_edge is None or self.vertical_edge is None:
            return

        # Delete previous rectangle if robot updates edges
        if self.boundary_rect is not None:
            self.canvas.delete(self.boundary_rect)

        # Convert to screen coords
        x0, y0 = self.to_screen(0, 0)
        x1, y1 = self.to_screen(self.horizontal_edge+OFFSET*2, self.vertical_edge+OFFSET*2)

        #self.canvas.delete("arrow")  # removes all movement arrows

        # Draw new rectangle
        self.boundary_rect = self.canvas.create_rectangle(
            x0, y0, x1, y1, outline="red", width=3
        )

        self.log(f"Boundary drawn: {self.horizontal_edge} x {self.vertical_edge} cm")

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
                self.canvas.create_line(sx1, sy1, sx2, sy2, fill="blue", width=2, arrow=tk.LAST, tags = "arrow")
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
            elif cmd == "EDGE":
                # Format: EDGE HORIZONTAL 600  OR  EDGE VERTICAL 400
                if len(parts) < 3:
                    return

                edge_type = parts[1].upper()
                length = float(parts[2])

                if edge_type == "HORIZONTAL":
                    self.horizontal_edge = length
                    self.log(f"Horizontal edge set: {length} cm")

                elif edge_type == "VERTICAL":
                    self.vertical_edge = length
                    self.log(f"Vertical edge set: {length} cm")

                # Now try to draw rectangle if both are known
                self.try_draw_boundary()
                    
                
        except: pass

    def draw_scan(self):
        objects = []
        curr = []

        ANGLE_STEP = 2.2
        DIST_THRESH = 20
        MIN_POINTS = 5
        MIN_WIDTH = 8

        last_angle = None
        last_dist = None

        for p in self.scan_data:

            # Validation filter
            d = self.valid_distance(p['ir'], p['ping'])
            if d is None:
                continue

            angle = p['a']

            if not curr:
                curr.append({'a': angle, 'd': d})
                last_angle = angle
                last_dist = d
                continue

            angle_diff = angle - last_angle
            dist_diff = abs(d - last_dist)

            if angle_diff <= ANGLE_STEP and dist_diff <= DIST_THRESH:
                curr.append({'a': angle, 'd': d})
            else:
                if len(curr) >= MIN_POINTS:
                    width = curr[-1]['a'] - curr[0]['a']
                    if width >= MIN_WIDTH:
                        objects.append(curr)
                curr = [{'a': angle, 'd': d}]

            last_angle = angle
            last_dist = d

        # last object
        if len(curr) >= MIN_POINTS:
            width = curr[-1]['a'] - curr[0]['a']
            if width >= MIN_WIDTH:
                objects.append(curr)

        # ==== DRAWING ====
        for obj in objects:
            avg_a = sum(p['a'] for p in obj) / len(obj)
            avg_d = sum(p['d'] for p in obj) / len(obj)

            bot_deg = self.dir * 90
            rad = math.radians(bot_deg + (avg_a - 90))

            sx, sy = self.to_screen(self.x + avg_d*math.cos(rad),
                                    self.y + avg_d*math.sin(rad))

            r = avg_d * 0.15 * SCALE
            self.canvas.create_oval(sx-r, sy-r, sx+r, sy+r,
                                    outline="blue", width=2)

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
    
    def valid_distance(self, ir, ping):
        # IR is valid only < 50 cm
        if ir < 50:
            return ir

        # If ping is absurdly different from IR, throw it away
        if abs(ir - ping) > 50:
            return None

        # Accept ping normally (but limit max range)
        if 0 < ping < 500:
            return ping

        return None


if __name__ == "__main__":
    root = tk.Tk()
    CyBotGUI(root)
    root.mainloop()
