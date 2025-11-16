import tkinter as tk
from tkinter import scrolledtext
import socket
import threading
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

# CyBot connection settings
CYBOT_IP = "192.168.1.1"
CYBOT_PORT = 288

class CyBotGUI:
    def __init__(self, root):
        self.root = root
        self.root.geometry("1000x700")
        
        # Socket connection
        self.sock = None
        self.connected = False
        self.buttons_pressed = set()
        self.send_loop_active = False
        
        # Scan data storage
        self.scan_angles = []
        self.ir_distances = []
        self.ping_distances = []
        self.detected_objects = []
        
        # Create GUI layout
        self.create_widgets()
        
        # Connect to CyBot
        self.connect_to_cybot()
        
        # Bind keyboard events
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.root.focus_set()
        
    def connect_to_cybot(self):
        """Establish socket connection to CyBot"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((CYBOT_IP, CYBOT_PORT))
            self.sock.settimeout(10)
            self.connected = True
            self.log_message(f"Connected to CyBot at {CYBOT_IP}:{CYBOT_PORT}")
        except Exception as e:
            self.log_message(f"Connection error: {e}")
            
    def create_widgets(self):
        """Create all GUI widgets"""
        
        # Main container
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left panel for controls
        control_frame = tk.Frame(main_frame, width=350)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        control_frame.pack_propagate(False)
        
        # create button layout
        button_frame = tk.Frame(control_frame)
        button_frame.pack(pady=10)
        
        # Create WASD button layout
        btn_style = {"width": 8, "height": 3, "font": ("Arial", 12, "bold")}
        
        # W button (forward)
        self.btn_w = tk.Button(button_frame, text="W\n↑\nForward", **btn_style)
        self.btn_w.grid(row=0, column=1, padx=5, pady=5)
        self.btn_w.bind("<ButtonPress-1>", lambda e: self.button_press('w'))
        self.btn_w.bind("<ButtonRelease-1>", lambda e: self.button_release('w'))
        
        # A button (left)
        self.btn_a = tk.Button(button_frame, text="A\n←\nLeft", **btn_style)
        self.btn_a.grid(row=1, column=0, padx=5, pady=5)
        self.btn_a.bind("<ButtonPress-1>", lambda e: self.button_press('a'))
        self.btn_a.bind("<ButtonRelease-1>", lambda e: self.button_release('a'))
        
        # S button (backward)
        self.btn_s = tk.Button(button_frame, text="S\n↓\nBackward", **btn_style)
        self.btn_s.grid(row=1, column=1, padx=5, pady=5)
        self.btn_s.bind("<ButtonPress-1>", lambda e: self.button_press('s'))
        self.btn_s.bind("<ButtonRelease-1>", lambda e: self.button_release('s'))
        
        # D button (right)
        self.btn_d = tk.Button(button_frame, text="D\n→\nRight", **btn_style)
        self.btn_d.grid(row=1, column=2, padx=5, pady=5)
        self.btn_d.bind("<ButtonPress-1>", lambda e: self.button_press('d'))
        self.btn_d.bind("<ButtonRelease-1>", lambda e: self.button_release('d'))
        
        # Scan button
        action_frame = tk.Frame(control_frame)
        action_frame.pack(pady=20)
        
        self.scan_btn = tk.Button(action_frame, text="SCAN (M)", 
                                  command=self.initiate_scan,
                                  width=15, height=2, font=("Arial", 10, "bold"))
        self.scan_btn.pack(pady=5)
        
        # Console log
        log_label = tk.Label(control_frame, text="Console Log", 
                            font=("Arial", 10, "bold"))
        log_label.pack(pady=(10, 5))
        
        self.log_text = scrolledtext.ScrolledText(control_frame, height=10, 
                                                  font=("Courier", 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # Right panel for plot
        plot_frame = tk.Frame(main_frame, bg="white")
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Create matplotlib figure (semicircle view)
        self.fig = Figure(figsize=(7, 4.5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='polar')
        
        # Configure for 0-180 degree semicircle
        self.ax.set_theta_zero_location('E')
        self.ax.set_theta_direction(1)
        self.ax.set_thetamin(0)
        self.ax.set_thetamax(180)
        self.ax.set_ylim(0, 4)
        self.ax.set_ylabel("Distance (m)", fontsize=10, labelpad=30)
        self.ax.grid(True, alpha=0.3)
        
        # Embed plot in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
    def log_message(self, message):
        """Add message to console log"""
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
                
    def button_press(self, key):
        """Handle button press for continuous sending"""
        self.buttons_pressed.add(key)
        if not self.send_loop_active:
            self.send_loop_active = True
            self.send_keys_loop()
            
    def button_release(self, key):
        """Handle button release"""
        self.buttons_pressed.discard(key)
        if not self.buttons_pressed:
            self.send_loop_active = False
            
    def send_keys_loop(self):
        """Continuously send pressed keys"""
        if self.send_loop_active and self.buttons_pressed:
            for key in self.buttons_pressed:
                if self.connected and self.sock:
                    try:
                        self.sock.sendall(key.encode('utf-8'))
                    except:
                        pass
            self.root.after(50, self.send_keys_loop)
            
    def on_key_press(self, event):
        """Handle keyboard key press"""
        key = event.char.lower()
        
        if key == 'q':
            self.quit_application()
        elif key == 'm':
            self.initiate_scan()
        elif key in ['w', 'a', 's', 'd']:
            self.button_press(key)
            
    def on_key_release(self, event):
        """Handle keyboard key release"""
        key = event.char.lower()
        if key in ['w', 'a', 's', 'd']:
            self.button_release(key)
        
    def initiate_scan(self):
        """Start scan in separate thread"""
        self.log_message("--- Starting Scan ---")
        self.scan_btn.config(state=tk.DISABLED)
            
        thread.start()
        
    def perform_scan(self):
        """Perform scan and update plot"""
        try:
            # Send scan command
            self.sock.sendall(b'm')
            
            # Clear previous data
            self.scan_angles = []
            self.ir_distances = []
            self.ping_distances = []
            self.detected_objects = []
            
            scan_angle = 0
            
            # Read scan data
            while True:
                try:
                    data = self.sock.recv(4096)
                    if not data:
                        break
                        
                    decoded = data.decode(errors="replace").strip()
                    if decoded:
                        self.root.after(0, self.log_message, decoded)
                        
                        # Parse scan data
                        lines = decoded.split('\n')
                        for line in lines:
                            parts = line.replace('\t', ' ').split()
                            
                            # Regular scan data: angle ir_distance ping_distance (3 columns)
                            if len(parts) >= 3 and scan_angle <= 180:
                                try:
                                    angle = float(parts[0])
                                    ir_dist = float(parts[1])
                                    ping_dist = float(parts[2])
                                    
                                    self.scan_angles.append(angle)
                                    self.ir_distances.append(ir_dist)
                                    self.ping_distances.append(ping_dist)
                                    scan_angle += 2
                                except ValueError:
                                    pass
                            
                            # Object detection data: 6 columns with object angle in 4th position
                            elif len(parts) == 6 and scan_angle > 180:
                                try:
                                    obj_angle = float(parts[3])
                                    self.detected_objects.append(obj_angle)
                                except (ValueError, IndexError):
                                    pass
                                    
                except socket.timeout:
                    break
                    
            # Update plot
            self.root.after(0, self.update_plot)
            self.root.after(0, self.log_message, "--- Scan Complete ---")
            
        except Exception as e:
            self.root.after(0, self.log_message, f"Scan error: {e}")
        finally:
            self.root.after(0, lambda: self.scan_btn.config(state=tk.NORMAL))
            
    def update_plot(self):
        """Update the polar plot with scan data"""
        self.ax.clear()
        
        # Configure for 0-180 degree semicircle
        self.ax.set_theta_zero_location('E')
        self.ax.set_theta_direction(1)
        self.ax.set_thetamin(0)
        self.ax.set_thetamax(180)
        
        if self.scan_angles and (self.ir_distances or self.ping_distances):
            # Convert angles to radians
            angles_rad = np.array([np.radians(a) for a in self.scan_angles])
            ir_dists = np.array(self.ir_distances)
            ping_dists = np.array(self.ping_distances)
            
            # Plot IR distances in RED
            if len(ir_dists) > 0:
                self.ax.plot(angles_rad, ir_dists, color='r', linewidth=2.0, label='IR Sensor')
            
            # Plot Ping distances in BLUE
            if len(ping_dists) > 0:
                self.ax.plot(angles_rad, ping_dists, color='b', linewidth=2.0, label='Ping Sensor')
            
            # Draw GREEN dashed rays for detected objects
            max_range = 200
            for i, obj_angle in enumerate(self.detected_objects):
                try:
                    obj_angle_rad = np.radians(float(obj_angle))
                    self.ax.plot([obj_angle_rad, obj_angle_rad], [0, max_range],
                               color='g', linewidth=2.0, linestyle='--',
                               label='Detected Object' if i == 0 else "")
                except (ValueError, TypeError):
                    continue
            
            # Set appropriate limits
            all_distances = list(ir_dists) + list(ping_dists)
            if all_distances:
                max_dist = max(all_distances)
                self.ax.set_rmax(min(max_dist * 1.2, 200))
            else:
                self.ax.set_rmax(200)
            
            self.ax.set_rticks([0, 50, 100, 150, 200])
            self.ax.set_rlabel_position(-22.5)
            
        else:
            self.ax.set_rmax(200)
            self.ax.set_rticks([0, 50, 100, 150, 200])
            self.ax.set_rlabel_position(-22.5)
            
        # Formatting
        self.ax.set_xlabel('Distance (cm)', fontsize=12)
        self.ax.set_ylabel('Angle (degrees)', fontsize=12)
        self.ax.xaxis.set_label_coords(0.5, 0.15)
        self.ax.tick_params(axis='both', which='major', labelsize=12)
        self.ax.set_xticks(np.arange(0, np.pi + 0.1, np.pi / 4))
        self.ax.grid(True)
        
        self.ax.set_title("Polar Plot of CyBot Sensor Scan", size=12, y=1.0, pad=-20)
        self.ax.legend(loc='upper right', bbox_to_anchor=(1.1, 1.1), fontsize=9)
        
        self.canvas.draw()
        
    def quit_application(self):
        """Close connection and quit"""
        self.log_message("Exiting application...")
        if self.sock:
            self.sock.close()
        self.root.quit()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = CyBotGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.quit_application)
    root.mainloop()


if __name__ == "__main__":
    main()