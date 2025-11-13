import socket
from pynput import keyboard
import subprocess

HOST = "192.168.1.1"
PORT = 288

cybot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
cybot_socket.connect((HOST, PORT))
cybot_socket.settimeout(10)  # default timeout for all reads

print("Connected to CyBot. Press 'w', 'a', 's', 'd' to drive, 'm' to scan, 'q' to quit.\n")


def send_bytes_and_log(b: bytes):
    for byte in b:
        cybot_socket.sendall(bytes([byte]))
        # print(f"Sent byte: {byte} (char={chr(byte) if 32 <= byte <= 126 else '.'})")


def read_response():
    """Read whatever data is available without blocking forever."""
    data = cybot_socket.recv(4096)
    if data:
        decoded = data.decode(errors="replace").strip()
        if decoded:
            print(decoded)
            return decoded + "\n"
    return ""


def on_press(key):
    try:
        char = key.char

        if char == 'q':
            print("\nExiting...")
            cybot_socket.close()
            return False

        send_bytes_and_log(char.encode('utf-8'))

        if char == 'm':
            print("\n--- Starting Scan ---")

            # open file for writing (overwrite each scan)
            with open("scan.txt", "w") as f:
                # f.write("Angle(Degrees)\tDistance(m)\n")
                while True:
                    try:
                        line = read_response()
                        if line:
                            f.write(line)
                        else:
                            break
                    except socket.timeout:
                        # print("socket timeout2")
                        break


            # print("--- Scan Complete ---\n")
            # print("✅ Scan data saved to scan.txt")

            # Runs the script as a separate process
            subprocess.run(["python3", "CyBot-Plot-Sensor-Scan-Values-From-File.py"])
        elif char == 'f':
            print("\n--- Following mode active (waiting for obstacle) ---")
            
            # Disable timeout so it blocks until data arrives
            cybot_socket.settimeout(None)
            
            with open("scan.txt", "w") as f:
                while True:
                    line = read_response()
                    if line:
                        if "found obstacle" in line.lower():
                            print("🚧 Found obstacle, stopping follow mode.")
                            break
                        f.write(line)
                    else:
                        continue

            # Restore timeout for normal operations
            cybot_socket.settimeout(10)

            # Plot after loop finishes
            subprocess.run(["python3", "CyBot-Plot-Sensor-Scan-Values-From-File.py"])

        else:
            read_response()

    except AttributeError:
        pass
    except Exception as e:
        print("Error:", e)


with keyboard.Listener(on_press=on_press) as listener:
    listener.join()
