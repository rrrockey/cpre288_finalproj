#Import/Include useful math and plotting functions
import numpy as np
import matplotlib.pyplot as plt
import os

# Path setup
absolute_path = os.path.dirname(__file__) 
relative_path = "./"   
full_path = os.path.join(absolute_path, relative_path)
filename = 'scan.txt'  # Make sure this file contains three columns: angle, distance1, distance2

# Initialize arrays
angle_degrees = []
distance1 = []  # First distance column (2nd column in file)
distance2 = []  # Second distance column (3rd column in file)
objects = [] # array to hold angles of objects

# Read file
file_object = open(full_path + filename, 'r')
file_header = file_object.readline()  # Skip [w] response
file_header = file_object.readline()  # Skip header
file_data = file_object.readlines()
file_object.close()
# Parse data
scan_angle = 0
for line in file_data:
    data = line.split()
    if len(data) >= 3 and scan_angle <= 180:  # Ensure we have enough columns
        try:
            angle_degrees.append(float(data[0]))
            distance1.append(float(data[1]))
            distance2.append(float(data[2]))
            scan_angle += 2
        except ValueError:
            pass
    elif len(data) == 6 and scan_angle > 180:
        try:
            objects.append(data[3])
        except ValueError:
            continue
        except IndexError:
            for number in data:
                print(number)
            continue


# Convert to numpy arrays
angle_degrees = np.array(angle_degrees)
angle_radians = (np.pi / 180) * angle_degrees
distance1 = np.array(distance1)
distance2 = np.array(distance2)

# Create polar plot
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

# Plot both distance sets with different colors
ax.plot(angle_radians, distance1, color='r', linewidth=2.0, label='Distance Set 1')
ax.plot(angle_radians, distance2, color='b', linewidth=2.0, label='Distance Set 2')

# print("Objects array contents:", objects[::])

# Draw green rays from center for each detected object
for obj in objects[1::]:
    try:
        # Try to convert string to float (skip non-numeric like "Mid")
        obj_angle_deg = float(obj)
        obj_angle_rad = np.deg2rad(obj_angle_deg)
        ax.plot([obj_angle_rad, obj_angle_rad], [0, ax.get_rmax()],
                color='g', linewidth=2.0, linestyle='--',
                label='Detected Object' if obj == objects[1] else "")
    except ValueError:
        # Skip any entries that aren’t valid numbers
        print(f"Skipping non-numeric object entry: {obj}")
        continue


# Add labels and formatting
ax.set_xlabel('Distance (m)', fontsize=14)
ax.set_ylabel('Angle (degrees)', fontsize=14)
ax.xaxis.set_label_coords(0.5, 0.15)
ax.tick_params(axis='both', which='major', labelsize=14)
ax.set_rmax(200)
ax.set_rticks([0, 50, 100, 150, 200])
ax.set_rlabel_position(-22.5)
ax.set_thetamax(180)
ax.set_xticks(np.arange(0, np.pi + 0.1, np.pi / 4))
ax.grid(True)

# Title and legend
ax.set_title("Polar Plot of CyBot Sensor Scan (2 Data Sets)", size=14, y=1.0, pad=-24)
ax.legend(loc='upper right', bbox_to_anchor=(1.1, 1.1))
# plt.show()  # Display plot

# save plot instead of show
plt.savefig("polar_plot.png", bbox_inches='tight')
plt.close()

