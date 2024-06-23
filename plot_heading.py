import csv
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# Specify the CSV file names
csv_file_name1 = "base.csv"
csv_file_name2 = "waypoints_pp.csv"

# Lists to store X, Y, and Heading values for each file
x_values1, y_values1 = [], []
x_values2, y_values2 = [], []
heading_vector = []
arrow_annotations = []

# Create a figure and axis for the plot
fig, ax = plt.subplots(figsize=(10, 10))
line1, = ax.plot([], [], marker='o', linestyle='-', label='Base 1', color='blue')
line2, = ax.plot([], [], marker='o', linestyle='-', label='Base 2', color='red')

# Disable gridlines
ax.grid(False)

# Set initial axis limits
ax.set_xlim(-30, 60)
ax.set_ylim(-25, 10)

# Function to update the plot
def update(frame):
    # Read data from the first CSV file
    with open(csv_file_name1, mode='r') as file:
        csv_reader = csv.reader(file)
        x_values1.clear()
        y_values1.clear()
        for row in csv_reader:
            x_values1.append(float(row[0]))
            y_values1.append(float(row[1]))

    # Read data from the second CSV file
    with open(csv_file_name2, mode='r') as file:
        csv_reader = csv.reader(file)
        x_values2.clear()
        y_values2.clear()
        heading_vector.clear()
        for row in csv_reader:
            x_values2.append(float(row[0]))
            y_values2.append(float(row[1]))
            heading_vector.append(float(row[2]))

    # Update the plot data
    line1.set_data(x_values1, y_values1)
    line2.set_data(x_values2, y_values2)

    # Remove previous arrows from the plot
    for annotation in arrow_annotations:
        annotation.remove()
    arrow_annotations.clear()

     # Add vector arrows to line2
    for x, y, heading in zip(x_values2, y_values2, heading_vector):
        arrow_length = 10
        arrow_color = 'coral'
        dx = arrow_length * np.sin(np.radians(heading))
        dy = arrow_length * np.cos(np.radians(heading))
        arrow = ax.annotate("", xy=(x + dx, y + dy), xytext=(x, y),
                    arrowprops=dict(arrowstyle="->", color=arrow_color, lw=1))
        arrow_annotations.append(arrow)

    # Adjust the plot limits and set equal aspect ratio
    ax.relim()
    ax.autoscale_view()
    ax.set_aspect('equal', adjustable='box')

# Create an animation
animation = FuncAnimation(fig, update, interval=1000)  # Refresh every 1000 milliseconds (1 second)

# Display the plot
plt.title('X and Y values over Time')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()
plt.show()