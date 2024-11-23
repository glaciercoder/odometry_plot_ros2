import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
file_path = '/mnt/data/rosbag2_edit_24_11_24-01_18_03_kissodometry.csv'
data = pd.read_csv(file_path)

# Extract position data
x = data['/kiss/odometry/pose/pose/position/x']
y = data['/kiss/odometry/pose/pose/position/y']

# Plot the odometry path
plt.figure(figsize=(10, 6))
plt.plot(x, y, label='Odometry Path')
plt.xlabel('Position X')
plt.ylabel('Position Y')
plt.title('Odometry Path from ROS2 Bag File')
plt.legend()
plt.grid()
plt.show()
