import pandas as pd
import matplotlib.pyplot as plt
import argparse

def plot_odometry(file_path):
    # Load the CSV file
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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot odometry data from a ROS2 bag file.")
    parser.add_argument("file_path", type=str, help="Path to the CSV file containing odometry data.")
    args = parser.parse_args()
    plot_odometry(args.file_path)
