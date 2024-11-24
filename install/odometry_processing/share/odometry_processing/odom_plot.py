import os
import pandas as pd
import matplotlib.pyplot as plt
from rclpy.node import Node
import rclpy
from ament_index_python.packages import get_package_share_directory


class OdometryPlotterNode(Node):
    def __init__(self):
        super().__init__('odometry_plotter')
        
        # Declare and get parameters
        self.declare_parameter('robot_name', 'robot0')
        self.declare_parameter('time', '2024-11-24_15-54-05')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.time = self.get_parameter('time').get_parameter_value().string_value
        
        # Define the data directory using get_package_share_directory
        try:
            base_dir = get_package_share_directory('odometry_processing')
        except Exception as e:
            self.get_logger().error(f"Could not find package 'odometry_processing': {e}")
            return
        
        folder_name = f"{self.robot_name}_ros2_bag_{self.time}"
        folder_path = os.path.join(base_dir, 'data', folder_name)
        
        # Process CSVs
        csv_files = self.find_csv_files(folder_path)
        
        if len(csv_files) != 2:
            self.get_logger().error(f"Expected exactly 2 CSV files in folder: {folder_path}, found {len(csv_files)}")
            return
        
        # Separate kissodometry and real pose files
        odometry_file = next((f for f in csv_files if 'kissodometry' in f), None)
        real_pose_file = next((f for f in csv_files if 'real_pose' in f), None)
        
        if not odometry_file or not real_pose_file:
            self.get_logger().error(f"Missing required files: odometry or real pose CSV.")
            return
        
        # Plot data
        self.plot_csv_data(odometry_file, real_pose_file)

    def find_csv_files(self, folder_path):
        try:
            files = os.listdir(folder_path)
            csv_files = [os.path.join(folder_path, f) for f in files if f.endswith('.csv')]
            return csv_files
        except FileNotFoundError:
            self.get_logger().error(f"Folder not found: {folder_path}")
            return []

    def plot_csv_data(self, odometry_file, real_pose_file):
        plt.figure(figsize=(10, 6))
        
        # Process odometry data
        try:
            odometry_data = pd.read_csv(odometry_file)
            x_odom = odometry_data['/kiss/odometry/pose/pose/position/x']
            y_odom = odometry_data['/kiss/odometry/pose/pose/position/y']
            plt.plot(x_odom, y_odom, label='Odometry Path', linestyle='-', marker='o')
        except Exception as e:
            self.get_logger().error(f"Failed to process odometry file {odometry_file}: {e}")
        
        # Process real pose data
        try:
            real_pose_data = pd.read_csv(real_pose_file)
            x_real = real_pose_data['/real/pose/pose/position/x']
            y_real = real_pose_data['/real/pose/pose/position/y']
            plt.plot(x_real, y_real, label='Real Pose Path', linestyle='--', marker='x')
        except Exception as e:
            self.get_logger().error(f"Failed to process real pose file {real_pose_file}: {e}")
        
        # Configure plot
        plt.xlabel('Position X')
        plt.ylabel('Position Y')
        plt.title('Odometry vs Real Pose')
        plt.legend()
        plt.grid()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPlotterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
