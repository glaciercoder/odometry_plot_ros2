from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_path = get_package_share_directory("odometry_processing")
    odom_csv_file_dir = os.path.join(bringup_path, 'data')

    odom_csv_file = LaunchConfiguration('odom_csv_file')

    declare_odom_file_cmd = DeclareLaunchArgument(
        'odom_csv_file',
        default_value=os.path.join(odom_csv_file_dir, "rosbag2_edit_24_11_24-01_18_03_kissodometry.csv"),
        description='The path to odom csv file')
    
    start_plot = ExecuteProcess(
            cmd=[
                "python3", 
                FindPackageShare("odometry_processing").find("odometry_processing") + "/odom_plot.py",
                odom_csv_file
            ],
            output="screen"
        )
    
    ld = LaunchDescription()
    ld.add_action(declare_odom_file_cmd)
    ld.add_action(start_plot)

    return ld
