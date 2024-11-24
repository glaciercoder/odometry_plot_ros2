import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message
from datetime import datetime
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory

class BagRecorderNode(Node):
    def __init__(self):
        super().__init__('bag_recorder_node')
        
        # Declare and read topic parameters
        self.declare_parameter('odometry_topic', '/kiss/odometry')
        self.declare_parameter('pose_topic', '/real_pose')
        self.declare_parameter('robot_name', 'robot0')

        self.odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        # Set up subscriptions
        self.odometry_subscription = self.create_subscription(
            Odometry,
            self.odometry_topic,
            self.odometry_callback,
            10
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

        # Initialize rosbag writer
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        
        package_path = Path(get_package_share_directory('odometry_processing'))
        output_dir = os.path.join(package_path , 'data')
        os.makedirs(output_dir, exist_ok=True)

        bag_name = os.path.join(output_dir , f'{self.robot_name}_ros2_bag_{timestamp}')
        print(f'Bag created:{bag_name}')
        self.storage_options = StorageOptions(uri=bag_name, storage_id='sqlite3')
        self.converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        self.bag_writer = SequentialWriter()
        self.bag_writer.open(self.storage_options, self.converter_options)
        self.bag_writer.create_topic(TopicMetadata(
            name=self.odometry_topic,
            type='nav_msgs/msg/Odometry',
            serialization_format='cdr'
        ))
        self.bag_writer.create_topic(TopicMetadata(
            name=self.pose_topic,
            type='geometry_msgs/msg/PoseStamped',
            serialization_format='cdr'
        ))

        self.get_logger().info(f"Started recording topics: {self.odometry_topic} and {self.pose_topic}")

    def odometry_callback(self, msg: Odometry):
        self.bag_writer.write(self.odometry_topic, serialize_message(msg), self.get_clock().now().nanoseconds)

    def pose_callback(self, msg: PoseStamped):
        self.bag_writer.write(self.pose_topic, serialize_message(msg), self.get_clock().now().nanoseconds)

    def destroy_node(self):
        self.bag_writer.close()
        self.get_logger().info("Bag file has been closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
