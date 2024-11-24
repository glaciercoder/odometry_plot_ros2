import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import asyncio

class GazeboStatePublisher(Node):
    def __init__(self):
        super().__init__("gazebo_state_publisher")

        # Declare and read parameters
        self.declare_parameter("entity_name", "robot0")
        self.declare_parameter("gazebo_service", "/gazebo/get_entity_state")
        self.declare_parameter("publish_topic", "/real_pose")
        self.declare_parameter("frequency", 10.0)  # Frequency in Hz

        self.entity_name = self.get_parameter("entity_name").value
        self.gazebo_service = self.get_parameter("gazebo_service").value
        self.publish_topic = self.get_parameter("publish_topic").value
        self.frequency = self.get_parameter("frequency").value

        # Set up Gazebo service client
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.gazebo_client = self.create_client(GetEntityState, self.gazebo_service, callback_group=self.callback_group)
        while not self.gazebo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service {self.gazebo_service}...")

        # Set up publisher
        self.state_publisher = self.create_publisher(PoseStamped, self.publish_topic, 10)
        self.get_logger().info(f"Publishing entity state on topic: {self.publish_topic}")

        # Timer to call the Gazebo service at the specified frequency
        self.timer = self.create_timer(1.0 / self.frequency, self.publish_entity_state, callback_group=self.service_group)

    async def publish_entity_state(self):
        # Prepare the service request
        request = GetEntityState.Request()
        request.name = self.entity_name

        # Call the service
        future = self.gazebo_client.call_async(request)
        await future

        if future.result() is not None:
            response = future.result()
            if response.success:
                # Convert response to PoseStamped and publish
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "world"  # Assuming world frame
                pose_msg.pose = response.state.pose

                self.state_publisher.publish(pose_msg)
                self.get_logger().info(
                    f"Published entity '{self.entity_name}' state: {pose_msg.pose.position}"
                )
            else:
                self.get_logger().warn(f"Failed to get state for entity '{self.entity_name}'")
        else:
            self.get_logger().error("Service call failed")


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node = GazeboStatePublisher()

    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
