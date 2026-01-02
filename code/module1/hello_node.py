import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)  # Initialize the rclpy library

    node = Node('hello_node')  # Create a node
    node.get_logger().info('Hello, ROS 2 World!')

    rclpy.spin(node)  # Keep the node running

    # Shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
