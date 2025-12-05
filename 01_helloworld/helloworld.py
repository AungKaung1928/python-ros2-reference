# ===== GENERAL PYTHON =====
def general_example():
    print("Hello, Python!")
    print("Hello", "World", "from", "Python")
    
    # Multiple ways to print
    message = "Hello from variable"
    print(message)
    
    # Formatted string
    name = "Alice"
    print(f"Hello, {name}!")

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('Hello ROS2 from Python!')
        self.get_logger().info('Node initialized successfully')

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = HelloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
