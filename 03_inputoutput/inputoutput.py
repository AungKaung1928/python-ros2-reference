# ===== GENERAL PYTHON =====
def general_example():
    # Basic input/output
    name = input("Enter your name: ")
    print(f"Hello, {name}!")
    
    # Input with type conversion
    age = int(input("Enter your age: "))
    print(f"You are {age} years old")
    
    # Multiple inputs
    num1 = int(input("Enter first number: "))
    num2 = int(input("Enter second number: "))
    sum_result = num1 + num2
    print(f"Sum: {sum_result}")
    
    # Formatted output
    pi = 3.14159
    print(f"Pi value: {pi:.2f}")

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class InputOutputNode(Node):
    def __init__(self):
        super().__init__('input_output_node')
        
        # Subscribe to input topics
        self.sub1_ = self.create_subscription(
            Int32, 'input1', self.callback1, 10)
        self.sub2_ = self.create_subscription(
            Int32, 'input2', self.callback2, 10)
        
        # Publish to output topic
        self.pub_ = self.create_publisher(Int32, 'sum', 10)
        
        self.value1_ = 0
        self.value2_ = 0
        
        self.get_logger().info('InputOutput node started')
    
    def callback1(self, msg):
        self.value1_ = msg.data
        self.get_logger().info(f'Received input1: {self.value1_}')
        self.publish_sum()
    
    def callback2(self, msg):
        self.value2_ = msg.data
        self.get_logger().info(f'Received input2: {self.value2_}')
        self.publish_sum()
    
    def publish_sum(self):
        result = Int32()
        result.data = self.value1_ + self.value2_
        self.pub_.publish(result)
        self.get_logger().info(f'Published sum: {result.data}')

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = InputOutputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
