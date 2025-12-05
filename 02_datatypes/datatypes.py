# ===== GENERAL PYTHON =====
def general_example():
    # Integer
    a = 10
    print(f"Integer: {a}, Type: {type(a)}")
    
    # Float
    b = 3.14
    print(f"Float: {b}, Type: {type(b)}")
    
    # String
    c = "Hello"
    print(f"String: {c}, Type: {type(c)}")
    
    # Boolean
    d = True
    print(f"Boolean: {d}, Type: {type(d)}")
    
    # Type conversion
    x = 5
    y = float(x)
    print(f"Converted {x} to {y}, Type: {type(y)}")
    
    # Multiple assignment
    name, age, height = "Bob", 25, 5.9
    print(f"Name: {name}, Age: {age}, Height: {height}")

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64, String, Bool

class DataTypesNode(Node):
    def __init__(self):
        super().__init__('datatypes_node')
        
        # Different data types in robotics
        speed = 100  # int
        distance = 5.75  # float
        status = "active"  # str
        is_moving = True  # bool
        
        self.get_logger().info(f'Speed (int): {speed}')
        self.get_logger().info(f'Distance (float): {distance:.2f}')
        self.get_logger().info(f'Status (str): {status}')
        self.get_logger().info(f'Moving (bool): {is_moving}')
        
        # Type checking
        self.get_logger().info(f'Speed type: {type(speed).__name__}')

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = DataTypesNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
