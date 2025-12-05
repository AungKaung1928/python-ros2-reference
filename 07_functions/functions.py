# ===== GENERAL PYTHON =====
def add(a, b):
    return a + b

def subtract(a, b):
    return a - b

def multiply(a, b):
    return a * b

def divide(a, b):
    if b != 0:
        return a / b
    else:
        return "Cannot divide by zero"

def greet(name, age=25):
    """Function with default parameter"""
    print(f"Hello {name}, you are {age} years old")

def get_max(a, b, c):
    """Return maximum of three numbers"""
    if a > b and a > c:
        return a
    elif b > c:
        return b
    else:
        return c

def general_example():
    # Basic function calls
    x = 10
    y = 5
    print(f"Add: {add(x, y)}")
    print(f"Subtract: {subtract(x, y)}")
    print(f"Multiply: {multiply(x, y)}")
    print(f"Divide: {divide(x, y)}")
    
    # Function with default parameter
    greet("Alice")
    greet("Bob", 30)
    
    # Function returning value
    maximum = get_max(10, 25, 15)
    print(f"Maximum: {maximum}")

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        self.sub_ = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('Obstacle avoidance started')
    
    def calculate_speed(self, distance):
        """Calculate speed based on distance"""
        if distance < 0.5:
            return 0.0
        elif distance < 1.0:
            return 0.1
        else:
            return 0.3
    
    def calculate_turn(self, left, right):
        """Calculate turn rate based on left/right distances"""
        return (left - right) * 0.5
    
    def is_path_clear(self, distance, threshold=0.5):
        """Check if path is clear"""
        return distance > threshold
    
    def scan_callback(self, msg):
        # Extract distances
        ranges = msg.ranges
        front = ranges[len(ranges) // 2]
        left = ranges[len(ranges) // 4]
        right = ranges[3 * len(ranges) // 4]
        
        # Use functions to calculate movement
        cmd = Twist()
        cmd.linear.x = self.calculate_speed(front)
        cmd.angular.z = self.calculate_turn(left, right)
        
        self.pub_.publish(cmd)
        
        if self.is_path_clear(front):
            self.get_logger().info(f'Path clear: {front:.2f}m')
        else:
            self.get_logger().warn(f'Obstacle detected: {front:.2f}m')

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
