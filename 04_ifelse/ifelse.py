# ===== GENERAL PYTHON =====
def general_example():
    # Find maximum of three numbers
    a = int(input("Enter first number: "))
    b = int(input("Enter second number: "))
    c = int(input("Enter third number: "))
    
    if a > b:
        if a > c:
            print(f"{a} is maximum")
        else:
            print(f"{c} is maximum")
    else:
        if b > c:
            print(f"{b} is maximum")
        else:
            print(f"{c} is maximum")
    
    # Check odd or even
    num = int(input("Enter a number: "))
    if num % 2 == 0:
        print(f"{num} is even")
    else:
        print(f"{num} is odd")

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        self.sub_ = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('Obstacle detector started')
    
    def scan_callback(self, msg):
        # Get distances in different directions
        ranges = msg.ranges
        front = ranges[len(ranges) // 2]
        left = ranges[len(ranges) // 4]
        right = ranges[3 * len(ranges) // 4]
        
        cmd = Twist()
        
        # Decision making based on obstacles
        if front > left and front > right:
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            self.get_logger().info(f'Front is clearest: {front:.2f}')
        elif left > right:
            cmd.linear.x = 0.1
            cmd.angular.z = 0.5
            self.get_logger().info(f'Left is clearest: {left:.2f}')
        else:
            cmd.linear.x = 0.1
            cmd.angular.z = -0.5
            self.get_logger().info(f'Right is clearest: {right:.2f}')
        
        self.pub_.publish(cmd)

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
