# ===== GENERAL PYTHON =====
def general_example():
    # Basic list comprehension
    numbers = [1, 2, 3, 4, 5]
    squared = [x**2 for x in numbers]
    print(f"Squared: {squared}")
    
    # List comprehension with condition
    evens = [x for x in numbers if x % 2 == 0]
    print(f"Even numbers: {evens}")
    
    # List comprehension with if-else
    labels = ["even" if x % 2 == 0 else "odd" for x in numbers]
    print(f"Labels: {labels}")
    
    # Nested list comprehension
    matrix = [[i*j for j in range(1, 4)] for i in range(1, 4)]
    print(f"Matrix: {matrix}")
    
    # List comprehension with function
    def square(x):
        return x * x
    
    squared_func = [square(x) for x in range(1, 6)]
    print(f"Squared with function: {squared_func}")
    
    # Filter and transform
    temps_c = [0, 10, 20, 30, 40]
    temps_f = [c * 9/5 + 32 for c in temps_c]
    print(f"Celsius to Fahrenheit: {temps_f}")
    
    # String manipulation
    words = ["hello", "world", "python"]
    upper_words = [w.upper() for w in words]
    print(f"Uppercase: {upper_words}")

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64MultiArray

class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter')
        
        self.scan_sub_ = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.filtered_pub_ = self.create_publisher(
            Float64MultiArray, 'filtered_scan', 10)
        
        self.get_logger().info('Scan filter node started')
    
    def scan_callback(self, msg):
        ranges = msg.ranges
        
        # Filter valid ranges using list comprehension
        valid_ranges = [r for r in ranges if 0.1 < r < 10.0]
        
        # Filter ranges in front sector (indices 160-200 out of 360)
        front_ranges = [ranges[i] for i in range(160, 200) if i < len(ranges)]
        
        # Convert ranges to points (polar to cartesian)
        angles = [msg.angle_min + i * msg.angle_increment 
                  for i in range(len(ranges))]
        
        # Create obstacle points (distances < 1.0m)
        import math
        obstacle_points = [
            (ranges[i] * math.cos(angles[i]), ranges[i] * math.sin(angles[i]))
            for i in range(len(ranges))
            if ranges[i] < 1.0 and ranges[i] > 0.1
        ]
        
        # Downsample scan (every 10th point)
        downsampled = [ranges[i] for i in range(0, len(ranges), 10)]
        
        # Find all obstacles within critical distance
        critical_obstacles = [i for i, r in enumerate(ranges) if r < 0.5]
        
        # Calculate average distance per sector
        sector_size = len(ranges) // 4
        sector_avgs = [
            sum(ranges[i*sector_size:(i+1)*sector_size]) / sector_size
            for i in range(4)
        ]
        
        # Log statistics
        self.get_logger().info(
            f'Valid: {len(valid_ranges)}/{len(ranges)}, '
            f'Obstacles: {len(obstacle_points)}, '
            f'Critical: {len(critical_obstacles)}')
        
        self.get_logger().info(f'Sector averages: {[f"{x:.2f}" for x in sector_avgs]}')
        
        # Publish filtered data
        filtered_msg = Float64MultiArray()
        filtered_msg.data = valid_ranges
        self.filtered_pub_.publish(filtered_msg)
        
        # Control decision based on comprehensions
        cmd = Twist()
        if len(critical_obstacles) > 0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            self.get_logger().warn('Critical obstacles detected!')
        elif min(front_ranges) > 1.0:
            cmd.linear.x = 0.3
        else:
            cmd.linear.x = 0.1
        
        self.pub_.publish(cmd)

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = ScanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
