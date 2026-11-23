# ===== GENERAL PYTHON =====
def general_example():
    # Create list
    numbers = [1, 2, 3, 4, 5]
    print(f"Numbers: {numbers}")
    
    # Access elements
    print(f"First element: {numbers[0]}")
    print(f"Last element: {numbers[-1]}")
    
    # Slicing
    print(f"First 3: {numbers[0:3]}")
    print(f"Last 2: {numbers[-2:]}")
    
    # Modify list
    numbers.append(6)
    print(f"After append: {numbers}")
    
    numbers.insert(0, 0)
    print(f"After insert: {numbers}")
    
    numbers.remove(3)
    print(f"After remove: {numbers}")
    
    # List operations
    fruits = ["apple", "banana", "cherry"]
    print(f"Length: {len(fruits)}")
    
    # Iterate through list
    for fruit in fruits:
        print(fruit)
    
    # List methods
    values = [3, 1, 4, 1, 5, 9, 2]
    values.sort()
    print(f"Sorted: {values}")
    print(f"Max: {max(values)}, Min: {min(values)}")

# ===== ROS2 EXAMPLE =====
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def ray_index(msg, angle):
    """Index of the ray nearest `angle` (rad; 0 = straight ahead, +pi/2 = left, REP 103).
    Uses angle_min/angle_increment, so it works whether the lidar scans 0..2*pi
    (TurtleBot3 LDS-01) or -pi..pi; fixed fractions like len(ranges) // 4 do not."""
    return round((angle - msg.angle_min) / msg.angle_increment) % len(msg.ranges)


class ScanProcessorNode(Node):
    def __init__(self):
        super().__init__('scan_processor')
        
        self.sub_ = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # List to store recent distances
        self.distance_history_ = []
        self.max_history_ = 10
        
        self.get_logger().info('Scan processor started')
    
    def scan_callback(self, msg):
        # Convert ranges to list
        ranges = list(msg.ranges)
        
        # Filter valid ranges
        valid_ranges = [r for r in ranges if 0.1 < r < 10.0]
        
        # Get front distance
        front_idx = ray_index(msg, 0.0)
        front = ranges[front_idx]
        
        # Store in history
        self.distance_history_.append(front)
        if len(self.distance_history_) > self.max_history_:
            self.distance_history_.pop(0)  # Remove oldest
        
        # Calculate average distance
        if len(self.distance_history_) > 0:
            avg_distance = sum(self.distance_history_) / len(self.distance_history_)
            self.get_logger().info(f'Avg distance: {avg_distance:.2f}m')
        
        # Find minimum distance in front sector
        # 20 rays around the front; % wraps past the end of the list (front can be index 0)
        front_sector = [ranges[(front_idx + k) % len(ranges)] for k in range(-10, 10)]
        min_dist = min(front_sector)
        
        # Decision based on list data
        cmd = Twist()
        if min_dist < 0.5:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            self.get_logger().warn(f'Obstacle at {min_dist:.2f}m')
        else:
            cmd.linear.x = 0.3
            self.get_logger().info(f'Moving forward, min: {min_dist:.2f}m')
        
        self.pub_.publish(cmd)
        
        # Log list statistics
        self.get_logger().info(f'Valid ranges: {len(valid_ranges)}/{len(ranges)}')

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = ScanProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
