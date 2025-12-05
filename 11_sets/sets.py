# ===== GENERAL PYTHON =====
def general_example():
    # Create set
    numbers = {1, 2, 3, 4, 5}
    print(f"Numbers: {numbers}")
    
    # Sets have unique elements
    duplicates = {1, 2, 2, 3, 3, 4}
    print(f"Set removes duplicates: {duplicates}")
    
    # Add and remove
    numbers.add(6)
    print(f"After add: {numbers}")
    
    numbers.remove(3)
    print(f"After remove: {numbers}")
    
    # Set operations
    set1 = {1, 2, 3, 4, 5}
    set2 = {4, 5, 6, 7, 8}
    
    print(f"Union: {set1 | set2}")
    print(f"Intersection: {set1 & set2}")
    print(f"Difference: {set1 - set2}")
    print(f"Symmetric difference: {set1 ^ set2}")
    
    # Check membership
    if 3 in set1:
        print("3 is in set1")
    
    # Convert list to set (remove duplicates)
    fruits = ["apple", "banana", "apple", "cherry", "banana"]
    unique_fruits = set(fruits)
    print(f"Unique fruits: {unique_fruits}")

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleTrackerNode(Node):
    def __init__(self):
        super().__init__('obstacle_tracker')
        
        # Sets to track detected obstacles
        self.detected_obstacles_ = set()
        self.previous_obstacles_ = set()
        
        # Set of valid zones
        self.safe_zones_ = {"zone_A", "zone_B", "zone_C"}
        self.danger_zones_ = {"zone_D", "zone_E"}
        
        self.scan_sub_ = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.zone_sub_ = self.create_subscription(
            String, 'current_zone', self.zone_callback, 10)
        
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('Obstacle tracker started')
        self.get_logger().info(f'Safe zones: {self.safe_zones_}')
    
    def scan_callback(self, msg):
        # Detect obstacles in different sectors
        current_obstacles = set()
        
        ranges = msg.ranges
        sectors = {
            "front": ranges[len(ranges)//2 - 10:len(ranges)//2 + 10],
            "left": ranges[0:len(ranges)//4],
            "right": ranges[3*len(ranges)//4:],
        }
        
        # Add obstacles to set
        for sector, distances in sectors.items():
            min_dist = min(distances) if distances else float('inf')
            if min_dist < 0.5:
                current_obstacles.add(sector)
        
        # Find new obstacles (set difference)
        new_obstacles = current_obstacles - self.previous_obstacles_
        if new_obstacles:
            self.get_logger().warn(f'New obstacles: {new_obstacles}')
        
        # Find cleared obstacles
        cleared = self.previous_obstacles_ - current_obstacles
        if cleared:
            self.get_logger().info(f'Cleared obstacles: {cleared}')
        
        # Update sets
        self.previous_obstacles_ = current_obstacles.copy()
        self.detected_obstacles_ = self.detected_obstacles_ | current_obstacles
        
        # Control based on obstacles
        cmd = Twist()
        if len(current_obstacles) == 0:
            cmd.linear.x = 0.3
            self.get_logger().info('No obstacles, moving forward')
        else:
            cmd.linear.x = 0.0
            self.get_logger().warn(f'Obstacles detected: {current_obstacles}')
        
        self.pub_.publish(cmd)
    
    def zone_callback(self, msg):
        current_zone = msg.data
        
        # Check if zone is safe (set membership)
        if current_zone in self.safe_zones_:
            self.get_logger().info(f'In safe zone: {current_zone}')
        elif current_zone in self.danger_zones_:
            self.get_logger().warn(f'In danger zone: {current_zone}')
        else:
            self.get_logger().info(f'Unknown zone: {current_zone}')

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = ObstacleTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
