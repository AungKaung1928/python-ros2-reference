# ===== GENERAL PYTHON =====
def general_example():
    # Create tuple
    coordinates = (10, 20, 30)
    print(f"Coordinates: {coordinates}")
    
    # Access elements
    print(f"X: {coordinates[0]}")
    print(f"Y: {coordinates[1]}")
    print(f"Z: {coordinates[2]}")
    
    # Tuple unpacking
    x, y, z = coordinates
    print(f"Unpacked - X: {x}, Y: {y}, Z: {z}")
    
    # Tuples are immutable
    # coordinates[0] = 15  # This would cause an error
    
    # Tuple with different types
    person = ("Alice", 25, 5.6)
    name, age, height = person
    print(f"Name: {name}, Age: {age}, Height: {height}")
    
    # Function returning tuple
    def get_min_max(numbers):
        return min(numbers), max(numbers)
    
    values = [3, 1, 4, 1, 5, 9, 2]
    minimum, maximum = get_min_max(values)
    print(f"Min: {minimum}, Max: {maximum}")
    
    # Tuple operations
    print(f"Length: {len(coordinates)}")
    print(f"Count of 20: {coordinates.count(20)}")
    print(f"Index of 30: {coordinates.index(30)}")

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry

class PoseTrackerNode(Node):
    def __init__(self):
        super().__init__('pose_tracker')
        
        self.odom_sub_ = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Store waypoints as tuples (immutable)
        self.waypoints_ = [
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0)
        ]
        
        self.current_waypoint_idx_ = 0
        
        self.get_logger().info('Pose tracker started')
        self.get_logger().info(f'Waypoints: {self.waypoints_}')
    
    def get_distance(self, pos1, pos2):
        """Calculate distance between two positions (tuples)"""
        x1, y1 = pos1
        x2, y2 = pos2
        return ((x2 - x1)**2 + (y2 - y1)**2)**0.5
    
    def odom_callback(self, msg):
        # Extract current position as tuple
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_pos = (current_x, current_y)
        
        # Get target waypoint
        if self.current_waypoint_idx_ < len(self.waypoints_):
            target = self.waypoints_[self.current_waypoint_idx_]
            target_x, target_y = target  # Tuple unpacking
            
            # Calculate distance to target
            distance = self.get_distance(current_pos, target)
            
            self.get_logger().info(
                f'Current: ({current_x:.2f}, {current_y:.2f}), '
                f'Target: ({target_x:.2f}, {target_y:.2f}), '
                f'Distance: {distance:.2f}m')
            
            # Move towards waypoint
            cmd = Twist()
            if distance < 0.2:
                self.current_waypoint_idx_ += 1
                self.get_logger().info('Waypoint reached!')
            else:
                cmd.linear.x = 0.2
            
            self.pub_.publish(cmd)
        else:
            self.get_logger().info('All waypoints reached!')

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = PoseTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
