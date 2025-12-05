# ===== GENERAL PYTHON =====
class Student:
    def __init__(self, name, age, grade):
        self.name = name
        self.age = age
        self.grade = grade
    
    def display_info(self):
        print(f"Name: {self.name}, Age: {self.age}, Grade: {self.grade}")
    
    def set_grade(self, new_grade):
        self.grade = new_grade
        print(f"{self.name}'s grade updated to {self.grade}")

class Rectangle:
    def __init__(self, width, height):
        self.width = width
        self.height = height
    
    def area(self):
        return self.width * self.height
    
    def perimeter(self):
        return 2 * (self.width + self.height)

def general_example():
    # Create student objects
    student1 = Student("Alice", 20, "A")
    student1.display_info()
    
    student2 = Student("Bob", 22, "B")
    student2.display_info()
    
    # Modify object
    student1.set_grade("A+")
    student1.display_info()
    
    # Rectangle class
    rect = Rectangle(10, 5)
    print(f"Area: {rect.area()}")
    print(f"Perimeter: {rect.perimeter()}")

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotController:
    """Class to manage robot movement logic"""
    def __init__(self, max_speed=0.5, turn_speed=0.3):
        self.max_speed = max_speed
        self.turn_speed = turn_speed
        self.current_speed = 0.0
        self.state = "idle"
    
    def calculate_velocity(self, front_distance):
        """Calculate appropriate velocity based on distance"""
        if front_distance < 0.5:
            self.state = "avoiding"
            return 0.0, self.turn_speed
        elif front_distance < 1.0:
            self.state = "slowing"
            return self.max_speed * 0.3, 0.0
        else:
            self.state = "moving"
            return self.max_speed, 0.0
    
    def get_state(self):
        return self.state

class SensorProcessor:
    """Class to process sensor data"""
    def __init__(self):
        self.min_range = 0.1
        self.max_range = 10.0
        self.history = []
        self.max_history = 10
    
    def filter_scan(self, ranges):
        """Filter valid ranges"""
        return [r for r in ranges if self.min_range < r < self.max_range]
    
    def get_front_distance(self, ranges):
        """Get distance in front"""
        if len(ranges) == 0:
            return float('inf')
        front_idx = len(ranges) // 2
        return ranges[front_idx]
    
    def update_history(self, distance):
        """Store distance in history"""
        self.history.append(distance)
        if len(self.history) > self.max_history:
            self.history.pop(0)
    
    def get_average(self):
        """Calculate average from history"""
        if len(self.history) == 0:
            return 0.0
        return sum(self.history) / len(self.history)

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Create class instances
        self.controller_ = RobotController(max_speed=0.5, turn_speed=0.3)
        self.sensor_ = SensorProcessor()
        
        self.scan_sub_ = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('Navigation node started with classes')
    
    def scan_callback(self, msg):
        # Use sensor processor class
        valid_ranges = self.sensor_.filter_scan(list(msg.ranges))
        front_dist = self.sensor_.get_front_distance(list(msg.ranges))
        
        # Update history
        self.sensor_.update_history(front_dist)
        avg_dist = self.sensor_.get_average()
        
        # Use robot controller class
        linear, angular = self.controller_.calculate_velocity(front_dist)
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.pub_.publish(cmd)
        
        # Log state
        self.get_logger().info(
            f'State: {self.controller_.get_state()}, '
            f'Front: {front_dist:.2f}m, '
            f'Avg: {avg_dist:.2f}m, '
            f'Valid: {len(valid_ranges)}')

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
