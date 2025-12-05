# ===== GENERAL PYTHON =====
class Animal:
    def __init__(self, name):
        self.name = name
    
    def speak(self):
        print(f"{self.name} makes a sound")
    
    def move(self):
        print(f"{self.name} is moving")

class Dog(Animal):
    def __init__(self, name, breed):
        super().__init__(name)
        self.breed = breed
    
    def speak(self):
        print(f"{self.name} barks")
    
    def fetch(self):
        print(f"{self.name} is fetching the ball")

class Cat(Animal):
    def __init__(self, name, color):
        super().__init__(name)
        self.color = color
    
    def speak(self):
        print(f"{self.name} meows")
    
    def climb(self):
        print(f"{self.name} is climbing")

def general_example():
    # Base class
    animal = Animal("Generic Animal")
    animal.speak()
    animal.move()
    
    print("---")
    
    # Derived class - Dog
    dog = Dog("Buddy", "Golden Retriever")
    dog.speak()  # Overridden method
    dog.move()   # Inherited method
    dog.fetch()  # New method
    
    print("---")
    
    # Derived class - Cat
    cat = Cat("Whiskers", "Orange")
    cat.speak()  # Overridden method
    cat.move()   # Inherited method
    cat.climb()  # New method

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class BaseRobot(Node):
    """Base robot class with common functionality"""
    def __init__(self, node_name):
        super().__init__(node_name)
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.max_speed = 0.3
        
        self.get_logger().info(f'{node_name} initialized')
    
    def stop(self):
        """Common stop functionality"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_.publish(cmd)
        self.get_logger().info('Robot stopped')
    
    def move_forward(self, speed=None):
        """Basic forward movement"""
        if speed is None:
            speed = self.max_speed
        cmd = Twist()
        cmd.linear.x = speed
        self.pub_.publish(cmd)
        self.get_logger().info(f'Moving forward at {speed}m/s')

class ExplorerRobot(BaseRobot):
    """Robot for exploration with obstacle avoidance"""
    def __init__(self):
        super().__init__('explorer_robot')
        self.max_speed = 0.5  # Override base speed
        
        self.scan_sub_ = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        self.get_logger().info('Explorer mode: Obstacle avoidance enabled')
    
    def scan_callback(self, msg):
        front = msg.ranges[len(msg.ranges) // 2]
        
        if front < 0.5:
            self.avoid_obstacle()
        else:
            self.move_forward()
    
    def avoid_obstacle(self):
        """Explorer-specific obstacle avoidance"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5
        self.pub_.publish(cmd)
        self.get_logger().warn('Avoiding obstacle - turning')

class PatrolRobot(BaseRobot):
    """Robot for patrolling predefined waypoints"""
    def __init__(self):
        super().__init__('patrol_robot')
        self.max_speed = 0.2  # Slower for patrol
        
        self.waypoints = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
        self.current_waypoint_idx = 0
        
        self.timer_ = self.create_timer(2.0, self.patrol_callback)
        
        self.get_logger().info(f'Patrol mode: {len(self.waypoints)} waypoints')
    
    def patrol_callback(self):
        """Patrol-specific behavior"""
        if self.current_waypoint_idx < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_idx]
            self.get_logger().info(f'Patrolling to waypoint {waypoint}')
            self.move_forward()
            self.current_waypoint_idx += 1
        else:
            self.get_logger().info('Patrol complete')
            self.stop()
    
    def move_forward(self, speed=None):
        """Override with slower patrol speed"""
        super().move_forward(self.max_speed * 0.5)

class DeliveryRobot(BaseRobot):
    """Robot for package delivery"""
    def __init__(self):
        super().__init__('delivery_robot')
        self.max_speed = 0.4
        self.has_package = False
        
        self.get_logger().info('Delivery mode: Ready for package')
    
    def pickup_package(self):
        """Delivery-specific method"""
        self.has_package = True
        self.get_logger().info('Package picked up')
    
    def deliver_package(self):
        """Delivery-specific method"""
        if self.has_package:
            self.has_package = False
            self.get_logger().info('Package delivered')
            self.stop()
        else:
            self.get_logger().warn('No package to deliver')

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    
    # Create different robot types
    # Uncomment one to test
    # node = ExplorerRobot()
    node = PatrolRobot()
    # node = DeliveryRobot()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
