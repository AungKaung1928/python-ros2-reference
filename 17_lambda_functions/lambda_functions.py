# ===== GENERAL PYTHON =====
def general_example():
    # Basic lambda function
    add = lambda x, y: x + y
    print(f"Add: {add(5, 3)}")
    
    # Lambda with single parameter
    square = lambda x: x ** 2
    print(f"Square of 5: {square(5)}")
    
    # Lambda with multiple parameters
    multiply = lambda x, y, z: x * y * z
    print(f"Multiply: {multiply(2, 3, 4)}")
    
    print("---")
    
    # Lambda with map
    numbers = [1, 2, 3, 4, 5]
    squared = list(map(lambda x: x**2, numbers))
    print(f"Squared: {squared}")
    
    # Lambda with filter
    evens = list(filter(lambda x: x % 2 == 0, numbers))
    print(f"Even numbers: {evens}")
    
    # Lambda with sorted
    points = [(1, 5), (3, 2), (2, 8), (4, 1)]
    sorted_by_y = sorted(points, key=lambda p: p[1])
    print(f"Sorted by y: {sorted_by_y}")
    
    print("---")
    
    # Lambda in list comprehension with conditional
    result = [(lambda x: x**2 if x % 2 == 0 else x**3)(x) for x in range(1, 6)]
    print(f"Conditional lambda: {result}")
    
    # Lambda with reduce
    from functools import reduce
    sum_all = reduce(lambda x, y: x + y, numbers)
    print(f"Sum using reduce: {sum_all}")

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float64

class LambdaRobotNode(Node):
    def __init__(self):
        super().__init__('lambda_robot')
        
        # Lambda for callbacks
        self.scan_sub_ = self.create_subscription(
            LaserScan, 'scan', 
            lambda msg: self.process_scan(msg), 10)
        
        self.mode_sub_ = self.create_subscription(
            Int32, 'mode',
            lambda msg: self.get_logger().info(f'Mode changed to: {msg.data}'), 10)
        
        self.speed_sub_ = self.create_subscription(
            Float64, 'speed_factor',
            lambda msg: setattr(self, 'speed_factor_', msg.data), 10)
        
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Lambda for speed calculation
        self.calculate_speed = lambda distance: 0.0 if distance < 0.5 else \
                                                 0.1 if distance < 1.0 else 0.3
        
        # Lambda for turn calculation
        self.calculate_turn = lambda left, right: (left - right) * 0.5
        
        # Lambda for filtering
        self.is_valid_range = lambda r: 0.1 < r < 10.0
        
        self.speed_factor_ = 1.0
        
        # Timer with lambda
        self.timer_ = self.create_timer(
            1.0, 
            lambda: self.get_logger().info(f'Heartbeat - Speed factor: {self.speed_factor_}'))
        
        self.get_logger().info('Lambda robot node started')
    
    def process_scan(self, msg):
        ranges = list(msg.ranges)
        
        # Filter valid ranges using lambda
        valid_ranges = list(filter(self.is_valid_range, ranges))
        
        # Map ranges to distances with safety margin
        safe_ranges = list(map(lambda r: r - 0.1, valid_ranges))
        
        # Get sector distances
        front_idx = len(ranges) // 2
        left_idx = len(ranges) // 4
        right_idx = 3 * len(ranges) // 4
        
        front = ranges[front_idx]
        left = ranges[left_idx]
        right = ranges[right_idx]
        
        # Use lambda functions for calculations
        linear_speed = self.calculate_speed(front) * self.speed_factor_
        angular_speed = self.calculate_turn(left, right)
        
        # Find minimum distance using lambda with reduce
        from functools import reduce
        min_distance = reduce(lambda x, y: x if x < y else y, valid_ranges, float('inf'))
        
        # Classify ranges using lambda
        close_obstacles = list(filter(lambda r: r < 0.5, valid_ranges))
        medium_obstacles = list(filter(lambda r: 0.5 <= r < 1.0, valid_ranges))
        far_obstacles = list(filter(lambda r: r >= 1.0, valid_ranges))
        
        # Sort sectors by distance using lambda
        sectors = [
            ('front', front),
            ('left', left),
            ('right', right)
        ]
        sorted_sectors = sorted(sectors, key=lambda s: s[1], reverse=True)
        clearest_direction = sorted_sectors[0][0]
        
        # Decision using lambda
        should_stop = (lambda: len(close_obstacles) > 10)()
        
        # Publish command
        cmd = Twist()
        if should_stop:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            self.get_logger().warn('Too many close obstacles - stopping')
        else:
            cmd.linear.x = linear_speed
            cmd.angular.z = angular_speed
            self.get_logger().info(
                f'Speed: {linear_speed:.2f}, Turn: {angular_speed:.2f}, '
                f'Clearest: {clearest_direction}')
        
        self.pub_.publish(cmd)
        
        # Log statistics using lambda
        stats = {
            'total': len(ranges),
            'valid': len(valid_ranges),
            'close': len(close_obstacles),
            'medium': len(medium_obstacles),
            'far': len(far_obstacles),
            'min': min_distance
        }
        
        stats_str = ', '.join(map(lambda item: f'{item[0]}={item[1]}', stats.items()))
        self.get_logger().info(f'Stats: {stats_str}')

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = LambdaRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
