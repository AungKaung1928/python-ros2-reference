# ===== GENERAL PYTHON =====
def general_example():
    # Basic generator
    def count_up_to(n):
        count = 1
        while count <= n:
            yield count
            count += 1
    
    print("Count up to 5:")
    for num in count_up_to(5):
        print(num, end=" ")
    print("\n---")
    
    # Generator for even numbers
    def even_numbers(n):
        for i in range(n):
            if i % 2 == 0:
                yield i
    
    print("Even numbers up to 10:")
    for num in even_numbers(10):
        print(num, end=" ")
    print("\n---")
    
    # Generator for fibonacci
    def fibonacci(n):
        a, b = 0, 1
        for _ in range(n):
            yield a
            a, b = b, a + b
    
    print("Fibonacci sequence (10 numbers):")
    for num in fibonacci(10):
        print(num, end=" ")
    print("\n---")
    
    # Generator expression
    squares = (x**2 for x in range(1, 6))
    print("Squares using generator expression:")
    for sq in squares:
        print(sq, end=" ")
    print("\n---")
    
    # Generator for reading large file (memory efficient)
    def read_large_file(filename):
        """Read file line by line without loading all into memory"""
        try:
            with open(filename, 'r') as file:
                for line in file:
                    yield line.strip()
        except FileNotFoundError:
            print(f"File {filename} not found")
    
    # Infinite generator
    def infinite_sequence():
        num = 0
        while True:
            yield num
            num += 1
    
    print("First 5 from infinite sequence:")
    gen = infinite_sequence()
    for _ in range(5):
        print(next(gen), end=" ")
    print()

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import time

class GeneratorRobotNode(Node):
    def __init__(self):
        super().__init__('generator_robot')
        
        self.scan_sub_ = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub_ = self.create_publisher(Path, 'path', 10)
        
        # Initialize waypoint generator
        self.waypoint_gen_ = self.waypoint_generator()
        
        # Timer to execute waypoints
        self.timer_ = self.create_timer(2.0, self.execute_next_waypoint)
        
        # Scan data generator
        self.scan_gen_ = None
        
        self.get_logger().info('Generator robot node started')
    
    def waypoint_generator(self):
        """Generator for waypoints - yields one at a time"""
        waypoints = [
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0)
        ]
        
        for x, y in waypoints:
            self.get_logger().info(f'Generated waypoint: ({x}, {y})')
            yield (x, y)
        
        self.get_logger().info('All waypoints generated')
    
    def scan_sector_generator(self, ranges, sector_size):
        """Generator to process scan data in sectors"""
        for i in range(0, len(ranges), sector_size):
            sector = ranges[i:i + sector_size]
            if sector:
                avg_distance = sum(sector) / len(sector)
                min_distance = min(sector)
                yield {
                    'index': i,
                    'avg': avg_distance,
                    'min': min_distance,
                    'size': len(sector)
                }
    
    def filtered_ranges_generator(self, ranges):
        """Generator to yield only valid ranges"""
        for i, r in enumerate(ranges):
            if 0.1 < r < 10.0:
                yield (i, r)
    
    def obstacle_generator(self, ranges, threshold=0.5):
        """Generator to yield obstacles below threshold"""
        for i, r in enumerate(ranges):
            if 0.1 < r < threshold:
                angle = -3.14 + (i * 6.28 / len(ranges))
                yield {
                    'index': i,
                    'distance': r,
                    'angle': angle
                }
    
    def path_planning_generator(self, start, goal, steps):
        """Generator for linear path interpolation"""
        x_start, y_start = start
        x_goal, y_goal = goal
        
        for i in range(steps + 1):
            t = i / steps
            x = x_start + t * (x_goal - x_start)
            y = y_start + t * (y_goal - y_start)
            yield (x, y)
    
    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        
        if len(ranges) == 0:
            return
        
        # Process scan in sectors using generator
        sector_gen = self.scan_sector_generator(ranges, sector_size=30)
        
        self.get_logger().info('=== Sector Analysis ===')
        for sector_data in sector_gen:
            self.get_logger().info(
                f"Sector {sector_data['index']}: "
                f"avg={sector_data['avg']:.2f}m, "
                f"min={sector_data['min']:.2f}m")
        
        # Find obstacles using generator
        obstacles = list(self.obstacle_generator(ranges, threshold=0.5))
        
        if obstacles:
            self.get_logger().warn(f'Found {len(obstacles)} obstacles')
            for obs in obstacles[:3]:  # Log first 3
                self.get_logger().warn(
                    f"Obstacle at index {obs['index']}: {obs['distance']:.2f}m")
        
        # Count valid ranges using generator
        valid_count = sum(1 for _ in self.filtered_ranges_generator(ranges))
        self.get_logger().info(f'Valid ranges: {valid_count}/{len(ranges)}')
        
        # Decision making
        cmd = Twist()
        if len(obstacles) > 10:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            self.get_logger().warn('Too many obstacles - turning')
        else:
            cmd.linear.x = 0.3
            self.get_logger().info('Path clear - moving forward')
        
        self.pub_.publish(cmd)
    
    def execute_next_waypoint(self):
        """Execute next waypoint from generator"""
        try:
            waypoint = next(self.waypoint_gen_)
            x, y = waypoint
            
            self.get_logger().info(f'Executing waypoint: ({x}, {y})')
            
            # Generate path to waypoint using generator
            current_pos = (0.0, 0.0)  # Simplified
            path_gen = self.path_planning_generator(current_pos, waypoint, steps=5)
            
            self.get_logger().info('Path points:')
            for px, py in path_gen:
                self.get_logger().info(f'  -> ({px:.2f}, {py:.2f})')
            
            # Move towards waypoint
            cmd = Twist()
            cmd.linear.x = 0.2
            self.pub_.publish(cmd)
            
        except StopIteration:
            self.get_logger().info('All waypoints completed - stopping')
            self.timer_.cancel()
            cmd = Twist()
            cmd.linear.x = 0.0
            self.pub_.publish(cmd)
    
    def infinite_heartbeat_generator(self):
        """Infinite generator for heartbeat"""
        count = 0
        while True:
            yield f'Heartbeat #{count}'
            count += 1

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = GeneratorRobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutdown requested')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
