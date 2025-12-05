# ===== GENERAL PYTHON =====
import time

# Basic decorator
def my_decorator(func):
    def wrapper():
        print("Before function call")
        func()
        print("After function call")
    return wrapper

@my_decorator
def say_hello():
    print("Hello!")

# Decorator with arguments
def repeat(times):
    def decorator(func):
        def wrapper(*args, **kwargs):
            for _ in range(times):
                result = func(*args, **kwargs)
            return result
        return wrapper
    return decorator

@repeat(3)
def greet(name):
    print(f"Hello, {name}!")

# Timer decorator
def timer(func):
    def wrapper(*args, **kwargs):
        start = time.time()
        result = func(*args, **kwargs)
        end = time.time()
        print(f"{func.__name__} took {end - start:.4f} seconds")
        return result
    return wrapper

@timer
def slow_function():
    time.sleep(1)
    print("Function completed")

# Logger decorator
def logger(func):
    def wrapper(*args, **kwargs):
        print(f"Calling {func.__name__} with args={args}, kwargs={kwargs}")
        result = func(*args, **kwargs)
        print(f"{func.__name__} returned {result}")
        return result
    return wrapper

@logger
def add(a, b):
    return a + b

def general_example():
    say_hello()
    print("---")
    
    greet("Alice")
    print("---")
    
    slow_function()
    print("---")
    
    result = add(5, 3)
    print(f"Final result: {result}")

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from functools import wraps
import time

# Performance timer decorator
def performance_timer(func):
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        start = time.time()
        result = func(self, *args, **kwargs)
        elapsed = (time.time() - start) * 1000  # Convert to ms
        self.get_logger().debug(f'{func.__name__} took {elapsed:.2f}ms')
        return result
    return wrapper

# Error handler decorator
def safe_callback(func):
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        try:
            return func(self, *args, **kwargs)
        except Exception as e:
            self.get_logger().error(f'Error in {func.__name__}: {e}')
            self.emergency_stop()
    return wrapper

# Rate limiter decorator
def rate_limit(max_calls_per_second):
    def decorator(func):
        last_call = [0.0]
        min_interval = 1.0 / max_calls_per_second
        
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            current_time = time.time()
            if current_time - last_call[0] >= min_interval:
                last_call[0] = current_time
                return func(self, *args, **kwargs)
            else:
                self.get_logger().debug(f'{func.__name__} rate limited')
        return wrapper
    return decorator

# Validation decorator
def validate_range(min_val, max_val):
    def decorator(func):
        @wraps(func)
        def wrapper(self, value, *args, **kwargs):
            if not (min_val <= value <= max_val):
                self.get_logger().warn(
                    f'{func.__name__}: value {value} out of range [{min_val}, {max_val}]')
                value = max(min_val, min(max_val, value))
            return func(self, value, *args, **kwargs)
        return wrapper
    return decorator

# Logging decorator
def log_execution(func):
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        self.get_logger().info(f'Executing {func.__name__}')
        result = func(self, *args, **kwargs)
        self.get_logger().info(f'{func.__name__} completed')
        return result
    return wrapper

class DecoratorRobotNode(Node):
    def __init__(self):
        super().__init__('decorator_robot')
        
        self.scan_sub_ = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('Decorator robot node started')
    
    @safe_callback
    @performance_timer
    def scan_callback(self, msg):
        """Main scan processing with error handling and timing"""
        ranges = list(msg.ranges)
        
        if len(ranges) == 0:
            raise ValueError("Empty scan data")
        
        # Process scan
        front = ranges[len(ranges) // 2]
        left = ranges[len(ranges) // 4]
        right = ranges[3 * len(ranges) // 4]
        
        # Calculate movement
        linear, angular = self.calculate_motion(front, left, right)
        
        # Publish command
        self.publish_command(linear, angular)
    
    @performance_timer
    def calculate_motion(self, front, left, right):
        """Calculate motion with performance timing"""
        if front < 0.5:
            return 0.0, 0.5
        elif front < 1.0:
            return 0.1, 0.0
        else:
            turn = (left - right) * 0.3
            return 0.3, turn
    
    @validate_range(-1.0, 1.0)
    @rate_limit(10)  # Max 10 calls per second
    def publish_command(self, linear, angular=0.0):
        """Publish with validation and rate limiting"""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.pub_.publish(cmd)
        self.get_logger().info(f'Command: linear={linear:.2f}, angular={angular:.2f}')
    
    @log_execution
    def emergency_stop(self):
        """Emergency stop with logging"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_.publish(cmd)
        self.get_logger().warn('EMERGENCY STOP')
    
    @performance_timer
    @safe_callback
    def process_waypoint(self, x, y):
        """Process waypoint with multiple decorators"""
        self.get_logger().info(f'Processing waypoint: ({x}, {y})')
        # Waypoint processing logic
        time.sleep(0.1)  # Simulate processing
        return True

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = DecoratorRobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutdown requested')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
