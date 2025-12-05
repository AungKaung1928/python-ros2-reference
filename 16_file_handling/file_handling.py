# ===== GENERAL PYTHON =====
def general_example():
    # Write to file
    print("Writing to file...")
    with open("data.txt", "w") as file:
        file.write("Hello, Python!\n")
        file.write("File handling example\n")
        file.write("Line 3\n")
    print("File written successfully")
    
    print("---")
    
    # Read from file
    print("Reading from file...")
    with open("data.txt", "r") as file:
        content = file.read()
        print(content)
    
    print("---")
    
    # Read line by line
    print("Reading line by line...")
    with open("data.txt", "r") as file:
        for line in file:
            print(line.strip())
    
    print("---")
    
    # Append to file
    print("Appending to file...")
    with open("data.txt", "a") as file:
        file.write("Appended line\n")
    
    # Read all lines
    with open("data.txt", "r") as file:
        lines = file.readlines()
        print(f"Total lines: {len(lines)}")
    
    print("---")
    
    # Writing list to file
    numbers = [1, 2, 3, 4, 5]
    with open("numbers.txt", "w") as file:
        for num in numbers:
            file.write(f"{num}\n")
    
    # Reading list from file
    with open("numbers.txt", "r") as file:
        loaded_numbers = [int(line.strip()) for line in file]
        print(f"Loaded numbers: {loaded_numbers}")

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import json
import csv
from datetime import datetime

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger')
        
        self.scan_sub_ = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub_ = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # File paths
        self.log_file_ = "robot_log.txt"
        self.sensor_csv_ = "sensor_data.csv"
        self.config_file_ = "robot_config.json"
        
        # Initialize CSV file
        self.init_csv()
        
        # Load configuration
        self.load_config()
        
        # Log startup
        self.log_message("Robot data logger started")
        
        self.get_logger().info('Data logger node started')
    
    def init_csv(self):
        """Initialize CSV file with headers"""
        try:
            with open(self.sensor_csv_, "w", newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["timestamp", "front_distance", "left_distance", "right_distance"])
            self.get_logger().info(f'CSV file initialized: {self.sensor_csv_}')
        except Exception as e:
            self.get_logger().error(f'Error initializing CSV: {e}')
    
    def load_config(self):
        """Load robot configuration from JSON file"""
        try:
            # Create default config if not exists
            default_config = {
                "max_speed": 0.5,
                "safe_distance": 0.5,
                "turn_speed": 0.3,
                "log_interval": 1.0
            }
            
            try:
                with open(self.config_file_, "r") as file:
                    config = json.load(file)
                    self.max_speed_ = config.get("max_speed", 0.5)
                    self.safe_distance_ = config.get("safe_distance", 0.5)
                    self.get_logger().info(f'Config loaded: max_speed={self.max_speed_}')
            except FileNotFoundError:
                # Create default config file
                with open(self.config_file_, "w") as file:
                    json.dump(default_config, file, indent=4)
                self.max_speed_ = default_config["max_speed"]
                self.safe_distance_ = default_config["safe_distance"]
                self.get_logger().info('Default config created')
        
        except Exception as e:
            self.get_logger().error(f'Error loading config: {e}')
            self.max_speed_ = 0.5
            self.safe_distance_ = 0.5
    
    def log_message(self, message):
        """Append message to log file"""
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            with open(self.log_file_, "a") as file:
                file.write(f"[{timestamp}] {message}\n")
        except Exception as e:
            self.get_logger().error(f'Error writing log: {e}')
    
    def save_sensor_data(self, front, left, right):
        """Save sensor data to CSV"""
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            with open(self.sensor_csv_, "a", newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, front, left, right])
        except Exception as e:
            self.get_logger().error(f'Error writing CSV: {e}')
    
    def scan_callback(self, msg):
        try:
            ranges = msg.ranges
            if len(ranges) == 0:
                return
            
            # Extract distances
            front = ranges[len(ranges) // 2]
            left = ranges[len(ranges) // 4]
            right = ranges[3 * len(ranges) // 4]
            
            # Save to CSV
            self.save_sensor_data(front, left, right)
            
            # Navigation logic
            cmd = Twist()
            if front < self.safe_distance_:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.3
                self.log_message(f'Obstacle detected at {front:.2f}m - avoiding')
            else:
                cmd.linear.x = self.max_speed_
                self.log_message(f'Moving forward - front: {front:.2f}m')
            
            self.pub_.publish(cmd)
        
        except Exception as e:
            self.get_logger().error(f'Error in scan callback: {e}')
            self.log_message(f'ERROR: {e}')
    
    def odom_callback(self, msg):
        """Log odometry data periodically"""
        try:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.log_message(f'Position: x={x:.2f}, y={y:.2f}')
        except Exception as e:
            self.get_logger().error(f'Error in odom callback: {e}')
    
    def shutdown(self):
        """Save final statistics before shutdown"""
        try:
            stats = {
                "shutdown_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "total_runtime": "N/A"
            }
            
            with open("shutdown_stats.json", "w") as file:
                json.dump(stats, file, indent=4)
            
            self.log_message("Robot shutting down - statistics saved")
            self.get_logger().info('Shutdown complete - all data saved')
        
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = DataLoggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutdown requested')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
