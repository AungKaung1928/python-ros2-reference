# ===== GENERAL PYTHON =====
def general_example():
    # Basic try-except
    try:
        x = int(input("Enter a number: "))
        result = 10 / x
        print(f"Result: {result}")
    except ZeroDivisionError:
        print("Error: Cannot divide by zero!")
    except ValueError:
        print("Error: Invalid input! Please enter a number.")
    
    print("---")
    
    # Try-except-else-finally
    try:
        numbers = [1, 2, 3, 4, 5]
        index = int(input("Enter index: "))
        print(f"Value at index {index}: {numbers[index]}")
    except IndexError:
        print("Error: Index out of range!")
    except ValueError:
        print("Error: Invalid index!")
    else:
        print("Success: Value retrieved")
    finally:
        print("Finally block always executes")
    
    print("---")
    
    # Multiple exceptions
    try:
        file = open("data.txt", "r")
        content = file.read()
        value = int(content)
    except FileNotFoundError:
        print("Error: File not found!")
    except ValueError:
        print("Error: File content is not a number!")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        print("Cleanup operations")

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class SafeNavigationNode(Node):
    def __init__(self):
        super().__init__('safe_navigation')
        
        self.scan_sub_ = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_sub_ = self.create_subscription(
            String, 'command', self.command_callback, 10)
        
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.safe_distance_ = 0.5
        self.max_speed_ = 0.5
        
        self.get_logger().info('Safe navigation with exception handling started')
    
    def scan_callback(self, msg):
        try:
            # Check if scan data is valid
            if len(msg.ranges) == 0:
                raise ValueError("Empty scan data received")
            
            # Get front distance with bounds checking
            front_idx = len(msg.ranges) // 2
            if front_idx >= len(msg.ranges):
                raise IndexError("Invalid front index")
            
            front = msg.ranges[front_idx]
            
            # Check for invalid range values
            if front <= 0.0 or front > 100.0:
                raise ValueError(f"Invalid range value: {front}")
            
            # Safe navigation logic
            cmd = Twist()
            if front < self.safe_distance_:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.3
                self.get_logger().warn(f'Obstacle at {front:.2f}m - avoiding')
            else:
                cmd.linear.x = self.max_speed_
                self.get_logger().info(f'Clear path: {front:.2f}m')
            
            self.pub_.publish(cmd)
            
        except IndexError as e:
            self.get_logger().error(f'Index error in scan data: {e}')
            self.emergency_stop()
        except ValueError as e:
            self.get_logger().error(f'Invalid scan data: {e}')
            self.emergency_stop()
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            self.emergency_stop()
        finally:
            # This always runs - could be used for logging
            pass
    
    def command_callback(self, msg):
        try:
            command = msg.data.strip().lower()
            
            if command == "forward":
                self.move_forward()
            elif command == "stop":
                self.emergency_stop()
            elif command == "turn":
                self.turn()
            elif command.startswith("speed"):
                # Extract speed value
                parts = command.split()
                if len(parts) != 2:
                    raise ValueError("Invalid speed command format")
                
                speed = float(parts[1])
                if speed < 0 or speed > 1.0:
                    raise ValueError("Speed must be between 0 and 1.0")
                
                self.max_speed_ = speed
                self.get_logger().info(f'Speed set to {speed}')
            else:
                raise ValueError(f"Unknown command: {command}")
        
        except ValueError as e:
            self.get_logger().error(f'Command error: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected command error: {e}')
    
    def move_forward(self):
        try:
            cmd = Twist()
            cmd.linear.x = self.max_speed_
            self.pub_.publish(cmd)
            self.get_logger().info('Moving forward')
        except Exception as e:
            self.get_logger().error(f'Error moving forward: {e}')
    
    def turn(self):
        try:
            cmd = Twist()
            cmd.angular.z = 0.5
            self.pub_.publish(cmd)
            self.get_logger().info('Turning')
        except Exception as e:
            self.get_logger().error(f'Error turning: {e}')
    
    def emergency_stop(self):
        """Safe stop that always works"""
        try:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_.publish(cmd)
            self.get_logger().warn('EMERGENCY STOP')
        except Exception as e:
            self.get_logger().fatal(f'Critical: Emergency stop failed: {e}')

def main(args=None):
    # general_example()  # Skip for ROS2
    
    try:
        rclpy.init(args=args)
        node = SafeNavigationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutdown requested by user')
    except Exception as e:
        print(f'Fatal error: {e}')
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass
        print('Cleanup complete')

if __name__ == '__main__':
    main()
