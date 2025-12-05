# ===== GENERAL PYTHON =====
def general_example():
    # Create dictionary
    student = {
        "name": "Alice",
        "age": 22,
        "grade": "A",
        "city": "Tokyo"
    }
    print(f"Student: {student}")
    
    # Access values
    print(f"Name: {student['name']}")
    print(f"Age: {student.get('age')}")
    
    # Modify dictionary
    student["age"] = 23
    student["major"] = "Computer Science"
    print(f"Updated: {student}")
    
    # Dictionary methods
    print(f"Keys: {student.keys()}")
    print(f"Values: {student.values()}")
    print(f"Items: {student.items()}")
    
    # Iterate through dictionary
    for key, value in student.items():
        print(f"{key}: {value}")
    
    # Check if key exists
    if "name" in student:
        print("Name exists in dictionary")
    
    # Remove item
    student.pop("city")
    print(f"After pop: {student}")
    
    # Nested dictionary
    robots = {
        "robot1": {"type": "mobile", "speed": 0.5},
        "robot2": {"type": "arm", "dof": 6}
    }
    print(f"Robots: {robots}")

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState

class RobotStateNode(Node):
    def __init__(self):
        super().__init__('robot_state')
        
        # Dictionary to store robot state
        self.robot_state_ = {
            "mode": "idle",
            "battery": 100.0,
            "speed": 0.0,
            "position": {"x": 0.0, "y": 0.0},
            "status": "ready"
        }
        
        self.mode_sub_ = self.create_subscription(
            String, 'mode', self.mode_callback, 10)
        self.battery_sub_ = self.create_subscription(
            BatteryState, 'battery', self.battery_callback, 10)
        
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(2.0, self.status_callback)
        
        self.get_logger().info('Robot state node started')
    
    def mode_callback(self, msg):
        # Update state dictionary
        self.robot_state_["mode"] = msg.data
        self.get_logger().info(f'Mode changed to: {self.robot_state_["mode"]}')
        
        # Control based on mode
        cmd = Twist()
        if self.robot_state_["mode"] == "navigation":
            cmd.linear.x = 0.3
            self.robot_state_["speed"] = 0.3
        elif self.robot_state_["mode"] == "docking":
            cmd.linear.x = 0.1
            self.robot_state_["speed"] = 0.1
        else:
            cmd.linear.x = 0.0
            self.robot_state_["speed"] = 0.0
        
        self.pub_.publish(cmd)
    
    def battery_callback(self, msg):
        self.robot_state_["battery"] = msg.percentage * 100.0
        
        # Update status based on battery
        if self.robot_state_["battery"] < 20.0:
            self.robot_state_["status"] = "low_battery"
        elif self.robot_state_["battery"] < 50.0:
            self.robot_state_["status"] = "warning"
        else:
            self.robot_state_["status"] = "ready"
        
        self.get_logger().info(
            f'Battery: {self.robot_state_["battery"]:.1f}%, '
            f'Status: {self.robot_state_["status"]}')
    
    def status_callback(self):
        # Print entire state dictionary
        self.get_logger().info('=== Robot State ===')
        for key, value in self.robot_state_.items():
            self.get_logger().info(f'{key}: {value}')

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = RobotStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
