# ===== GENERAL PYTHON =====
def general_example():
    # Continue - skip even numbers
    print("Skip numbers divisible by 3:")
    for i in range(1, 21):
        if i % 3 == 0:
            continue
        print(i, end=" ")
    print()
    
    print("---")
    
    # Break - stop when condition met
    pocket_money = 3000
    for date in range(1, 31):
        if date % 2 == 0:
            continue  # Skip even dates
        if pocket_money == 0:
            break  # Stop when no money
        print(f"Go out on date {date}")
        pocket_money -= 300
    
    print("---")
    
    # Check prime number with break
    n = int(input("Enter a number: "))
    is_prime = True
    for i in range(2, n):
        if n % i == 0:
            is_prime = False
            break
    
    if is_prime and n > 1:
        print("Prime Number")
    else:
        print("Not a Prime Number")

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, LaserScan
from geometry_msgs.msg import Twist

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        
        self.battery_sub_ = self.create_subscription(
            BatteryState, 'battery', self.battery_callback, 10)
        self.scan_sub_ = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.battery_percent_ = 100.0
        self.get_logger().info('Battery monitor started')
    
    def battery_callback(self, msg):
        self.battery_percent_ = msg.percentage * 100.0
        self.get_logger().info(f'Battery: {self.battery_percent_:.1f}%')
    
    def scan_callback(self, msg):
        # Process scan data
        cmd = Twist()
        
        # Break equivalent - return early if low battery
        if self.battery_percent_ < 20.0:
            self.get_logger().warn('Low battery! Stopping.')
            cmd.linear.x = 0.0
            self.pub_.publish(cmd)
            return
        
        # Continue equivalent - skip processing for certain ranges
        valid_ranges = []
        for i, r in enumerate(msg.ranges):
            if i % 3 == 0:
                continue  # Skip every 3rd ray
            if r > 0.1 and r < 10.0:
                valid_ranges.append(r)
        
        # Move forward
        cmd.linear.x = 0.3
        self.pub_.publish(cmd)
        self.get_logger().info(f'Processed {len(valid_ranges)} valid ranges')

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
