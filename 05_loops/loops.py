# ===== GENERAL PYTHON =====
def general_example():
    # For loop - print table
    n = int(input("Enter a number: "))
    for i in range(1, 11):
        print(f"{n} x {i} = {n * i}")
    
    print("---")
    
    # While loop - keep dividing by 2
    num = int(input("Enter a number: "))
    while num != 1:
        print(num, end=" ")
        if num % 2 == 0:
            num = num // 2
        else:
            num = 3 * num + 1
    print(num)
    
    print("---")
    
    # For loop with list
    fruits = ["apple", "banana", "cherry"]
    for fruit in fruits:
        print(fruit)

# ===== ROS2 EXAMPLE =====
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class PeriodicPublisherNode(Node):
    def __init__(self):
        super().__init__('periodic_publisher')
        
        self.pub_ = self.create_publisher(String, 'status', 10)
        self.cmd_pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer creates a loop that runs periodically
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.count_ = 0
        
        self.get_logger().info('Periodic publisher started')
    
    def timer_callback(self):
        # This runs in a loop every 1 second
        msg = String()
        msg.data = f'Heartbeat #{self.count_}'
        self.pub_.publish(msg)
        
        self.get_logger().info(f'Published: {msg.data}')
        
        self.count_ += 1
        if self.count_ >= 10:
            self.count_ = 0  # Reset after 10
        
        # Send movement command
        cmd = Twist()
        cmd.linear.x = 0.2
        self.cmd_pub_.publish(cmd)

def main(args=None):
    # general_example()  # Skip for ROS2
    
    rclpy.init(args=args)
    node = PeriodicPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
