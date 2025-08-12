import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')

        self.emergency_sub = self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel_smoothed', self.cmd_callback, 10) # cmd commands from nav2 (velocity_smoother)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)

        self.emergency = False

    def emergency_callback(self, msg):
        self.emergency = msg.data
        if self.emergency:
            self.get_logger().info(f"EMERGENCY STOP!")

    def cmd_callback(self, msg):
        if not self.emergency:
            self.cmd_pub.publish(msg)
        else:
            self.cmd_pub.publish(Twist()) # Stop moving if emergency

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
