import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
  # ...existing code...

class TrackingCmd(Node):
    def __init__(self):
        super().__init__('tracking_cmd_node')
        self.publisher_ = self.create_publisher(Bool, 'tracking_cmd', 10)
        self.get_logger().info('Waiting for spacebar press...')

    def run(self):
        import threading
        self.enter_pressed = False

        def wait_for_enter():
            while rclpy.ok():
                input('Press Enter to send tracking command (Ctrl+C to exit)...')
                self.enter_pressed = True

        thread = threading.Thread(target=wait_for_enter, daemon=True)
        thread.start()

        while rclpy.ok():
            msg = Bool()
            if self.enter_pressed:
                msg.data = True
                self.enter_pressed = False
                self.get_logger().info('Published TRUE message!')
            else:
                msg.data = False
            self.publisher_.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            

def main(args=None):
    rclpy.init(args=args)
    node = TrackingCmd()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()