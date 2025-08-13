import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_geometry_msgs
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Twist, Point, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time


class TrackingController(Node):
    def __init__(self):
        super().__init__('tracking_controller')

        self.uwb_data = Point()
        self.uwb_target = None
        self.start_tracking = False
        self.tracking = False

        self.intial = self.get_clock().now()

        self.uwb_sub = self.create_subscription(PointStamped, '/uwb_filtered_position', self.uwb_callback, 10)
        self.tracking_sub = self.create_subscription(Bool, '/start_tracking', self.tracking_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.goal_update_pub = self.create_publisher(PoseStamped, '/goal_update', 10)

        # TF2 buffer and listener for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def tracking_callback(self, msg):
        self.start_tracking = msg.data
        if self.start_tracking:
            self.get_logger().info(f"Start tracking UWB Target")
        else:
            self.get_logger().info(f"Stop tracking")

    def uwb_callback(self, msg: PointStamped):
        self.uwb_data = msg # uwb filtered position

        goal = PoseStamped()
        goal.header.frame_id = msg.header.frame_id
        goal.header.stamp = rclpy.time.Time().to_msg()
        goal.pose.position = msg.point
        goal_transformed = self.tf_buffer.transform(goal, 'odom', timeout=rclpy.duration.Duration(seconds=1.0))

        dist = math.hypot(msg.point.x, msg.point.y)
        # if dist > 2.0 and not self.tracking:
        #     self.start_tracking = True
        
        # extra (goal is always 1 m away from target based on BT, so nav2 will stop anyways)
        if (dist < 1.0 and self.tracking):
            self.tracking = False

        if self.start_tracking:
            self.start_tracking = False
            self.tracking = True
            self.goal_pub.publish(goal_transformed)
            # self.get_logger().info(f"Sent initial goal to nav2")
        elif self.tracking:
            self.goal_update_pub.publish(goal_transformed)
            # self.get_logger().info(f"Sent updated goal to nav2")
    

def main(args=None):
    rclpy.init()
    node = TrackingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

