#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from jimbo_msgs.msg import MotorRPM
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster
import serial
import math

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')

        # Declare parameters
        self.declare_parameter('port', '/dev/motor_arduino')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.0635)
        self.declare_parameter('wheel_base', 0.3)

        # Get parameters
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now()

        # Serial
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Serial opened on {self.port} at {self.baudrate} baud.")
        except serial.SerialException:
            self.get_logger().error(f"Unable to open serial port {self.port}")
            raise

        # ROS interfaces
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.rpm_pub = self.create_publisher(MotorRPM, 'motor_rpm', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.rpm_cmd_sub = self.create_subscription(MotorRPM, 'motor_rpm_cmd', self.rpm_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.update_loop)  # 20Hz

    # read Twist command and send to arduino
    def cmd_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        # 속도 -> RPM 변환
        v_rpm = linear * 60 / (2 * math.pi * self.wheel_radius)
        a_rpm = angular * self.wheel_base * 60 / (2 * math.pi * self.wheel_radius)

        # 왼쪽 바퀴의 부호 반전
        left_rpm = -int(v_rpm - a_rpm / 2)
        right_rpm = int(v_rpm + a_rpm / 2)

        cmd = f"R{left_rpm},{right_rpm}\n"
        self.ser.write(cmd.encode())

    # read MotorRPM command and send to arduino
    def rpm_callback(self, msg: MotorRPM):
        # 왼쪽 RPM 부호 반전
        left_rpm = -int(msg.left_rpm)
        right_rpm = int(msg.right_rpm)
        cmd = f"R{left_rpm},{right_rpm}\n"
        self.get_logger().info(f"Manual RPM Command: {cmd.strip()}")
        self.ser.write(cmd.encode())

    # read encoder data from arduino (starts with "E:") and publish
    def update_loop(self):
        try:
            line = self.ser.readline().decode().strip()
        except UnicodeDecodeError:
            return

        if line.startswith("E:") and "," in line:
            try:
                data = line[2:]
                left_rpm_str, right_rpm_str = data.split(",")
                left_rpm = int(left_rpm_str)
                right_rpm = int(right_rpm_str)
                self.publish_odom(left_rpm, right_rpm)
                self.publish_motor_rpm(left_rpm, right_rpm)
            except ValueError:
                self.get_logger().warn("Failed to parse RPM data.")

    def publish_odom(self, left_rpm, right_rpm):
        now = self.get_clock().now()
        dt = max((now - self.last_time).nanoseconds / 1e9, 1e-6)
        self.last_time = now

        # 왼쪽 바퀴 부호 반전
        left_rpm_corrected = -left_rpm

        v = self.wheel_radius * (left_rpm_corrected + right_rpm) * math.pi / 60 / 2
        w = self.wheel_radius * (right_rpm - left_rpm_corrected) * math.pi / 60 / self.wheel_base

        dx = v * math.cos(self.theta) * dt
        dy = v * math.sin(self.theta) * dt
        dtheta = w * dt

        self.x += dx
        self.y += dy
        self.theta += dtheta

        q = R.from_euler('z', self.theta).as_quat()

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        # TF 브로드캐스트
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # 추가: base_link → laser_frame
        laser_tf = TransformStamped()
        laser_tf.header.stamp = now.to_msg()
        laser_tf.header.frame_id = 'base_link'
        laser_tf.child_frame_id = 'laser_frame'
        laser_tf.transform.translation.x = 0.2
        laser_tf.transform.translation.y = 0.0
        laser_tf.transform.translation.z = 0.1
        laser_tf.transform.rotation.x = 0.0
        laser_tf.transform.rotation.y = 0.0
        laser_tf.transform.rotation.z = 0.0
        laser_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(laser_tf)

        # 추가: base_link → camera_link
        cam_tf = TransformStamped()
        cam_tf.header.stamp = now.to_msg()
        cam_tf.header.frame_id = 'base_link'
        cam_tf.child_frame_id = 'camera_link'
        cam_tf.transform.translation.x = 0.1
        cam_tf.transform.translation.y = 0.0
        cam_tf.transform.translation.z = 0.15
        cam_tf.transform.rotation.x = 0.0
        cam_tf.transform.rotation.y = 0.0
        cam_tf.transform.rotation.z = 0.0
        cam_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(cam_tf)

    def publish_motor_rpm(self, left_rpm, right_rpm):
        # 실제 센서에서 읽은 값을 그대로 publish (부호 반전 안함)
        msg = MotorRPM()
        msg.left_rpm = left_rpm
        msg.right_rpm = right_rpm
        self.rpm_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
