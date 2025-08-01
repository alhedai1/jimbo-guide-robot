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
import signal
import sys
from pymodbus.client.serial import ModbusSerialClient as ModbusClient

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')

        # Declare parameters
        self.declare_parameter('port', '/dev/motor_usb')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.0635)
        self.declare_parameter('wheel_base', 0.6) # Distance between left and right wheels

        # Get parameters
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now()

        self.odom_count = 0

        # initialize PID variables
        self.target_rpm_left = 0.0
        self.target_rpm_right = 0.0
        self.integral_left = 0.0
        self.integral_right = 0.0
        self.prev_error_left = 0.0
        self.prev_error_right = 0.0

        # with these gains, dwa kinda works (very slow, passes the goal and keeps going though)
        # for bso_hfc, only kp = 1.0 was working
        self.Kp = 0.1
        self.Ki = 0.01
        self.Kd = 0.0

        self.dt = 0.05 # 20 hz

        # Serial
        try:
            # self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.client = ModbusClient(port=self.port, baudrate=115200)
            self.unit_id = 1  # Modbus slave ID of ZLAC8015D
            self.client.connect()
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

        # enable both motors
        self.client.write_register(address=0x200E, value=8, device_id=self.unit_id)
        # set speed mode
        self.client.write_register(address=0x200D, value=3, device_id=self.unit_id)

        signal.signal(signal.SIGINT, self.handle_sigint)

    # read Twist command and send to arduino
    def cmd_callback(self, msg: Twist):
        # self.get_logger().info(f"Received cmd_vel: x = {msg.linear.x}, z = {msg.angular.z}")
        linear = msg.linear.x
        angular = msg.angular.z

        # 속도 -> RPM 변환
        v_rpm = linear * 60 / (2 * math.pi * self.wheel_radius)
        a_rpm = angular * self.wheel_base * 60 / (2 * math.pi * self.wheel_radius)

        # 왼쪽 바퀴의 부호 반전
        left_rpm = -(v_rpm - a_rpm / 2)
        right_rpm = (v_rpm + a_rpm / 2)

        # # write to motors directly
        # self.client.write_register(address=0x2088, value=left_rpm & 0xFFFF, device_id=self.unit_id)
        # self.client.write_register(address=0x2089, value=right_rpm & 0xFFFF, device_id=self.unit_id)

        self.target_rpm_left = left_rpm
        self.target_rpm_right = right_rpm
        # self.get_logger().info(f"target rpm: left = {self.target_rpm_left}, right = {self.target_rpm_right}")
        
        ### Add PID CONTROL

    # read MotorRPM command and send to arduino
    def rpm_callback(self, msg: MotorRPM):
        # 왼쪽 RPM 부호 반전
        left_rpm = -round(msg.left_rpm)
        right_rpm = round(msg.right_rpm)
        self.target_rpm_left = left_rpm
        self.target_rpm_right = right_rpm
        # cmd = f"R{left_rpm},{right_rpm}\n"  #cccc
        # self.get_logger().info(f"Manual RPM Command: {cmd.strip()}")
        # self.ser.write(cmd.encode())

    # read encoder data from arduino (starts with "E:") and publish
    def update_loop(self):      #cccc
        try:
            # line = self.ser.readline().decode().strip()
            resp1 = self.client.read_holding_registers(address=0x20AB, count=1, device_id=self.unit_id)
            resp2 = self.client.read_holding_registers(address=0x20AC, count=1, device_id=self.unit_id)
            if resp1.isError() or resp2.isError():
                return
            actual_rpm_left = resp1.registers[0]
            actual_rpm_right = resp2.registers[0]
            if actual_rpm_left > 32767:
                actual_rpm_left -= 65536
            if actual_rpm_right > 32767:
                actual_rpm_right -= 65536
            actual_rpm_left = (actual_rpm_left * 0.1)
            actual_rpm_right = (actual_rpm_right * 0.1)
            # self.get_logger().info(f"actual left: {actual_rpm_left}, actual right: {actual_rpm_right}")
            
            error_left = self.target_rpm_left - actual_rpm_left
            self.integral_left += error_left *self.dt
            derivative_left = (error_left - self.prev_error_left) / self.dt
            output_left = self.Kp * error_left + self.Ki * self.integral_left + self.Kd * derivative_left
            self.prev_error_left = error_left

            error_right = self.target_rpm_right - actual_rpm_right
            self.integral_right += error_right * self.dt
            derivative_right = (error_right - self.prev_error_right) / self.dt
            output_right = self.Kp * error_right + self.Ki * self.integral_right + self.Kd * derivative_right
            self.prev_error_right = error_right

            output_left = max(min((output_left), 300), -300)
            output_right = max((min((output_right), 300), -300))
            # self.get_logger().info(f"output: ({output_left}, {output_right}), actual: ({actual_rpm_left}, {actual_rpm_right})")

            # deadband (no correction)
            if abs(error_left) < 0.1:
                output_left = 0.0
            if abs(error_right) < 0.1:
                output_right = 0.0

            self.client.write_register(address=0x2088, value=round(output_left) & 0xFFFF, device_id=self.unit_id)
            self.client.write_register(address=0x2089, value=round(output_right) & 0xFFFF, device_id=self.unit_id)

            if self.odom_count > 30:
                self.publish_odom(round(actual_rpm_left), round(actual_rpm_right))
            self.odom_count += 1
            self.publish_motor_rpm(round(actual_rpm_left), round(actual_rpm_right))
    
        except UnicodeDecodeError:
            return

        # if line.startswith("E:") and "," in line:
        #     try:
        #         data = line[2:]
        #         left_rpm_str, right_rpm_str = data.split(",")
        #         left_rpm = round(left_rpm_str)
        #         right_rpm = round(right_rpm_str)
        #         self.publish_odom(left_rpm, right_rpm)
        #         self.publish_motor_rpm(left_rpm, right_rpm)
        #     except ValueError:
        #         self.get_logger().warn("Failed to parse RPM data.")

    def publish_odom(self, left_rpm, right_rpm):
        now = self.get_clock().now()
        dt = max((now - self.last_time).nanoseconds / 1e9, 1e-6)
        self.last_time = now

        # 왼쪽 바퀴 부호 반전
        left_rpm_corrected = -left_rpm

        v = self.wheel_radius * (left_rpm_corrected + right_rpm) * 2 * math.pi / 60 / 2
        w = self.wheel_radius * (right_rpm - left_rpm_corrected) * 2 * math.pi / 60 / self.wheel_base

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
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # Add covariance matrices for navigation
        # Pose covariance (6x6 matrix)
        odom.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,  # y
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,  # z
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,  # roll
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,  # pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1   # yaw
        ]
        
        # Twist covariance (6x6 matrix)
        odom.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # linear x
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,  # linear y
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,  # linear z
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,  # angular x
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,  # angular y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1   # angular z
        ]

        self.odom_pub.publish(odom)

        # TF 브로드캐스트 - odom to base_footprint (dynamic transform)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # # 추가: base_link → laser_frame
        # laser_tf = TransformStamped()
        # laser_tf.header.stamp = now.to_msg()
        # laser_tf.header.frame_id = 'base_link'
        # laser_tf.child_frame_id = 'laser_frame'
        # laser_tf.transform.translation.x = 0.2
        # laser_tf.transform.translation.y = 0.0
        # laser_tf.transform.translation.z = 0.1
        # laser_tf.transform.rotation.x = 0.0
        # laser_tf.transform.rotation.y = 0.0
        # laser_tf.transform.rotation.z = 0.0
        # laser_tf.transform.rotation.w = 1.0
        # self.tf_broadcaster.sendTransform(laser_tf)

        # # 추가: base_link → camera_link
        # cam_tf = TransformStamped()
        # cam_tf.header.stamp = now.to_msg()
        # cam_tf.header.frame_id = 'base_link'
        # cam_tf.child_frame_id = 'camera_link'
        # cam_tf.transform.translation.x = 0.1
        # cam_tf.transform.translation.y = 0.0
        # cam_tf.transform.translation.z = 0.15
        # cam_tf.transform.rotation.x = 0.0
        # cam_tf.transform.rotation.y = 0.0
        # cam_tf.transform.rotation.z = 0.0
        # cam_tf.transform.rotation.w = 1.0
        # self.tf_broadcaster.sendTransform(cam_tf)

    def publish_motor_rpm(self, left_rpm, right_rpm):
        # 실제 센서에서 읽은 값을 그대로 publish (부호 반전 안함)
        msg = MotorRPM()
        msg.left_rpm = left_rpm
        msg.right_rpm = right_rpm
        self.rpm_pub.publish(msg)

    def handle_sigint(self, signum, frame):
        try:
            self.client.write_register(address=0x2088, value=0 & 0xFFFF, device_id=self.unit_id)
            self.client.write_register(address=0x2089, value=0 & 0xFFFF, device_id=self.unit_id)
            self.client.write_register(address=0x200E, value=0, device_id=self.unit_id)
            self.client.close()
            self.get_logger().info("Sent stop command to motors on SIGINT.")
        except Exception as e:
            self.get_logger().warn(f"Failed to send stop command on SIGINT: {e}")
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
