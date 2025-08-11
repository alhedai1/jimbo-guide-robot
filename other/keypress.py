#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller')

        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.latest_cmd = Twist()
        self.emergency_stop = False

        # Start timer to process commands at fixed rate
        self.timer = rospy.Timer(rospy.Duration(0.05), self.update_motor_command)

        # Start timer to check keyboard input
        self.keyboard_timer = rospy.Timer(rospy.Duration(0.1), self.keyboard_check)

        rospy.loginfo("MotorController running. Press 's' to stop.")
        rospy.spin()

    def cmd_vel_callback(self, msg):
        self.latest_cmd = msg

    def update_motor_command(self, event):
        if self.emergency_stop:
            rpm_left, rpm_right = 0, 0
        else:
            rpm_left, rpm_right = self.convert_twist_to_rpm(self.latest_cmd)

        # Send RPM to motor driver over serial here
        rospy.loginfo(f"Sending RPM: L={rpm_left}, R={rpm_right}")

    def convert_twist_to_rpm(self, twist):
        # Replace with your real conversion logic
        linear = twist.linear.x
        angular = twist.angular.z
        # Dummy logic
        rpm_left = int(100 * (linear - angular))
        rpm_right = int(100 * (linear + angular))
        return rpm_left, rpm_right

    def keyboard_check(self, event):
        key = self.get_key_nonblocking()
        if key == 's':  # Emergency stop key
            self.emergency_stop = True
            rospy.logwarn("Emergency stop activated! Sending 0 RPM.")
        elif key == 'r':  # Resume
            self.emergency_stop = False
            rospy.loginfo("Resuming normal operation.")

    def get_key_nonblocking(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

if __name__ == '__main__':
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        MotorController()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
