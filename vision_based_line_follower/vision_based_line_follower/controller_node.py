#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import serial

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.sub = self.create_subscription(Float32, '/line_offset', self.cb, 10)
        self.pub = self.create_publisher(Twist, '/motor_cmd', 10)
        self.cmdvel_pub = self.create_publisher(Twist, '/cmd_vel', 10)


        self.declare_parameter('kp', 0.01)
        self.declare_parameter('kd', 0.002)
        self.declare_parameter('base_speed', 80)
        self.declare_parameter('use_sim', True)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.base_speed = self.get_parameter('base_speed').value
        self.use_sim = self.get_parameter('use_sim').value

        if not self.use_sim:
            self.ser = serial.Serial(self.get_parameter('serial_port').value,
                                     self.get_parameter('baudrate').value,
                                     timeout=0.1)

        self.last_error = 0.0

    def cb(self, msg: Float32):
        error = msg.data
        derivative = error - self.last_error
        self.last_error = error
        correction = self.kp*error + self.kd*derivative

        left_pwm = int(self.base_speed - correction)
        right_pwm = int(self.base_speed + correction)

        left_pwm = max(min(left_pwm, 127), -127)
        right_pwm = max(min(right_pwm, 127), -127)

        cmd = Twist()
        cmd.linear.x = float((left_pwm + right_pwm)/2.0)
        cmd.angular.z = float(correction)
        self.pub.publish(cmd)

        if self.use_sim:
            self.cmdvel_pub.publish(cmd)  # also send to Gazebo
        else:
            s = f"M,{left_pwm},{right_pwm}\n"
            self.ser.write(s.encode())

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if not node.use_sim:
        node.ser.close()
    node.destroy_node()
    rclpy.shutdown()
