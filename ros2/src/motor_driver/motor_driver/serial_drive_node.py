import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class SerialDriveNode(Node):
    def __init__(self):
        super().__init__('serial_drive_node')
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        time.sleep(2)
        self.get_logger().info('Serial connected')

        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.5, self.watchdog)

    def watchdog(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 1.0:
            self.send('S')

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear  = msg.linear.x
        angular = msg.angular.z

        if abs(linear) < 0.1:
            linear = 0.0
        if abs(angular) < 0.1:
            angular = 0.0

        if linear == 0.0 and angular == 0.0:
            self.send('S')
            return

        left_pwm  = int((linear - angular) * 255)
        right_pwm = int((linear + angular) * 255)
        left_pwm  = max(-255, min(255, left_pwm))
        right_pwm = max(-255, min(255, right_pwm))

        self.send(f'M {left_pwm} {right_pwm}')

    def send(self, cmd):
        self.ser.write(f'{cmd}\n'.encode())
        self.get_logger().info(f'Sent: {cmd}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
