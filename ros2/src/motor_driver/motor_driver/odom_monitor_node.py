import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomMonitorNode(Node):
    """Prints the latest /odom pose on one updating line, for odometry
    accuracy testing (drive a known distance/turn, compare to reality)."""

    def __init__(self):
        super().__init__('odom_monitor_node')
        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.get_logger().info('Listening on /odom ... drive the robot.')

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # The driver only sets quaternion z, w (planar), so yaw = 2*atan2(z, w).
        yaw = math.degrees(2.0 * math.atan2(q.z, q.w))
        dist = math.hypot(p.x, p.y)
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        # \r keeps it on a single refreshing line.
        sys.stdout.write(
            f'\rx={p.x:+.3f} m  y={p.y:+.3f} m  yaw={yaw:+7.2f} deg  '
            f'dist={dist:.3f} m  v={v:+.3f} m/s  w={w:+.3f} rad/s   ')
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = OdomMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write('\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
