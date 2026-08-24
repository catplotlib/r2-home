import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Empty
from rclpy.duration import Duration
import tf2_ros
import serial
import time
import math

WHEEL_DIAMETER = 0.06436  # calibrated: 1.00 m actual read as 1.010 m at 0.065
WHEEL_BASE     = 0.197    # calibrated via 5-turn test (1800 deg read as 1830 at 0.194)
WHEEL_RADIUS   = WHEEL_DIAMETER / 2
# 12 PPR hall encoder x 2 (both edges of channel A) x 90:1 gearbox = 2160 counts/wheel-rev.
PPR1           = 2160.0
PPR2           = 2160.0

# TF: broadcast odom->base_link on its own 50 Hz timer (decoupled from the
# ~20 Hz serial read) and forward-date the stamp so tf2 never extrapolates
# forward for the future-stamped LiDAR scan. Required for SLAM/Nav2 TF chain
# across the two machines.
TF_PUBLISH_HZ  = 50.0
TF_FUTURE_DATE = 0.2   # seconds to forward-date the TF stamp

# Closed-loop wheel-velocity control: PWM = feedforward(target) + PI(error).
# cmd_vel is interpreted as real m/s / rad/s (what Nav2 sends). Live-tunable
# via `ros2 param set /serial_drive_node <name> <value>`.
VEL_KV = 500.0   # feedforward gain: PWM per (m/s) of target speed
VEL_KS = 70.0    # static feedforward: PWM to overcome stiction (signed, per wheel)
VEL_KP = 300.0   # proportional gain: PWM per (m/s) of velocity error
VEL_KI = 600.0   # integral gain: PWM per (m/s * s)

# Below this |target| a wheel is treated as commanded-stop.
VEL_DEADBAND = 0.01


class SerialDriveNode(Node):
    def __init__(self):
        super().__init__('serial_drive_node')
        self.ser = None
        self.connect()

        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.reset_sub = self.create_subscription(
            Empty, '/reset_odom', self.reset_odom_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.last_cmd_time = self.get_clock().now()
        self.last_odom_time = self.get_clock().now()
        self.create_timer(0.5, self.watchdog)
        self.create_timer(0.05, self.read_serial)
        self.create_timer(1.0 / TF_PUBLISH_HZ, self.publish_tf)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_enc1 = 0
        self.last_enc2 = 0

        # Velocity-control state.
        self.target_v_left = 0.0   # m/s
        self.target_v_right = 0.0
        self.integ_left = 0.0      # integral accumulators (m/s * s)
        self.integ_right = 0.0
        self.last_pwm = (0, 0)     # last PWM actually sent (for send-on-change)
        self.log_counter = 0

        # Live-tunable PI + feedforward gains.
        self.declare_parameter('vel_kv', VEL_KV)
        self.declare_parameter('vel_ks', VEL_KS)
        self.declare_parameter('vel_kp', VEL_KP)
        self.declare_parameter('vel_ki', VEL_KI)

    def connect(self):
        ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0']
        for port in ports:
            try:
                self.ser = serial.Serial(port, 9600, timeout=1)
                time.sleep(2)
                self.get_logger().info(f'Serial connected on {port}')
                return
            except Exception:
                continue
        self.get_logger().error('No Arduino found!')

    def watchdog(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 1.0:
            # Lost the command stream — stop and clear the controller.
            self.target_v_left = 0.0
            self.target_v_right = 0.0
            self.integ_left = 0.0
            self.integ_right = 0.0
            if self.last_pwm != (0, 0):
                self.send('S')
                self.last_pwm = (0, 0)

    def read_serial(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('E '):
                    parts = line.split()
                    if len(parts) == 5:
                        enc1 = int(parts[1])
                        enc2 = int(parts[2])
                        self.update_odometry(enc1, enc2)
        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')

    def reset_odom_callback(self, msg):
        # Zero the pose but keep last_enc* so subsequent tick deltas stay valid.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.get_logger().info('Odometry reset to (0, 0, 0)')

    def update_odometry(self, enc1, enc2):
        d_enc1 = enc1 - self.last_enc1
        d_enc2 = enc2 - self.last_enc2
        self.last_enc1 = enc1
        self.last_enc2 = enc2

        left_dist  = (d_enc1 / PPR1) * math.pi * WHEEL_DIAMETER
        right_dist = (d_enc2 / PPR2) * math.pi * WHEEL_DIAMETER

        center_dist = (left_dist + right_dist) / 2.0
        delta_theta = (right_dist - left_dist) / WHEEL_BASE

        self.x     += center_dist * math.cos(self.theta + delta_theta / 2.0)
        self.y     += center_dist * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now

        if dt <= 0:
            return

        linear_vel  = center_dist / dt
        angular_vel = delta_theta / dt
        v_left_meas  = left_dist / dt
        v_right_meas = right_dist / dt

        self.publish_odom(now, linear_vel, angular_vel)

        # Skip the velocity controller on abnormally short samples (buffered
        # serial lines) where measured velocity would be noise.
        if dt < 0.02:
            return

        pwm_left = self.wheel_control(
            self.target_v_left, v_left_meas, 'integ_left', dt)
        pwm_right = self.wheel_control(
            self.target_v_right, v_right_meas, 'integ_right', dt)
        self.actuate(pwm_left, pwm_right)

        self.log_counter += 1
        if self.log_counter % 10 == 0:  # ~2 Hz
            self.get_logger().info(
                f'L tgt={self.target_v_left:+.3f} meas={v_left_meas:+.3f} pwm={pwm_left} | '
                f'R tgt={self.target_v_right:+.3f} meas={v_right_meas:+.3f} pwm={pwm_right} | '
                f'pose x={self.x:.2f} y={self.y:.2f} th={math.degrees(self.theta):.0f}deg')

    def wheel_control(self, target_v, meas_v, integ_attr, dt):
        """Feedforward + PI velocity controller for one wheel. Returns PWM."""
        if abs(target_v) < VEL_DEADBAND:
            setattr(self, integ_attr, 0.0)
            return 0

        kv = self.get_parameter('vel_kv').value
        ks = self.get_parameter('vel_ks').value
        kp = self.get_parameter('vel_kp').value
        ki = self.get_parameter('vel_ki').value

        error = target_v - meas_v
        sign = 1.0 if target_v > 0 else -1.0
        integ = getattr(self, integ_attr)
        new_integ = integ + error * dt

        pwm_unsat = kv * target_v + ks * sign + kp * error + ki * new_integ
        pwm = max(-255.0, min(255.0, pwm_unsat))
        # Anti-windup: only accumulate the integral when the output isn't saturated.
        if pwm == pwm_unsat:
            setattr(self, integ_attr, new_integ)
        return int(pwm)

    def actuate(self, pwm_left, pwm_right):
        """Send a motor command, throttled to changes (spares the 9600 link)."""
        if pwm_left == 0 and pwm_right == 0:
            if self.last_pwm != (0, 0):
                self.send('S')
                self.last_pwm = (0, 0)
            return
        if (abs(pwm_left - self.last_pwm[0]) > 2 or
                abs(pwm_right - self.last_pwm[1]) > 2):
            self.send(f'M {pwm_left} {pwm_right}')
            self.last_pwm = (pwm_left, pwm_right)

    def publish_odom(self, now, linear_vel, angular_vel):
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x  = linear_vel
        odom.twist.twist.angular.z = angular_vel
        self.odom_pub.publish(odom)

    def publish_tf(self):
        # Broadcast odom->base_link at a steady 50 Hz from the latest integrated
        # pose, stamped slightly ahead so the scan's (future) timestamp always
        # falls within tf2's buffer instead of past its newest entry.
        stamp = self.get_clock().now() + Duration(seconds=TF_FUTURE_DATE)
        tf = TransformStamped()
        tf.header.stamp = stamp.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = math.sin(self.theta / 2.0)
        tf.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(tf)

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear  = msg.linear.x   # m/s
        angular = msg.angular.z  # rad/s

        if abs(linear) < VEL_DEADBAND:
            linear = 0.0
        if abs(angular) < 0.02:
            angular = 0.0

        # Diff-drive inverse kinematics -> per-wheel ground speed (m/s).
        half = WHEEL_BASE / 2.0
        self.target_v_left  = linear - angular * half
        self.target_v_right = linear + angular * half

    def send(self, cmd):
        if self.ser is None or not self.ser.is_open:
            self.connect()
            return
        try:
            self.ser.write(f'{cmd}\n'.encode())
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')
            self.ser = None
            self.connect()


def main(args=None):
    rclpy.init(args=args)
    node = SerialDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
