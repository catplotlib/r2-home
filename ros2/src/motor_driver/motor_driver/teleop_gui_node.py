import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# How often we (re)publish the current command while a button/key is held.
# The serial driver's watchdog (serial_drive_node.py) stops the motors after
# ~1s of silence, so we must keep sending faster than that.
PUBLISH_HZ = 10.0


class TeleopGuiNode(Node):
    def __init__(self):
        super().__init__('teleop_gui_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Current commanded direction: -1, 0, or +1 on each axis.
        self.lin_dir = 0.0
        self.ang_dir = 0.0

        # Target speeds sent to the velocity controller (real units now).
        # Live-tunable: ros2 param set /teleop_gui_node linear_speed <m/s>
        self.declare_parameter('linear_speed', 0.15)   # m/s
        self.declare_parameter('angular_speed', 0.8)   # rad/s

        self.create_timer(1.0 / PUBLISH_HZ, self._publish)

    def set_motion(self, lin_dir, ang_dir):
        self.lin_dir = float(lin_dir)
        self.ang_dir = float(ang_dir)

    def stop(self):
        self.lin_dir = 0.0
        self.ang_dir = 0.0
        # Send an immediate stop rather than waiting for the next tick.
        self._publish()

    def _publish(self):
        linear_speed = self.get_parameter('linear_speed').value
        angular_speed = self.get_parameter('angular_speed').value
        msg = Twist()
        msg.linear.x = self.lin_dir * linear_speed
        msg.angular.z = self.ang_dir * angular_speed
        self.pub.publish(msg)


class TeleopGui:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Robot Control')
        self.root.configure(padx=12, pady=12)

        btn_opts = dict(width=6, height=2, font=('Sans', 14, 'bold'))

        # Directional pad laid out in a 3x3 grid.
        self.up = tk.Button(self.root, text='↑', **btn_opts)
        self.left = tk.Button(self.root, text='←', **btn_opts)
        self.stop_btn = tk.Button(self.root, text='■', fg='red', **btn_opts)
        self.right = tk.Button(self.root, text='→', **btn_opts)
        self.down = tk.Button(self.root, text='↓', **btn_opts)

        self.up.grid(row=0, column=1, padx=4, pady=4)
        self.left.grid(row=1, column=0, padx=4, pady=4)
        self.stop_btn.grid(row=1, column=1, padx=4, pady=4)
        self.right.grid(row=1, column=2, padx=4, pady=4)
        self.down.grid(row=2, column=1, padx=4, pady=4)

        # Hold-to-move: press starts motion, release stops.
        self._bind_hold(self.up, 1.0, 0.0)
        self._bind_hold(self.down, -1.0, 0.0)
        self._bind_hold(self.left, 0.0, 1.0)
        self._bind_hold(self.right, 0.0, -1.0)
        self.stop_btn.configure(command=self.node.stop)

        # Keyboard control: arrows and WASD. Space = stop.
        self._bind_keys()

        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    def _bind_hold(self, widget, lin, ang):
        widget.bind('<ButtonPress-1>',
                    lambda e: self.node.set_motion(lin, ang))
        widget.bind('<ButtonRelease-1>', lambda e: self.node.stop())

    def _bind_keys(self):
        keymap = {
            'Up': (1.0, 0.0), 'w': (1.0, 0.0),
            'Down': (-1.0, 0.0), 's': (-1.0, 0.0),
            'Left': (0.0, 1.0), 'a': (0.0, 1.0),
            'Right': (0.0, -1.0), 'd': (0.0, -1.0),
        }
        for key, (lin, ang) in keymap.items():
            self.root.bind(
                f'<KeyPress-{key}>',
                lambda e, l=lin, a=ang: self.node.set_motion(l, a))
            self.root.bind(f'<KeyRelease-{key}>', lambda e: self.node.stop())
        self.root.bind('<space>', lambda e: self.node.stop())

    def _on_close(self):
        self.node.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopGuiNode()

    # Spin ROS in a background thread; tkinter owns the main thread.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    gui = TeleopGui(node)
    try:
        gui.run()
    finally:
        node.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
