import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import pygame
import sys
import math
import time as time_module

PINK = (255, 100, 130)
BG   = (15, 15, 25)

class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.awake_sub = self.create_subscription(
            Bool, '/robot_awake', self.awake_callback, 10)

        pygame.init()
        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        self.WIDTH, self.HEIGHT = self.screen.get_size()

        self.target_x = 0.0
        self.target_y = 0.0
        self.pupil_x  = 0.0
        self.pupil_y  = 0.0

        self.blink_timer    = 0
        self.blink_frame    = 0
        self.is_blinking    = False
        self.BLINK_INTERVAL = 90

        self.is_awake = False
        self.glow_intensity = 0.0
        self.glow_off_time = 0.0
        self.glow_angle = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.033, self.update_display)

    def cmd_vel_callback(self, msg):
        linear  = msg.linear.x
        angular = msg.angular.z

        if linear > 0.0:
            self.target_x =  0.0
            self.target_y = -1.0
        elif linear < 0.0:
            self.target_x =  0.0
            self.target_y =  1.0
        elif angular > 0.0:
            self.target_x = -1.0
            self.target_y =  0.0
        elif angular < 0.0:
            self.target_x =  1.0
            self.target_y =  0.0
        else:
            self.target_x =  0.0
            self.target_y =  0.0

        self.last_cmd_time = self.get_clock().now()

    def awake_callback(self, msg):
        self.is_awake = msg.data
        if msg.data:
            self.glow_off_time = time_module.time() + 5.0

    def update_display(self):
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN and event.key == pygame.K_q:
                pygame.quit()
                sys.exit()

        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 1.5:
            self.target_x = 0.0
            self.target_y = 0.0

        speed = 0.35
        self.pupil_x += (self.target_x - self.pupil_x) * speed
        self.pupil_y += (self.target_y - self.pupil_y) * speed

        self.blink_timer += 1
        if self.blink_timer >= self.BLINK_INTERVAL:
            self.is_blinking = True
            self.blink_frame += 1
            if self.blink_frame >= 6:
                self.is_blinking = False
                self.blink_frame = 0
                self.blink_timer = 0

        glow_active = self.is_awake and time_module.time() < self.glow_off_time

        if glow_active:
            self.glow_intensity = min(1.0, self.glow_intensity + 0.08)
            self.glow_angle += 0.12
        else:
            self.glow_intensity = max(0.0, self.glow_intensity - 0.04)

        self.draw()

    def draw_glow_orbit(self, cx, cy, eye_r):
        g = self.glow_intensity
        if g < 0.01:
            return

        ring_r = eye_r + 20

        # layered glow rings
        for thickness, alpha_mul, color in [
            (30, 0.15, (255, 80, 180)),
            (14, 0.35, (255, 100, 200)),
            (5,  0.9,  (255, 160, 220)),
        ]:
            surf = pygame.Surface((ring_r * 2 + 120, ring_r * 2 + 120), pygame.SRCALPHA)
            sc = ring_r + 60
            for t in range(thickness, 0, -1):
                alpha = int(255 * g * alpha_mul * (1.0 - t / thickness))
                pygame.draw.circle(surf, (*color, alpha), (sc, sc), ring_r + t, 2)
            self.screen.blit(surf, (cx - sc, cy - sc))
    
    def draw_eye(self, cx, cy, eye_r, pupil_r, iris_r, px, py, blink_frac):
        self.draw_glow_orbit(cx, cy, eye_r)

        # white of eye
        pygame.draw.circle(self.screen, (230, 230, 230), (cx, cy), eye_r)

        if blink_frac < 1.0:
            pygame.draw.circle(self.screen, PINK, (px, py), iris_r)
            pygame.draw.circle(self.screen, (10, 10, 10), (px, py), pupil_r)
            pygame.draw.circle(self.screen, (255, 255, 255), (px + 18, py - 18), 12)
            pygame.draw.circle(self.screen, (255, 255, 255), (px + 28, py - 8), 6)

        eyelid_h = int(eye_r * 2 * blink_frac)
        if eyelid_h > 0:
            eyelid_rect = pygame.Rect(cx - eye_r, cy - eye_r, eye_r * 2, eyelid_h)
            pygame.draw.ellipse(self.screen, BG, eyelid_rect)

        if self.glow_intensity > 0.01:
            g = self.glow_intensity
            outline_color = (
                min(255, int(180 + 75 * g)),
                min(255, int(60 + 40 * g)),
                min(255, int(90 + 50 * g))
            )
            pygame.draw.circle(self.screen, outline_color, (cx, cy), eye_r, 6)
        else:
            pygame.draw.circle(self.screen, PINK, (cx, cy), eye_r, 6)

    def draw(self):
        self.screen.fill(BG)

        CX = self.WIDTH  // 2
        CY = self.HEIGHT // 2

        eye_r   = 140
        iris_r  = 85
        pupil_r = 50
        travel  = eye_r - pupil_r - 15

        blink_frac = 0.0
        if self.is_blinking:
            if self.blink_frame <= 3:
                blink_frac = self.blink_frame / 3.0
            else:
                blink_frac = 1.0 - (self.blink_frame - 3) / 3.0

        for eye_cx in [CX - 200, CX + 200]:
            px = int(eye_cx + self.pupil_x * travel)
            py = int(CY     + self.pupil_y * travel)
            self.draw_eye(eye_cx, CY, eye_r, pupil_r, iris_r, px, py, blink_frac)

        pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
