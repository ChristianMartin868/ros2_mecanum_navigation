#!/usr/bin/env python3
"""
Standalone 2D PyGame simulator for the mecanum corridor-following demo.

Recreates the environment described for the original Aut4 study project:
an outer wall and an inner rectangular wall, with the robot driving in the
corridor between them. Eight ToF distance sensors are computed by ray casting
against the walls and published as std_msgs/Float32MultiArray on robot1/tof.
The robot subscribes to robot1/cmd_vel and integrates its pose, so the C++
node `mecanum_navigation_node` closes the loop and the robot circles the
inner wall.

Sensor index layout (angle relative to robot heading):
    0: front (0deg)        1: back (180deg)
    2: +45deg              3: -45deg
    4: +90deg              5: -90deg
    6: +135deg             7: -135deg
The controller uses e = (tof[4]-tof[5]) + (tof[2]-tof[3]) to balance the
left/right sensor pairs and stay centred in the corridor.
"""
import math
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

import pygame

# --- World geometry ---------------------------------------------------------
# The verified layout below is the original 800px scene; change WIN to rescale
# the whole scene uniformly (keeps proportions, and thus the behaviour).
WIN = 700
_f = WIN / 800.0           # uniform scale factor
WIDTH = HEIGHT = WIN
SCALE = 50.0 * _f          # pixels per metre
MAX_RANGE_PX = 500.0 * _f  # ToF max range
ROBOT_RADIUS = 12.0 * _f

# Outer wall (inside of the border) and inner wall, as rectangles.
OUTER = pygame.Rect(round(50 * _f), round(50 * _f), round(700 * _f), round(700 * _f))
INNER = pygame.Rect(round(275 * _f), round(275 * _f), round(250 * _f), round(250 * _f))

# Sensor angle offsets relative to robot heading (radians).
SENSOR_OFFSETS = [
    0.0,            # 0 front
    math.pi,        # 1 back
    +math.pi / 4,   # 2 +45
    -math.pi / 4,   # 3 -45
    +math.pi / 2,   # 4 +90
    -math.pi / 2,   # 5 -90
    +3 * math.pi / 4,  # 6 +135
    -3 * math.pi / 4,  # 7 -135
]


def rect_segments(rect):
    """Return the four edges of a pygame.Rect as ((x1,y1),(x2,y2)) tuples."""
    tl = (rect.left, rect.top)
    tr = (rect.right, rect.top)
    br = (rect.right, rect.bottom)
    bl = (rect.left, rect.bottom)
    return [(tl, tr), (tr, br), (br, bl), (bl, tl)]


WALLS = rect_segments(OUTER) + rect_segments(INNER)


def ray_segment_dist(ox, oy, dx, dy, p1, p2):
    """Distance from ray origin (ox,oy) dir (dx,dy) to segment p1-p2, or None."""
    x1, y1 = p1
    x2, y2 = p2
    sx, sy = x2 - x1, y2 - y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-9:
        return None
    t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom   # along ray
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom   # along segment
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


def point_segment_dist(px, py, p1, p2):
    """Shortest distance from point to segment (for collision)."""
    x1, y1 = p1
    x2, y2 = p2
    sx, sy = x2 - x1, y2 - y1
    seg_len2 = sx * sx + sy * sy
    if seg_len2 < 1e-9:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * sx + (py - y1) * sy) / seg_len2))
    cx, cy = x1 + t * sx, y1 + t * sy
    return math.hypot(px - cx, py - cy)


class MecanumSim(Node):
    def __init__(self):
        super().__init__('mecanum_sim')

        # Robot pose (pixels, radians). Start in the left corridor heading up.
        self.x = 160.0 * _f
        self.y = 400.0 * _f
        self.theta = -math.pi / 2.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.pub_tof = self.create_publisher(Float32MultiArray, 'robot1/tof', 1)
        self.create_subscription(Twist, 'robot1/cmd_vel', self.cmd_cb, 1)

        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption('Mecanum corridor following')
        self.font = pygame.font.SysFont('monospace', 16)
        self.clock = pygame.time.Clock()
        self.last_t = self.get_clock().now()

        # ~60 Hz update loop.
        self.create_timer(1.0 / 60.0, self.update)
        self._log_accum = 0.0

    def cmd_cb(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

    def cast_all(self):
        """Return 8 ToF distances in metres."""
        out = []
        for off in SENSOR_OFFSETS:
            a = self.theta + off
            dx, dy = math.cos(a), math.sin(a)
            best = MAX_RANGE_PX
            for p1, p2 in WALLS:
                d = ray_segment_dist(self.x, self.y, dx, dy, p1, p2)
                if d is not None and d < best:
                    best = d
            out.append(best / SCALE)
        return out

    def min_wall_dist(self, x, y):
        return min(point_segment_dist(x, y, p1, p2) for p1, p2 in WALLS)

    def update(self):
        # quit handling
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (
                    ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                rclpy.shutdown()
                return

        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / 60.0

        # integrate pose (mecanum: body-frame vx/vy + yaw rate)
        ct, st = math.cos(self.theta), math.sin(self.theta)
        nx = self.x + (self.vx * ct - self.vy * st) * dt * SCALE
        ny = self.y + (self.vx * st + self.vy * ct) * dt * SCALE
        # simple soft collision: only move if we stay clear of walls
        if self.min_wall_dist(nx, ny) > ROBOT_RADIUS:
            self.x, self.y = nx, ny
        self.theta += self.wz * dt

        tof = self.cast_all()
        self.pub_tof.publish(Float32MultiArray(data=tof))

        self._log_accum += dt
        if self._log_accum >= 1.0:
            self._log_accum = 0.0
            self.get_logger().info(
                'pose x=%.0f y=%.0f th=%.2f  tof L(%.2f,%.2f) R(%.2f,%.2f)'
                % (self.x, self.y, self.theta, tof[2], tof[4], tof[3], tof[5]))

        self.render(tof)

    def render(self, tof):
        self.screen.fill((20, 20, 25))
        pygame.draw.rect(self.screen, (200, 200, 200), OUTER, 3)
        pygame.draw.rect(self.screen, (200, 120, 60), INNER, 0)

        # sensor rays
        for off, d in zip(SENSOR_OFFSETS, tof):
            a = self.theta + off
            ex = self.x + math.cos(a) * d * SCALE
            ey = self.y + math.sin(a) * d * SCALE
            pygame.draw.line(self.screen, (60, 90, 60),
                             (self.x, self.y), (ex, ey), 1)

        # robot
        pygame.draw.circle(self.screen, (80, 160, 255),
                           (int(self.x), int(self.y)), int(ROBOT_RADIUS))
        hx = self.x + math.cos(self.theta) * ROBOT_RADIUS * 1.8
        hy = self.y + math.sin(self.theta) * ROBOT_RADIUS * 1.8
        pygame.draw.line(self.screen, (255, 255, 255),
                         (self.x, self.y), (hx, hy), 2)

        txt = self.font.render('ESC quits', True, (180, 180, 180))
        self.screen.blit(txt, (10, 10))
        pygame.display.flip()
        self.clock.tick(60)


def main():
    rclpy.init(args=sys.argv)
    node = MecanumSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
