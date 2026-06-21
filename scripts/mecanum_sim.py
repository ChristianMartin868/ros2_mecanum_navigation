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
left (2,4) and right (3,5) sensor pairs and stay centred in the corridor.
"""
import collections
import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

import pygame

# Sensor angle offsets relative to robot heading (radians).
SENSOR_OFFSETS = [
    0.0,               # 0 front
    math.pi,           # 1 back
    +math.pi / 4,      # 2 +45  (left pair, used by controller)
    -math.pi / 4,      # 3 -45  (right pair, used by controller)
    +math.pi / 2,      # 4 +90  (left pair, used by controller)
    -math.pi / 2,      # 5 -90  (right pair, used by controller)
    +3 * math.pi / 4,  # 6 +135
    -3 * math.pi / 4,  # 7 -135
]
LEFT_IDX = (2, 4)
RIGHT_IDX = (3, 5)

# Colours
C_BG = (16, 19, 28)
C_GRID = (26, 30, 42)
C_OUTER = (210, 210, 215)
C_INNER = (212, 130, 60)
C_INNER_EDGE = (150, 88, 38)
C_ROBOT = (80, 160, 255)
C_HEAD = (255, 255, 255)
C_TRAIL = (90, 210, 230)
C_LEFT = (90, 220, 130)
C_RIGHT = (240, 95, 95)
C_DIM = (70, 80, 95)
C_TEXT = (210, 214, 222)
C_PANEL = (0, 0, 0, 130)


def rect_segments(rect):
    """Return the four edges of a pygame.Rect as ((x1,y1),(x2,y2)) tuples."""
    tl = (rect.left, rect.top)
    tr = (rect.right, rect.top)
    br = (rect.right, rect.bottom)
    bl = (rect.left, rect.bottom)
    return [(tl, tr), (tr, br), (br, bl), (bl, tl)]


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

        pygame.init()
        # Auto-fit a square window to the available screen height.
        screen_h = pygame.display.Info().current_h
        win = max(480, min(760, screen_h - 110))
        self.win = win
        f = win / 800.0   # uniform scale over the verified 800px layout

        self.scale = 50.0 * f          # pixels per metre
        self.max_range_px = 500.0 * f  # ToF max range
        self.robot_radius = 12.0 * f

        m = round(50 * f)
        s = round(700 * f)
        self.outer = pygame.Rect(m, m, s, s)
        i0 = round(275 * f)
        isz = round(250 * f)
        self.inner = pygame.Rect(i0, i0, isz, isz)
        self.walls = rect_segments(self.outer) + rect_segments(self.inner)

        self.screen = pygame.display.set_mode((win, win))
        pygame.display.set_caption('Mecanum corridor following')
        fs = max(12, int(win * 0.022))
        self.font = pygame.font.SysFont('monospace', fs)
        self.font_b = pygame.font.SysFont('monospace', fs, bold=True)
        self.clock = pygame.time.Clock()
        self.last_t = self.get_clock().now()

        # Robot pose (pixels, radians). Start in the left corridor heading up.
        self.x = 160.0 * f
        self.y = 400.0 * f
        self.theta = -math.pi / 2.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.trail = collections.deque(maxlen=700)

        self.pub_tof = self.create_publisher(Float32MultiArray, 'robot1/tof', 1)
        self.create_subscription(Twist, 'robot1/cmd_vel', self.cmd_cb, 1)

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
            best = self.max_range_px
            for p1, p2 in self.walls:
                d = ray_segment_dist(self.x, self.y, dx, dy, p1, p2)
                if d is not None and d < best:
                    best = d
            out.append(best / self.scale)
        return out

    def min_wall_dist(self, x, y):
        return min(point_segment_dist(x, y, p1, p2) for p1, p2 in self.walls)

    def update(self):
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
        nx = self.x + (self.vx * ct - self.vy * st) * dt * self.scale
        ny = self.y + (self.vx * st + self.vy * ct) * dt * self.scale
        if self.min_wall_dist(nx, ny) > self.robot_radius:
            self.x, self.y = nx, ny
        self.theta += self.wz * dt
        self.trail.append((self.x, self.y))

        tof = self.cast_all()
        self.pub_tof.publish(Float32MultiArray(data=tof))

        self._log_accum += dt
        if self._log_accum >= 1.0:
            self._log_accum = 0.0
            self.get_logger().info(
                'pose x=%.0f y=%.0f th=%.2f  tof L(%.2f,%.2f) R(%.2f,%.2f)'
                % (self.x, self.y, self.theta, tof[2], tof[4], tof[3], tof[5]))

        self.render(tof)

    # ---- rendering ---------------------------------------------------------
    def render(self, tof):
        sc = self.screen
        sc.fill(C_BG)
        self._draw_grid()

        # inner wall (filled, with edge) and outer border
        pygame.draw.rect(sc, C_INNER, self.inner)
        pygame.draw.rect(sc, C_INNER_EDGE, self.inner, 3)
        pygame.draw.rect(sc, C_OUTER, self.outer, 3)

        self._draw_trail()
        self._draw_sensors(tof)
        self._draw_robot()
        self._draw_hud(tof)

        pygame.display.flip()
        self.clock.tick(60)

    def _draw_grid(self):
        step = max(30, int(self.win / 16))
        for x in range(0, self.win, step):
            pygame.draw.line(self.screen, C_GRID, (x, 0), (x, self.win))
        for y in range(0, self.win, step):
            pygame.draw.line(self.screen, C_GRID, (0, y), (self.win, y))

    def _draw_trail(self):
        pts = list(self.trail)
        n = len(pts)
        if n < 2:
            return
        for i in range(1, n):
            a = i / n  # 0 (old) .. 1 (recent)
            col = (int(C_TRAIL[0] * a), int(C_TRAIL[1] * a), int(C_TRAIL[2] * a))
            pygame.draw.line(self.screen, col, pts[i - 1], pts[i], 2)

    def _draw_sensors(self, tof):
        for idx, (off, d) in enumerate(zip(SENSOR_OFFSETS, tof)):
            if idx in LEFT_IDX:
                col = C_LEFT
            elif idx in RIGHT_IDX:
                col = C_RIGHT
            else:
                col = C_DIM
            a = self.theta + off
            ex = self.x + math.cos(a) * d * self.scale
            ey = self.y + math.sin(a) * d * self.scale
            pygame.draw.line(self.screen, col, (self.x, self.y), (ex, ey), 1)
            pygame.draw.circle(self.screen, col, (int(ex), int(ey)), 3)

    def _draw_robot(self):
        r = int(self.robot_radius)
        pygame.draw.circle(self.screen, C_ROBOT, (int(self.x), int(self.y)), r)
        hx = self.x + math.cos(self.theta) * r * 1.9
        hy = self.y + math.sin(self.theta) * r * 1.9
        pygame.draw.line(self.screen, C_HEAD, (self.x, self.y), (hx, hy), 2)

    def _draw_hud(self, tof):
        left = tof[2] + tof[4]
        right = tof[3] + tof[5]
        e = (tof[4] - tof[5]) + (tof[2] - tof[3])
        if self.wz > 0:
            turn = 'turn +w'
        elif self.wz < 0:
            turn = 'turn -w'
        else:
            turn = 'straight'

        lines = [
            'ESC quits',
            'v = %.2f m/s   w = %+.2f rad/s' % (self.vx, self.wz),
            'e = %+.2f  -> %s' % (e, turn),
            'L(2,4) = %.2f m   R(3,5) = %.2f m' % (left, right),
            '%.0f fps' % self.clock.get_fps(),
        ]
        pad = 8
        w = max(self.font.size(t)[0] for t in lines) + 2 * pad
        h = len(lines) * (self.font.get_height() + 2) + 2 * pad
        panel = pygame.Surface((w, h), pygame.SRCALPHA)
        panel.fill(C_PANEL)
        self.screen.blit(panel, (6, 6))
        y = 6 + pad
        for i, t in enumerate(lines):
            font = self.font_b if i == 0 else self.font
            self.screen.blit(font.render(t, True, C_TEXT), (6 + pad, y))
            y += self.font.get_height() + 2

        self._draw_balance_bars(left, right)

    def _draw_balance_bars(self, left, right):
        """Two bars (top-right) showing the left vs right pair distances."""
        bw = int(self.win * 0.16)
        bh = 14
        x0 = self.win - bw - 14
        y0 = 14
        maxv = max(1.0, left, right, 4.0)
        for label, val, col, yo in (('L', left, C_LEFT, 0),
                                    ('R', right, C_RIGHT, bh + 8)):
            y = y0 + yo
            pygame.draw.rect(self.screen, (40, 44, 56), (x0, y, bw, bh))
            fillw = int(bw * min(1.0, val / maxv))
            pygame.draw.rect(self.screen, col, (x0, y, fillw, bh))
            txt = self.font.render('%s %.1f' % (label, val), True, C_TEXT)
            self.screen.blit(txt, (x0 - txt.get_width() - 6, y - 2))


def main():
    rclpy.init(args=sys.argv)
    node = MecanumSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        pygame.quit()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
