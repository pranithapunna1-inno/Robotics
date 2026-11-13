"""
robot_sim.py - 2D robot navigation & obstacle-avoidance simulation (Pygame).

Run:  python robot_sim.py

Structure
---------
  Robot       - differential-drive kinematics, noisy + filtered distance sensors
  Simulation  - game state and the three control modes
                  manual   : keyboard (+ optional avoidance assist)
                  auto     : A* path planning + proportional path follower
                  reactive : sensors only (go-to-goal + wall-following, "Bug"-style)
  Renderer    - everything that draws to the screen
  main()      - window, event loop

Control pipeline (every frame):
  1. Sensors are read (optionally noisy, then low-pass filtered)
  2. A mode produces a command  (throttle, turn)  in [-1, 1]
  3. The command is converted to LEFT / RIGHT WHEEL speeds (with acceleration
     limits) and the robot moves; walls clamp it, obstacles block it
"""

import math
import random
import sys

import pygame

from layouts import ARENA_H, ARENA_W, LAYOUTS
from navigation import GridMap, cast_ray, circle_hits_any, clamp, wrap_angle

# ----------------------------------------------------------------------------
# Window / layout constants
# ----------------------------------------------------------------------------
HUD_H = 100                       # status bar height
FOOTER_H = 48                     # controls hint lines
MARGIN = 20
WIN_W = ARENA_W + 2 * MARGIN
WIN_H = HUD_H + ARENA_H + FOOTER_H
ARENA_POS = (MARGIN, HUD_H)       # where the arena is drawn on the window
FPS = 60

MODE_NAMES = {"manual": "Manual", "auto": "A* (autonomous)", "reactive": "Reactive (sensors only)"}
MODE_ORDER = ["manual", "auto", "reactive"]

# Colours
BG = (24, 26, 32)
ARENA_BG = (38, 42, 52)
GRID = (46, 51, 63)
WALL = (200, 205, 215)
OBSTACLE = (178, 84, 74)
OBSTACLE_EDGE = (225, 130, 115)
START_COL = (80, 200, 120)
GOAL_COL = (245, 200, 60)
ROBOT_COL = (70, 150, 240)
ROBOT_HIT = (240, 70, 70)
PATH_COL = (120, 190, 255)
TEXT = (235, 237, 242)
DIM = (150, 156, 170)
BUTTON = (58, 64, 82)
BUTTON_HOVER = (78, 86, 110)


# ----------------------------------------------------------------------------
# Robot
# ----------------------------------------------------------------------------
class Robot:
    """A circular differential-drive robot with five distance sensors.

    Two independently driven wheels sit on either side of the body:
        forward speed   v = (v_left + v_right) / 2
        turn rate       w = (v_left - v_right) / TRACK     (+ = turn right/clockwise)
    Wheel speeds change gradually (WHEEL_ACCEL), so the robot has momentum and
    turns in smooth arcs instead of snapping to a new heading.
    """

    RADIUS = 14
    TRACK = 2 * RADIUS                         # distance between the wheels
    MAX_SPEED = 170.0                          # max wheel speed, pixels / second
    TURN_RATE = 180.0                          # commanded turn rate, degrees / second
    WHEEL_ACCEL = 700.0                        # pixels / second^2
    # Sensors: (angle relative to heading, sideways offset). Negative = left.
    #   index 0-4: rays from the body centre at -60, -30, 0, +30, +60 degrees
    #   index 5,6: "whiskers" - parallel rays from the left / right edge of the
    #              body, so obstacles that would clip the robot's width are seen
    SENSOR_SPECS = ((-60, 0), (-30, 0), (0, 0), (30, 0), (60, 0),
                    (0, -RADIUS), (0, RADIUS))
    SENSOR_RANGE = 130.0                       # pixels (measured from robot edge)
    NOISE_SIGMA = 4.0                          # std-dev of sensor noise, pixels
    FILTER_ALPHA = 0.6                         # low-pass filter weight of new reading

    def __init__(self, x, y, heading):
        self.x, self.y, self.heading = float(x), float(y), float(heading)
        self.v_left = 0.0
        self.v_right = 0.0
        self.true_sensors = [self.SENSOR_RANGE] * len(self.SENSOR_SPECS)
        self.sensors = list(self.true_sensors)   # what the controllers actually use

    # -- sensing -------------------------------------------------------------------
    def sensor_rays(self):
        """Start point and absolute angle of every sensor ray: [(x, y, angle_deg)].

        Centre rays start at the body's edge; whiskers start at the front corners.
        """
        out = []
        h = math.radians(self.heading)
        for rel, lateral in self.SENSOR_SPECS:
            ang = self.heading + rel
            if lateral == 0:
                sx = self.x + math.cos(math.radians(ang)) * self.RADIUS
                sy = self.y + math.sin(math.radians(ang)) * self.RADIUS
            else:
                # forward by RADIUS, then sideways (heading + 90 deg = robot's right)
                sx = self.x + math.cos(h) * self.RADIUS + math.cos(h + math.pi / 2) * lateral
                sy = self.y + math.sin(h) * self.RADIUS + math.sin(h + math.pi / 2) * lateral
            out.append((sx, sy, ang))
        return out

    def read_sensors(self, obstacles, noise=False, reset_filter=False):
        """Cast one ray per sensor; distances are measured from the robot's edge.

        With noise=True each reading gets Gaussian error. A simple exponential
        moving-average filter (new = a*reading + (1-a)*previous) smooths it again.
        """
        true = []
        for sx, sy, ang in self.sensor_rays():
            d = cast_ray(sx, sy, ang, self.SENSOR_RANGE, obstacles, ARENA_W, ARENA_H)
            true.append(d)
        self.true_sensors = true

        if not noise:
            self.sensors = list(true)
            return self.sensors
        raw = [clamp(d + random.gauss(0, self.NOISE_SIGMA), 0.0, self.SENSOR_RANGE)
               for d in true]
        if reset_filter:
            self.sensors = raw
        else:
            a = self.FILTER_ALPHA
            self.sensors = [a * n + (1 - a) * old for n, old in zip(raw, self.sensors)]
        return self.sensors

    # -- motion --------------------------------------------------------------------
    def drive(self, throttle, turn, dt, speed_scale, obstacles):
        """Convert a (throttle, turn) command into wheel speeds and move.

        Returns (distance_moved, hit_obstacle).
        """
        v_cmd = throttle * self.MAX_SPEED * speed_scale
        w_cmd = math.radians(turn * self.TURN_RATE)            # rad/s, + = clockwise
        left_target = v_cmd + w_cmd * self.TRACK / 2
        right_target = v_cmd - w_cmd * self.TRACK / 2

        # Respect the motor limit while keeping the left/right ratio (same curve).
        limit = self.MAX_SPEED * speed_scale
        biggest = max(abs(left_target), abs(right_target))
        if biggest > limit:
            k = limit / biggest
            left_target *= k
            right_target *= k

        # Wheels can only accelerate so fast.
        max_dv = self.WHEEL_ACCEL * dt
        self.v_left += clamp(left_target - self.v_left, -max_dv, max_dv)
        self.v_right += clamp(right_target - self.v_right, -max_dv, max_dv)

        # Differential-drive kinematics.
        v = (self.v_left + self.v_right) / 2
        w = (self.v_left - self.v_right) / self.TRACK
        self.heading = wrap_angle(self.heading + math.degrees(w) * dt)

        moved, hit = (self.move(v * dt, obstacles) if v else (0.0, False))
        if hit:
            self.v_left = self.v_right = 0.0     # bumped: wheels stall
        return moved, hit

    def move(self, distance, obstacles):
        """Move along the heading by `distance` pixels.

        Returns (distance_actually_moved, hit_obstacle).
          * Arena boundaries: the new position is clamped so the robot's edge
            never leaves the arena.
          * Obstacles: if the move would overlap one, we try sliding along each
            axis separately; if both are blocked the robot stays put.
        """
        r = self.RADIUS
        rad = math.radians(self.heading)
        nx = clamp(self.x + math.cos(rad) * distance, r, ARENA_W - r)
        ny = clamp(self.y + math.sin(rad) * distance, r, ARENA_H - r)

        hit = False
        if circle_hits_any(nx, ny, r, obstacles):
            hit = True
            if not circle_hits_any(nx, self.y, r, obstacles):
                ny = self.y                     # slide horizontally
            elif not circle_hits_any(self.x, ny, r, obstacles):
                nx = self.x                     # slide vertically
            else:
                nx, ny = self.x, self.y         # fully blocked

        moved = math.hypot(nx - self.x, ny - self.y)
        self.x, self.y = nx, ny
        return moved, hit


# ----------------------------------------------------------------------------
# Simulation (all logic, no drawing)
# ----------------------------------------------------------------------------
class Simulation:
    SAFE_DIST = 45.0       # start slowing / steering when something is this close
    STOP_DIST = 6.0        # throttle reaches zero here
    SIDE_DIST = 22.0       # +/-30 degree rays: closer than this also triggers avoidance
    GOAL_RADIUS = 22.0
    PLAN_CLEARANCE = Robot.RADIUS + 8   # robot radius + safety margin for A*

    # Reactive (sensor-only) mode tuning
    REACT_DIST = 50.0      # obstacle this close ahead -> start wall following
    WALL_DIST = 38.0       # desired centre-to-wall distance while wall following
    WALL_SPEED = 0.75      # throttle while following a wall

    def __init__(self, layout_index=0):
        self.layout_index = layout_index
        self.mode = "manual"
        self.assist = True            # avoidance layer active in manual mode
        self.noise = True             # noisy (then filtered) sensors
        self.show_path = True
        self.show_sensors = True
        self.speed_scale = 1.0
        self.records = {}             # (layout, mode) -> result of the last finished run
        self.load_layout(layout_index)

    # -- setup ---------------------------------------------------------------------
    def load_layout(self, index):
        self.layout_index = index % len(LAYOUTS)
        layout = LAYOUTS[self.layout_index]
        self.name = layout["name"]
        self.obstacles = layout["obstacles"]
        self.start = layout["start"]
        self.goal = layout["goal"]
        self.grid = GridMap(ARENA_W, ARENA_H, self.obstacles, self.PLAN_CLEARANCE)
        self.reset()

    def reset(self):
        layout = LAYOUTS[self.layout_index]
        self.robot = Robot(*self.start, layout["start_heading"])
        self.robot.read_sensors(self.obstacles, self.noise, reset_filter=True)
        self.elapsed = 0.0
        self.collisions = 0
        self.avoid_events = 0
        self.distance = 0.0
        self.finished = False
        self.paused = False
        self.in_collision = False
        self.hit_flash = 0.0
        self.avoiding = False
        self.avoid_dir = 0            # -1 = turning left, +1 = right, 0 = none
        self.escape_timer = 0.0
        self.stuck_timer = 0.0
        self.stuck_ref = (self.robot.x, self.robot.y)
        self.path = []
        self.path_index = 0
        self.plan_failed = False
        # reactive-mode state machine
        self.rstate = "goal"          # "goal" = drive to goal, "wall" = follow a wall
        self.wall_side = 1            # +1 wall on the right, -1 wall on the left
        self.hit_dist = 0.0           # distance to goal when the wall was first met
        if self.mode == "auto":
            self.plan_path()

    def set_mode(self, mode):
        self.mode = mode
        self.reset()

    def cycle_mode(self):
        self.set_mode(MODE_ORDER[(MODE_ORDER.index(self.mode) + 1) % len(MODE_ORDER)])

    @property
    def state_text(self):
        if self.mode == "reactive":
            if self.rstate == "goal":
                return "heading to goal"
            return f"following wall ({'right' if self.wall_side == 1 else 'left'})"
        return ""

    def plan_path(self):
        """Run A* from the robot's current position to the goal."""
        path = self.grid.find_path((self.robot.x, self.robot.y), self.goal)
        self.path = path or []
        self.path_index = 0
        self.plan_failed = path is None

    # -- shared helpers --------------------------------------------------------------
    def check_stuck(self, dt):
        """True if the robot has barely moved for 1.5 s (call once per frame)."""
        rb = self.robot
        self.stuck_timer += dt
        if self.stuck_timer < 1.5:
            return False
        moved = math.hypot(rb.x - self.stuck_ref[0], rb.y - self.stuck_ref[1])
        self.stuck_timer = 0.0
        self.stuck_ref = (rb.x, rb.y)
        return moved < 8

    def ahead(self):
        """Clear distance in front of the robot's whole width (centre + both whiskers)."""
        s = self.robot.sensors
        return min(s[2], s[5], s[6])

    def goal_info(self):
        """Distance to the goal and heading error toward it (like a GPS + compass)."""
        rb = self.robot
        dx, dy = self.goal[0] - rb.x, self.goal[1] - rb.y
        bearing = math.degrees(math.atan2(dy, dx))
        return math.hypot(dx, dy), bearing, wrap_angle(bearing - rb.heading)

    @staticmethod
    def drive_toward(err):
        """Proportional steering: returns (throttle, turn) for a heading error."""
        turn = clamp(err / 30.0, -1.0, 1.0)
        throttle = max(0.0, math.cos(math.radians(err))) if abs(err) < 90 else 0.0
        if abs(err) < 60:
            throttle = max(throttle, 0.2)
        return throttle, turn

    # -- mode: A* ------------------------------------------------------------------------
    def auto_command(self, dt):
        """Follow the A* waypoints. Returns (throttle, turn) in [-1, 1]."""
        rb = self.robot

        # Back up briefly after being stuck, then re-plan.
        if self.escape_timer > 0:
            self.escape_timer -= dt
            return -0.6, 0.0
        if not self.path:
            return 0.0, 0.0

        # Advance to the next waypoint once we are close to the current one.
        while (self.path_index < len(self.path) - 1 and
               math.hypot(self.path[self.path_index][0] - rb.x,
                          self.path[self.path_index][1] - rb.y) < 18):
            self.path_index += 1
        tx, ty = self.path[self.path_index]

        desired = math.degrees(math.atan2(ty - rb.y, tx - rb.x))
        throttle, turn = self.drive_toward(wrap_angle(desired - rb.heading))

        if self.check_stuck(dt):
            self.escape_timer = 0.5
            rb.heading += random.uniform(-25, 25)
            self.plan_path()
        return throttle, turn

    # -- mode: reactive (sensors only) ---------------------------------------------------
    def reactive_command(self, dt):
        """Bug-style navigation using only sensors + the goal direction (no map).

        State "goal": turn toward the goal and drive.
        State "wall": an obstacle blocks the way, so keep it on one side and
                      follow its edge (like tracing a wall with your hand).
        Leave the wall when the direct line to the goal is clear AND we are
        closer to the goal than where we first hit the obstacle.
        """
        rb = self.robot
        s = rb.sensors                     # [-60, -30, 0, +30, +60, whisker L, whisker R]
        R = Robot.RADIUS
        centre = self.ahead()
        dist_goal, bearing, err = self.goal_info()

        # Back up briefly after being stuck, and try the other side next time.
        if self.escape_timer > 0:
            self.escape_timer -= dt
            return -0.6, 0.0
        if self.check_stuck(dt):
            self.escape_timer = 0.5
            self.wall_side = -self.wall_side
            self.rstate = "wall"
            self.hit_dist = dist_goal
            return -0.6, 0.0

        front_blocked = centre < self.REACT_DIST or min(s[1], s[3]) < 28

        if self.rstate == "goal":
            if not front_blocked:
                return self.drive_toward(err)
            # Obstacle ahead: start wall following. Keep the wall on the side
            # AWAY from the goal so we go around it the short way.
            self.rstate = "wall"
            self.hit_dist = dist_goal
            left_room = s[0] + s[1] + s[5]
            right_room = s[3] + s[4] + s[6]
            if abs(right_room - left_room) > 25 or abs(err) < 10:
                # Obstacle clearly on one side: keep it on that side and steer away.
                self.wall_side = 1 if right_room < left_room else -1
            else:
                self.wall_side = -1 if err > 0 else 1

        # ---- wall following ----
        # Can we leave? Cast three rays toward the goal; all must be clear.
        look = min(dist_goal, 80.0)
        goal_clear = all(
            cast_ray(rb.x, rb.y, bearing + off, look + R, self.obstacles,
                     ARENA_W, ARENA_H) >= look + R
            for off in (-25, 0, 25))
        if goal_clear and dist_goal < self.hit_dist - 15:
            self.rstate = "goal"
            return self.drive_toward(err)

        side = self.wall_side
        near, far = (3, 4) if side == 1 else (1, 0)
        # Perpendicular distance to the wall from the two side rays (L * sin(angle)).
        wall_p = min((s[near] + R) * math.sin(math.radians(30)),
                     (s[far] + R) * math.sin(math.radians(60)))
        # Slow down as the front fills up, or when an obstacle corner is close to
        # either front-diagonal sensor (lets the robot rotate away before moving on).
        scale = clamp((centre - self.STOP_DIST) / (self.REACT_DIST - self.STOP_DIST), 0.0, 1.0)
        corner = clamp((min(s[1], s[3]) - 4.0) / 24.0, 0.0, 1.0)
        speed = self.WALL_SPEED * min(scale, max(corner, 0.1))
        if centre < self.REACT_DIST:
            return speed, float(-side)                        # turn away from the wall
        turn = side * clamp((wall_p - self.WALL_DIST) / 25.0, -1.0, 1.0)
        return speed, turn

    # -- avoidance layer (manual + A*) ------------------------------------------------------
    def avoidance(self, throttle, turn, override_turn):
        """Reactive obstacle avoidance using the distance sensors.

        * Slows down in proportion to how close the centre sensor reading is.
        * Picks the more open side (left vs right) and steers that way.
        * The chosen direction is remembered until the way ahead is clear, which
          stops the robot from jittering between left and right.
        """
        s = self.robot.sensors            # [-60, -30, 0, +30, +60, whisker L, whisker R]
        centre = self.ahead()
        blocked = (centre < self.SAFE_DIST or s[1] < self.SIDE_DIST
                   or s[3] < self.SIDE_DIST)

        if blocked:
            if self.avoid_dir == 0:
                left_room, right_room = s[0] + s[1] + s[5], s[3] + s[4] + s[6]
                self.avoid_dir = -1 if left_room >= right_room else 1
            if throttle > 0:
                scale = clamp((centre - self.STOP_DIST) /
                              (self.SAFE_DIST - self.STOP_DIST), 0.0, 1.0)
                throttle *= scale
            if override_turn:
                turn = float(self.avoid_dir)
        elif centre > self.SAFE_DIST + 15:
            self.avoid_dir = 0            # path is clear again: forget the decision

        if blocked and not self.avoiding:
            self.avoid_events += 1
        self.avoiding = blocked
        return throttle, turn

    # -- main update ------------------------------------------------------------------------
    def update(self, dt, key_throttle=0.0, key_turn=0.0):
        """Advance the simulation by dt seconds."""
        if self.finished or self.paused:
            return
        dt = min(dt, 1 / 30)              # avoid huge steps after a lag spike
        self.elapsed += dt
        self.hit_flash = max(0.0, self.hit_flash - dt)
        rb = self.robot

        rb.read_sensors(self.obstacles, self.noise)

        if self.mode == "auto":
            throttle, turn = self.auto_command(dt)
            # The avoidance layer only takes over steering when an obstacle is
            # really close (the A* path normally keeps us clear).
            critical = self.ahead() < self.SAFE_DIST * 0.6
            throttle, turn = self.avoidance(throttle, turn, override_turn=critical)
        elif self.mode == "reactive":
            throttle, turn = self.reactive_command(dt)
            wall = self.rstate == "wall"
            if wall and not self.avoiding:
                self.avoid_events += 1
            self.avoiding = wall
        else:
            throttle, turn = key_throttle, key_turn
            if self.assist:
                throttle, turn = self.avoidance(throttle, turn,
                                                override_turn=abs(turn) < 0.1)
            else:
                self.avoiding = False

        # Apply the command through the differential-drive model.
        moved, hit = rb.drive(throttle, turn, dt, self.speed_scale, self.obstacles)
        self.distance += moved
        if hit and not self.in_collision:
            self.collisions += 1
            self.hit_flash = 0.3
        self.in_collision = hit

        # Goal check.
        if math.hypot(rb.x - self.goal[0], rb.y - self.goal[1]) <= self.GOAL_RADIUS:
            self.finished = True
            self.records[(self.layout_index, self.mode)] = (
                self.elapsed, self.collisions, self.distance / 10)


# ----------------------------------------------------------------------------
# Rendering
# ----------------------------------------------------------------------------
class Button:
    def __init__(self, rect, action, label_fn):
        self.rect = pygame.Rect(rect)
        self.action = action
        self.label = label_fn


class Renderer:
    def __init__(self, screen):
        self.screen = screen
        self.arena_surf = pygame.Surface((ARENA_W, ARENA_H))
        names = "segoeui,arial,helvetica,dejavusans"
        self.font = pygame.font.SysFont(names, 18)
        self.small = pygame.font.SysFont(names, 14)
        self.big = pygame.font.SysFont(names, 44, bold=True)
        self.tick = 0.0

    def text(self, surf, msg, pos, font=None, color=TEXT, center=False):
        img = (font or self.font).render(msg, True, color)
        rect = img.get_rect(center=pos) if center else img.get_rect(topleft=pos)
        surf.blit(img, rect)

    # -- arena ---------------------------------------------------------------------------
    def draw_arena(self, sim):
        a = self.arena_surf
        a.fill(ARENA_BG)
        for x in range(0, ARENA_W, 40):
            pygame.draw.line(a, GRID, (x, 0), (x, ARENA_H))
        for y in range(0, ARENA_H, 40):
            pygame.draw.line(a, GRID, (0, y), (ARENA_W, y))

        for rect in sim.obstacles:
            pygame.draw.rect(a, OBSTACLE, rect)
            pygame.draw.rect(a, OBSTACLE_EDGE, rect, 2)

        # Start marker
        sx, sy = sim.start
        pygame.draw.circle(a, START_COL, (sx, sy), 24, 3)
        self.text(a, "START", (sx, sy + 38), self.small, START_COL, center=True)

        # Goal marker (pulsing ring)
        gx, gy = sim.goal
        pulse = 4 * math.sin(self.tick * 4)
        pygame.draw.circle(a, GOAL_COL, (gx, gy), int(Simulation.GOAL_RADIUS), 0)
        pygame.draw.circle(a, GOAL_COL, (gx, gy), int(Simulation.GOAL_RADIUS + 8 + pulse), 2)
        self.text(a, "GOAL", (gx, gy + 40), self.small, GOAL_COL, center=True)

        rb = sim.robot
        # Planned path (A* mode) or goal direction (reactive mode)
        if sim.show_path:
            if sim.mode == "auto" and sim.path:
                pts = [(rb.x, rb.y)] + sim.path[sim.path_index:]
                if len(pts) > 1:
                    pygame.draw.lines(a, PATH_COL, False, pts, 2)
                for p in sim.path[sim.path_index:]:
                    pygame.draw.circle(a, PATH_COL, (int(p[0]), int(p[1])), 4)
            elif sim.mode == "reactive":
                pygame.draw.line(a, (90, 100, 125), (rb.x, rb.y), (gx, gy), 1)

        # Sensors (filtered readings - what the controller actually sees)
        if sim.show_sensors:
            for (sx, sy, ang_deg), dist in zip(rb.sensor_rays(), rb.sensors):
                ang = math.radians(ang_deg)
                start = (sx, sy)
                end = (sx + math.cos(ang) * dist, sy + math.sin(ang) * dist)
                closeness = 1 - dist / Robot.SENSOR_RANGE      # 0 = far, 1 = touching
                color = (int(80 + 175 * closeness), int(220 - 150 * closeness), 90)
                pygame.draw.line(a, color, start, end, 2)
                if dist < Robot.SENSOR_RANGE - 1:
                    pygame.draw.circle(a, color, (int(end[0]), int(end[1])), 4)

        # Robot body + heading line
        body = ROBOT_HIT if sim.hit_flash > 0 else ROBOT_COL
        center = (int(rb.x), int(rb.y))
        pygame.draw.circle(a, body, center, Robot.RADIUS)
        pygame.draw.circle(a, (255, 255, 255), center, Robot.RADIUS, 2)
        ang = math.radians(rb.heading)
        pygame.draw.line(a, (255, 255, 255), center,
                         (rb.x + math.cos(ang) * Robot.RADIUS * 1.4,
                          rb.y + math.sin(ang) * Robot.RADIUS * 1.4), 3)
        # Wheels (little bars; brighter = faster)
        for sign, speed in ((-1, rb.v_left), (1, rb.v_right)):
            # sign -1 = left wheel, +1 = right wheel (relative to heading)
            px = rb.x + math.cos(ang + sign * math.pi / 2) * Robot.RADIUS
            py = rb.y + math.sin(ang + sign * math.pi / 2) * Robot.RADIUS
            level = int(clamp(abs(speed) / Robot.MAX_SPEED, 0, 1) * 155)
            wheel_col = (100 + level, 100 + level, 100) if speed >= 0 else (100 + level, 80, 80)
            pygame.draw.circle(a, wheel_col, (int(px), int(py)), 5)

        # Arena boundary
        pygame.draw.rect(a, WALL, (0, 0, ARENA_W, ARENA_H), 3)

    # -- HUD -----------------------------------------------------------------------------
    def draw_hud(self, sim, buttons, mouse_pos):
        s = self.screen
        pygame.draw.rect(s, (30, 33, 42), (0, 0, WIN_W, HUD_H))
        self.text(s, f"Layout {sim.layout_index + 1}/{len(LAYOUTS)}: {sim.name}",
                  (MARGIN, 10), self.font, GOAL_COL)

        if sim.finished:
            status, col = "GOAL REACHED", GOAL_COL
        elif sim.paused:
            status, col = "PAUSED", DIM
        elif sim.avoiding:
            status, col = "AVOIDING OBSTACLE", (255, 150, 90)
        else:
            status, col = "NAVIGATING", START_COL
        self.text(s, status, (MARGIN, 36), self.font, col)

        if sim.mode == "manual":
            mode = "MANUAL" + (" + avoidance assist" if sim.assist else " (assist off)")
        elif sim.mode == "auto":
            mode = "AUTONOMOUS (A* path planning)"
        else:
            mode = f"REACTIVE (sensors only): {sim.state_text}"
        self.text(s, f"Mode: {mode}", (MARGIN, 62), self.small, DIM)

        goal_dist = math.hypot(sim.robot.x - sim.goal[0], sim.robot.y - sim.goal[1])
        stats = [f"Time: {sim.elapsed:6.1f} s",
                 f"Collisions: {sim.collisions}",
                 f"Distance: {sim.distance / 10:6.1f} u",
                 f"To goal: {goal_dist / 10:5.1f} u",
                 f"Speed: x{sim.speed_scale:.2f}"]
        for i, line in enumerate(stats):
            self.text(s, line, (330 + (i % 3) * 150, 14 + (i // 3) * 28), self.font)
        noise = f"ON (+/-{Robot.NOISE_SIGMA:.0f}px, filtered)" if sim.noise else "OFF"
        self.text(s, f"Avoidance events: {sim.avoid_events}   Sensor noise: {noise}",
                  (330, 70), self.small, DIM)

        for b in buttons:
            hover = b.rect.collidepoint(mouse_pos)
            pygame.draw.rect(s, BUTTON_HOVER if hover else BUTTON, b.rect, border_radius=6)
            self.text(s, b.label(sim), b.rect.center, self.small, center=True)

        y0 = HUD_H + ARENA_H
        pygame.draw.rect(s, (30, 33, 42), (0, y0, WIN_W, FOOTER_H))
        self.text(s, "Arrows/WASD drive | M cycle mode | H assist | N sensor noise | "
                     "T path/goal line | V sensors", (MARGIN, y0 + 6), self.small, DIM)
        self.text(s, "+/- speed | 1-3 or L layout | R restart | Space pause | Esc quit",
                  (MARGIN, y0 + 26), self.small, DIM)

    def draw_summary(self, sim):
        """Overlay shown when the goal is reached (performance summary)."""
        panel = pygame.Surface((ARENA_W, ARENA_H), pygame.SRCALPHA)
        panel.fill((0, 0, 0, 170))
        cx, cy = ARENA_W // 2, ARENA_H // 2
        self.text(panel, "GOAL REACHED!", (cx, cy - 190), self.big, GOAL_COL, center=True)
        lines = [f"Time: {sim.elapsed:.1f} s    Collisions: {sim.collisions}    "
                 f"Distance: {sim.distance / 10:.1f} units",
                 f"Avoidance manoeuvres: {sim.avoid_events}    Mode: {MODE_NAMES[sim.mode]}"]
        for i, line in enumerate(lines):
            self.text(panel, line, (cx, cy - 130 + i * 26), self.font, center=True)

        # Comparison table of the best finished run per mode on this layout.
        top = cy - 55
        self.text(panel, f"Comparison - {sim.name}", (cx, top), self.font, GOAL_COL, center=True)
        cols = [cx - 270, cx - 40, cx + 60, cx + 170]
        for x, head in zip(cols, ("Mode", "Time (s)", "Collisions", "Distance (u)")):
            self.text(panel, head, (x, top + 28), self.small, DIM)
        for i, mode in enumerate(MODE_ORDER):
            rec = sim.records.get((sim.layout_index, mode))
            row = [MODE_NAMES[mode]] + (["-", "-", "-"] if rec is None else
                                        [f"{rec[0]:.1f}", str(rec[1]), f"{rec[2]:.1f}"])
            colr = TEXT if mode == sim.mode else DIM
            for x, cell in zip(cols, row):
                self.text(panel, cell, (x, top + 52 + i * 24), self.small, colr)

        self.text(panel, "R restart | M next mode | 1-3 change layout",
                  (cx, cy + 150), self.small, DIM, center=True)
        self.screen.blit(panel, ARENA_POS)

    def render(self, sim, buttons, mouse_pos, dt):
        self.tick += dt
        self.screen.fill(BG)
        self.draw_arena(sim)
        self.screen.blit(self.arena_surf, ARENA_POS)
        self.draw_hud(sim, buttons, mouse_pos)
        if sim.finished:
            self.draw_summary(sim)
        pygame.display.flip()


# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------
def keyboard_command(keys):
    """Translate pressed keys into (throttle, turn)."""
    throttle = turn = 0.0
    if keys[pygame.K_UP] or keys[pygame.K_w]:
        throttle += 1.0
    if keys[pygame.K_DOWN] or keys[pygame.K_s]:
        throttle -= 0.6
    if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
        turn += 1.0
    if keys[pygame.K_LEFT] or keys[pygame.K_a]:
        turn -= 1.0
    return throttle, turn


def main():
    pygame.init()
    pygame.display.set_caption("Robot Navigation & Obstacle Avoidance")
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    clock = pygame.time.Clock()

    sim = Simulation(0)
    renderer = Renderer(screen)

    bx = WIN_W - MARGIN - 170
    buttons = [
        Button((bx, 12, 170, 26), lambda: sim.reset(), lambda s: "Restart (R)"),
        Button((bx, 42, 170, 26), lambda: sim.cycle_mode(),
               lambda s: {"manual": "Mode: Manual (M)", "auto": "Mode: A* (M)",
                          "reactive": "Mode: Reactive (M)"}[s.mode]),
        Button((bx, 72, 170, 22), lambda: sim.load_layout(sim.layout_index + 1),
               lambda s: "Next layout (L)"),
    ]

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                k = event.key
                if k == pygame.K_ESCAPE:
                    running = False
                elif k == pygame.K_r:
                    sim.reset()
                elif k == pygame.K_m:
                    sim.cycle_mode()
                elif k == pygame.K_h:
                    sim.assist = not sim.assist
                elif k == pygame.K_n:
                    sim.noise = not sim.noise
                elif k == pygame.K_t:
                    sim.show_path = not sim.show_path
                elif k == pygame.K_v:
                    sim.show_sensors = not sim.show_sensors
                elif k == pygame.K_SPACE:
                    sim.paused = not sim.paused
                elif k == pygame.K_l:
                    sim.load_layout(sim.layout_index + 1)
                elif k in (pygame.K_1, pygame.K_2, pygame.K_3):
                    sim.load_layout(k - pygame.K_1)
                elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    sim.speed_scale = min(2.0, sim.speed_scale + 0.25)
                elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    sim.speed_scale = max(0.5, sim.speed_scale - 0.25)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                for b in buttons:
                    if b.rect.collidepoint(event.pos):
                        b.action()

        throttle, turn = keyboard_command(pygame.key.get_pressed())
        sim.update(dt, throttle, turn)
        renderer.render(sim, buttons, pygame.mouse.get_pos(), dt)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
