
import math
import random
import sys

import pygame
import os
import csv
# ============================================================
# CONFIG
# ============================================================

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 800

FPS = 60

BG_COLOR = (20, 20, 20)
ROAD_COLOR = (60, 60, 60)
LANE_LINE_COLOR = (120, 120, 120)
INTERSECTION_COLOR = (80, 80, 80)

HONEST_COLOR = (0, 150, 255)   # blue
LIAR_COLOR = (220, 60, 60)     # red
CRASH_COLOR = (255, 240, 0)    # yellow

TEXT_COLOR = (230, 230, 230)

CAR_SIZE = 20  # slightly larger to fit letters

CENTER_X = SCREEN_WIDTH // 2
CENTER_Y = SCREEN_HEIGHT // 2

# Intersection modeled roughly as a circle; inside this radius is "intersection"
INTERSECTION_RADIUS = 70
EXIT_MARGIN = 30

# How finely we sample paths (distance between consecutive points)
PATH_STEP = 6.0  # in pixels

# Car speed: pixels per second
CAR_SPEED_PPS = 120.0

# Spawn logic: ensure continuous flow from all directions
SPAWN_INTERVAL = 2.5          # seconds between spawns per direction
MAX_CARS_PER_DIR = 20         # cap to avoid infinite buildup
TOTAL_CARS = 100               # total cars per experiment run
LIAR_PROBABILITY = 0.5        # some cars lie about intent
USE_LIARS = True               # global switch: if False, no new liars are spawned

# Minimum gap to maintain between cars in the same lane (to avoid rear-end crashes)
FOLLOW_DISTANCE = CAR_SIZE * 1.5

# Minimum time headway between *successive* cars of the same movement
MIN_HEADWAY = 1.5  # seconds

# Allow multiple cars from the same movement to occupy the intersection
# at once (small platoons), instead of strictly one-at-a-time.
MAX_ACTIVE_PER_MOVEMENT = 3

# Intent-specific time headways so that we can let protected left-turn
# platoons move more tightly while keeping through traffic conservative.
HEADWAY_BY_INTENT = {
    "straight": MIN_HEADWAY,
    "right":    0.9,
    "left":     0.3,
}

# Directions and intents
DIRECTIONS = ["N", "E", "S", "W"]
INTENTS = ["straight", "right", "left"]

# Lane offset from road center (for right-hand traffic)
LANE_OFFSET = 35

# Traffic signal phases
PHASES = [
    # Phase 0: N-S straight + right
    {("N", "straight"), ("N", "right"),
     ("S", "straight"), ("S", "right")},

    # Phase 1: E-W straight + right
    {("E", "straight"), ("E", "right"),
     ("W", "straight"), ("W", "right")},

    # Phase 2: N protected left turn
    {("N", "left")},

    # Phase 3: S protected left turn
    {("S", "left")},

    # Phase 4: E protected left turn
    {("E", "left")},

    # Phase 5: W protected left turn
    {("W", "left")},
]

PHASE_NAMES = [
    "N-S straight+right",   # phase 0
    "E-W straight+right",   # phase 1
    "N left-turn",          # phase 2
    "S left-turn",          # phase 3
    "E left-turn",          # phase 4
    "W left-turn",          # phase 5
]

# PHASE_DURATION is no longer used for fixed phases but kept for
# compatibility if referenced elsewhere.
PHASE_DURATION = 6.0


# ============================================================
# PATH GENERATION (CENTERLINES)
# ============================================================


def distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def generate_routes():
    """
    Generate centerline routes and intersection entry indices
    for each direction + intent.

    IMPORTANT: We define geometry so that:
      - 'right' = driver's right turn (from the car's perspective)
      - 'left'  = driver's left turn

    That means:
      N right -> W,  N left -> E
      S right -> E,  S left -> W
      E right -> N,  E left -> S
      W right -> S,  W left -> N
    """
    routes = {d: {} for d in DIRECTIONS}

    SPAWN_OFFSET = 100
    ARC_STEPS = 20

    # STRAIGHT PATHS (centerlines)
    # N (top) -> S (bottom)
    points, entry_index = [], None
    x = CENTER_X
    y = -SPAWN_OFFSET
    while y <= SCREEN_HEIGHT + SPAWN_OFFSET:
        if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
            entry_index = len(points)
        points.append((x, y))
        y += PATH_STEP
    routes["N"]["straight"] = (points, entry_index)

    # S (bottom) -> N (top)
    points, entry_index = [], None
    x = CENTER_X
    y = SCREEN_HEIGHT + SPAWN_OFFSET
    while y >= -SPAWN_OFFSET:
        if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
            entry_index = len(points)
        points.append((x, y))
        y -= PATH_STEP
    routes["S"]["straight"] = (points, entry_index)

    # W (left) -> E (right)
    points, entry_index = [], None
    y = CENTER_Y
    x = -SPAWN_OFFSET
    while x <= SCREEN_WIDTH + SPAWN_OFFSET:
        if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
            entry_index = len(points)
        points.append((x, y))
        x += PATH_STEP
    routes["W"]["straight"] = (points, entry_index)

    # E (right) -> W (left)
    points, entry_index = [], None
    y = CENTER_Y
    x = SCREEN_WIDTH + SPAWN_OFFSET
    while x >= -SPAWN_OFFSET:
        if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
            entry_index = len(points)
        points.append((x, y))
        x -= PATH_STEP
    routes["E"]["straight"] = (points, entry_index)

    # Use conservative radii for turning arcs that keep the *actual car body*
    # (after lane shifts) safely inside the intersection so turning cars
    # never sweep into queued traffic behind the stop line.
    # We choose a smaller radius for left turns so they pass closer to the
    # center, and a slightly larger one for right turns which hug the corner.
    base_safe_R = max(
        INTERSECTION_RADIUS * 0.3,
        INTERSECTION_RADIUS - (LANE_OFFSET + CAR_SIZE * 1.2),
    )
    # Left turns: deepest into the intersection.
    TURN_R_LEFT = base_safe_R
    # Right turns: can be a bit wider, but still keep margin to queues.
    TURN_R_RIGHT = min(INTERSECTION_RADIUS - (LANE_OFFSET + CAR_SIZE * 0.5), INTERSECTION_RADIUS * 0.9)
    if TURN_R_RIGHT < TURN_R_LEFT:
        TURN_R_RIGHT = TURN_R_LEFT

    ARC_STEPS = 24

    # Helper to build a route: approach -> arc -> exit
    def build_turn_route(start, approach_dir, turn, end, intent):
        """Build a turning route centerline.
        start: (x, y) start outside
        approach_dir: 'N','S','E','W' indicating direction of travel toward center
        turn: 'cw' or 'ccw' in world coordinates
        end: destination axis 'N','S','E','W' where we exit along that axis
        intent: 'right' or 'left' from the DRIVER perspective
        """
        # Choose radius based on driver intent.
        if intent == "right":
            R = TURN_R_RIGHT
        else:
            R = TURN_R_LEFT

        points = []
        entry_index = None
        
        # IMPORTANT: We keep the turning centerlines conservative so that
        # after lane_shift and car width, bodies remain inside the
        # INTERSECTION_RADIUS envelope.

        # Approach straight until reaching circle of radius R
        x, y = start
        if approach_dir == "N":   # coming from top, moving down
            while y <= CENTER_Y - R:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                y += PATH_STEP
        elif approach_dir == "S":  # coming from bottom, moving up
            while y >= CENTER_Y + R:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                y -= PATH_STEP
        elif approach_dir == "W":  # coming from left, moving right
            while x <= CENTER_X - R:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                x += PATH_STEP
        elif approach_dir == "E":  # coming from right, moving left
            while x >= CENTER_X + R:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                x -= PATH_STEP

        # Arc around center
        if approach_dir == "N" and end == "W":  # driver's right from N
            theta_start, theta_end = -math.pi / 2 - (0 if turn == "ccw" else 0), -math.pi if turn == "ccw" else 0
        elif approach_dir == "N" and end == "E":  # driver's left from N
            theta_start, theta_end = -math.pi / 2, 0
        elif approach_dir == "S" and end == "E":  # driver's right from S
            theta_start, theta_end = math.pi / 2, 0
        elif approach_dir == "S" and end == "W":  # driver's left from S
            theta_start, theta_end = math.pi / 2, math.pi
        elif approach_dir == "E" and end == "N":  # driver's right from E
            theta_start, theta_end = 0, -math.pi / 2
        elif approach_dir == "E" and end == "S":  # driver's left from E
            theta_start, theta_end = 0, math.pi / 2
        elif approach_dir == "W" and end == "S":  # driver's right from W
            theta_start, theta_end = math.pi, math.pi / 2
        elif approach_dir == "W" and end == "N":  # driver's left from W
            theta_start, theta_end = math.pi, -math.pi / 2
        else:
            theta_start, theta_end = 0, 0  # shouldn't happen

        # Ensure arc direction consistent
        if theta_start < theta_end:
            step = (theta_end - theta_start) / ARC_STEPS
        else:
            step = (theta_end - theta_start) / ARC_STEPS

        for i in range(ARC_STEPS + 1):
            theta = theta_start + step * i
            ax = CENTER_X + R * math.cos(theta)
            ay = CENTER_Y + R * math.sin(theta)
            if entry_index is None and distance(ax, ay, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                entry_index = len(points)
            points.append((ax, ay))

        # Exit straight along destination axis
        if end == "E":
            x_start = CENTER_X + R
            y_const = CENTER_Y
            x = x_start
            while x <= SCREEN_WIDTH + SPAWN_OFFSET:
                points.append((x, y_const))
                x += PATH_STEP
        elif end == "W":
            x_start = CENTER_X - R
            y_const = CENTER_Y
            x = x_start
            while x >= -SPAWN_OFFSET:
                points.append((x, y_const))
                x -= PATH_STEP
        elif end == "S":
            x_const = CENTER_X
            y_start = CENTER_Y + R
            y = y_start
            while y <= SCREEN_HEIGHT + SPAWN_OFFSET:
                points.append((x_const, y))
                y += PATH_STEP
        elif end == "N":
            x_const = CENTER_X
            y_start = CENTER_Y - R
            y = y_start
            while y >= -SPAWN_OFFSET:
                points.append((x_const, y))
                y -= PATH_STEP

        return points, entry_index

    def build_left_L_route(start, approach_dir, end):
        """Simpler left-turn route: go straight toward the intersection
        center, then make a 90-degree turn and go straight out. This keeps
        the path well inside the intersection envelope and avoids sweeping
        into queued vehicles behind the stop line.
        """
        points = []
        entry_index = None

        # Pivot point roughly at the center of the intersection.
        pivot_x, pivot_y = CENTER_X, CENTER_Y

        x, y = start

        # 1) Approach: move along the incoming lane axis toward the pivot.
        if approach_dir == "N":  # coming from top, moving down
            while y < pivot_y:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                y += PATH_STEP
        elif approach_dir == "S":  # coming from bottom, moving up
            while y > pivot_y:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                y -= PATH_STEP
        elif approach_dir == "W":  # coming from left, moving right
            while x < pivot_x:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                x += PATH_STEP
        elif approach_dir == "E":  # coming from right, moving left
            while x > pivot_x:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                x -= PATH_STEP

        # 2) Turn and go straight out along the exit axis.
        # Snap exactly to the pivot first so the corner is clean.
        points.append((pivot_x, pivot_y))
        if entry_index is None and distance(pivot_x, pivot_y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
            entry_index = len(points) - 1

        if end == "E":
            y_const = pivot_y
            x = pivot_x
            while x <= SCREEN_WIDTH + SPAWN_OFFSET:
                points.append((x, y_const))
                x += PATH_STEP
        elif end == "W":
            y_const = pivot_y
            x = pivot_x
            while x >= -SPAWN_OFFSET:
                points.append((x, y_const))
                x -= PATH_STEP
        elif end == "N":
            x_const = pivot_x
            y = pivot_y
            while y >= -SPAWN_OFFSET:
                points.append((x_const, y))
                y -= PATH_STEP
        elif end == "S":
            x_const = pivot_x
            y = pivot_y
            while y <= SCREEN_HEIGHT + SPAWN_OFFSET:
                points.append((x_const, y))
                y += PATH_STEP

        return points, entry_index
    def build_right_L_route(start, approach_dir, end):
        """Simpler right-turn route: move a short distance into the
        intersection, pivot in the appropriate quadrant, then go straight
        out. Each (direction, end) pair uses a distinct pivot so that
        opposite right turns never overlap."""
        points = []
        entry_index = None

        # Choose a quadrant pivot that stays inside the intersection and
        # separates opposite right turns.
        corner_r = INTERSECTION_RADIUS * 0.6
        if approach_dir == "N" and end == "W":
            pivot_x, pivot_y = CENTER_X - corner_r, CENTER_Y - corner_r
        elif approach_dir == "S" and end == "E":
            pivot_x, pivot_y = CENTER_X + corner_r, CENTER_Y + corner_r
        elif approach_dir == "E" and end == "N":
            pivot_x, pivot_y = CENTER_X + corner_r, CENTER_Y - corner_r
        elif approach_dir == "W" and end == "S":
            pivot_x, pivot_y = CENTER_X - corner_r, CENTER_Y + corner_r
        else:
            pivot_x, pivot_y = CENTER_X, CENTER_Y  # fallback, should not happen

        x, y = start

        # 1) Approach toward the pivot along the incoming axis, but only
        # until we reach the pivot projection.
        if approach_dir == "N":  # from top, moving down
            while y < pivot_y:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                y += PATH_STEP
        elif approach_dir == "S":  # from bottom, moving up
            while y > pivot_y:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                y -= PATH_STEP
        elif approach_dir == "W":  # from left, moving right
            while x < pivot_x:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                x += PATH_STEP
        elif approach_dir == "E":  # from right, moving left
            while x > pivot_x:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                x -= PATH_STEP

        # 2) Add pivot and then go straight out along the exit axis.
        points.append((pivot_x, pivot_y))
        if entry_index is None and distance(pivot_x, pivot_y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
            entry_index = len(points) - 1

        if end == "E":
            y_const = pivot_y
            x = pivot_x
            while x <= SCREEN_WIDTH + SPAWN_OFFSET:
                points.append((x, y_const))
                x += PATH_STEP
        elif end == "W":
            y_const = pivot_y
            x = pivot_x
            while x >= -SPAWN_OFFSET:
                points.append((x, y_const))
                x -= PATH_STEP
        elif end == "N":
            x_const = pivot_x
            y = pivot_y
            while y >= -SPAWN_OFFSET:
                points.append((x_const, y))
                y -= PATH_STEP
        elif end == "S":
            x_const = pivot_x
            y = pivot_y
            while y <= SCREEN_HEIGHT + SPAWN_OFFSET:
                points.append((x_const, y))
                y += PATH_STEP

        return points, entry_index



    # Now define right/left based on DRIVER perspective
    # N: right->W, left->E
    routes["N"]["right"] = build_right_L_route((CENTER_X, -SPAWN_OFFSET), "N", "W")
    routes["N"]["left"] = build_left_L_route((CENTER_X, -SPAWN_OFFSET), "N", "E")

    # S: right->E, left->W
    routes["S"]["right"] = build_right_L_route((CENTER_X, SCREEN_HEIGHT + SPAWN_OFFSET), "S", "E")
    routes["S"]["left"] = build_left_L_route((CENTER_X, SCREEN_HEIGHT + SPAWN_OFFSET), "S", "W")

    # E: right->N, left->S
    routes["E"]["right"] = build_right_L_route((SCREEN_WIDTH + SPAWN_OFFSET, CENTER_Y), "E", "N")
    routes["E"]["left"] = build_left_L_route((SCREEN_WIDTH + SPAWN_OFFSET, CENTER_Y), "E", "S")

    # W: right->S, left->N
    routes["W"]["right"] = build_right_L_route((-SPAWN_OFFSET, CENTER_Y), "W", "S")
    routes["W"]["left"] = build_left_L_route((-SPAWN_OFFSET, CENTER_Y), "W", "N")

    return routes


ROUTES = generate_routes()

# ============================================================
# TRAFFIC MANAGER WITH SIGNALS
# ============================================================


class TrafficManager:
    """
    Manages signals assuming cars are truthful.
    Allows small platoons of cars from the same (direction, intent)
    movement to occupy the intersection at once, with intent-specific
    time headways between successive cars.
    """

    def __init__(self):
        self.queues = {(d, m): [] for d in DIRECTIONS for m in INTENTS}

        self.phase_index = 0
        self.phase_timer = 0.0
        self.current_phase = PHASES[0]

        self.active_counts = {(d, m): 0 for d in DIRECTIONS for m in INTENTS}
        self.time = 0.0
        self.last_release_time = {(d, m): -1e9 for d in DIRECTIONS for m in INTENTS}
        # Maximum concurrent cars per movement allowed inside the intersection.
        self.max_active_per_movement = MAX_ACTIVE_PER_MOVEMENT

    def register_waiting(self, car):
        key = (car.direction, car.reported_intent)
        q = self.queues[key]
        if car not in q:
            q.append(car)
        car.can_move = False
        car.tms_state = "waiting"

    def notify_exit(self, car):
        key = (car.direction, car.reported_intent)
        if self.active_counts[key] > 0:
            self.active_counts[key] -= 1
        car.in_intersection = False
        car.tms_state = "passed"

    def update(self, dt):
        self.time += dt

        # Phase timer and switch
        self.phase_timer += dt
        if self.phase_timer >= PHASE_DURATION:
            self._choose_new_phase()
            self.phase_timer = 0.0

        # For each movement in current phase, possibly let a PLATOON go
        for key in self.current_phase:
            d, intent = key

            # Allow up to N cars per movement instead of just one.
            if self.active_counts[key] >= self.max_active_per_movement:
                continue

            # Base headway depends on intent (tight for left-turn platoons).
            headway = HEADWAY_BY_INTENT.get(intent, MIN_HEADWAY)
            if self.time - self.last_release_time[key] < headway:
                continue

            q = self.queues[key]
            # Drop any finished/crashed cars that are still lingering in the queue.
            while q and (q[0].finished or q[0].crashed):
                q.pop(0)
            if not q:
                continue

            # If we have a big left-turn queue, relax headway a bit more so
            # 3–4 cars can clear in a single green.
            if intent == "left" and len(q) >= self.max_active_per_movement:
                headway = min(headway, 0.15)

            car = q.pop(0)
            if not car.can_move:
                car.can_move = True
                car.in_intersection = True
                car.tms_state = "in_intersection"
                self.active_counts[key] += 1
                self.last_release_time[key] = self.time

    def _choose_new_phase(self):
        # Choose the phase that serves the largest total queue, but
        # break ties in a round‑robin fashion starting from the next
        # phase after the current one. This avoids always favoring
        # N‑S when queues are equal.
        num_phases = len(PHASES)
        start = (self.phase_index + 1) % num_phases
        best_idx = self.phase_index
        best_score = -1

        for step in range(num_phases):
            idx = (start + step) % num_phases
            phase = PHASES[idx]
            score = 0
            for key in phase:
                q = self.queues[key]
                score += len([c for c in q if not c.finished and not c.crashed])
            if score > best_score:
                best_score = score
                best_idx = idx

        self.phase_index = best_idx
        self.current_phase = PHASES[self.phase_index]

    def get_signal_state(self, direction, intent):
        return "G" if (direction, intent) in self.current_phase else "R"


# ============================================================
# CAR
# ============================================================


class Car:
    _next_id = 0
    def __init__(self, direction, true_intent, reported_intent, liar=False):
        # assign a stable id for logging crashes
        self.id = Car._next_id
        Car._next_id += 1
        self.direction = direction
        self.true_intent = true_intent
        self.reported_intent = reported_intent
        self.liar = liar

        self.color = LIAR_COLOR if liar else HONEST_COLOR

        self.base_route_points, self.entry_index = ROUTES[direction][true_intent]
        self.route_len = len(self.base_route_points)

        if direction == "N":
            self.lane_shift = (-LANE_OFFSET, 0)
        elif direction == "S":
            self.lane_shift = (LANE_OFFSET, 0)
        elif direction == "W":
            self.lane_shift = (0, LANE_OFFSET)
        elif direction == "E":
            self.lane_shift = (0, -LANE_OFFSET)
        else:
            self.lane_shift = (0, 0)

        self.route_pos = 0.0
        self.speed_idx_per_sec = CAR_SPEED_PPS / PATH_STEP

        x0, y0 = self.base_route_points[0]
        self.pos = (x0 + self.lane_shift[0], y0 + self.lane_shift[1])

        self.can_move = True
        self.in_intersection = False
        self.has_queued = False
        self.tms_state = "approaching"

        self.finished = False
        self.crashed = False

    def _has_safe_gap(self, cand_x, cand_y, cars):
        for other in cars:
            if other is self or other.finished or other.crashed:
                continue
            if other.direction != self.direction:
                continue

            ox, oy = other.pos

            if self.direction == "N":
                if oy <= cand_y:
                    continue
            elif self.direction == "S":
                if oy >= cand_y:
                    continue
            elif self.direction == "W":
                if ox <= cand_x:
                    continue
            elif self.direction == "E":
                if ox >= cand_x:
                    continue

            if self.direction in ("N", "S"):
                if abs(ox - cand_x) > CAR_SIZE:
                    continue
            else:
                if abs(oy - cand_y) > CAR_SIZE:
                    continue

            if abs(ox - cand_x) < FOLLOW_DISTANCE and abs(oy - cand_y) < FOLLOW_DISTANCE:
                return False

        return True

    def update(self, dt, tm: TrafficManager, cars):
        if self.finished or self.crashed:
            return

        if self.in_intersection:
            cx, cy = self.pos
            d = distance(cx, cy, CENTER_X, CENTER_Y)
            if d > INTERSECTION_RADIUS + EXIT_MARGIN and self.route_pos > self.entry_index:
                tm.notify_exit(self)

        if not self.can_move:
            return

        raw_next = self.route_pos + self.speed_idx_per_sec * dt
        if raw_next > self.route_len - 1:
            raw_next = self.route_len - 1

        if (not self.has_queued and
                self.entry_index is not None and
                self.route_pos < self.entry_index <= raw_next):
            cand_route_pos = float(self.entry_index)
        else:
            cand_route_pos = raw_next

        i0 = int(cand_route_pos)
        i1 = min(i0 + 1, self.route_len - 1)
        alpha = cand_route_pos - i0
        x0, y0 = self.base_route_points[i0]
        x1, y1 = self.base_route_points[i1]
        cand_x = x0 * (1 - alpha) + x1 * alpha + self.lane_shift[0]
        cand_y = y0 * (1 - alpha) + y1 * alpha + self.lane_shift[1]

        if not self._has_safe_gap(cand_x, cand_y, cars):
            return

        self.route_pos = cand_route_pos
        self.pos = (cand_x, cand_y)

        if (not self.has_queued and
                self.entry_index is not None and
                abs(self.route_pos - self.entry_index) < 1e-3):
            self.has_queued = True
            tm.register_waiting(self)
            return

        if self.route_pos >= self.route_len - 1:
            self.finished = True
            if self.in_intersection:
                tm.notify_exit(self)
            return

    def draw(self, surface, font):
        x, y = self.pos
        rect = pygame.Rect(0, 0, CAR_SIZE, CAR_SIZE)
        rect.center = (int(x), int(y))

        # Border color encodes REPORTED intent
        if self.reported_intent == "straight":
            border_color = (255, 255, 255)
        elif self.reported_intent == "right":
            border_color = (0, 255, 0)
        else:
            border_color = (255, 165, 0)

        if self.crashed:
            fill_color = CRASH_COLOR
        else:
            fill_color = self.color

        pygame.draw.rect(surface, border_color, rect)
        inner = rect.inflate(-4, -4)
        pygame.draw.rect(surface, fill_color, inner)

        letter_map = {"straight": "S", "right": "R", "left": "L"}
        label = letter_map.get(self.true_intent, "?")
        text_surf = font.render(label, True, (0, 0, 0))
        text_rect = text_surf.get_rect(center=rect.center)
        surface.blit(text_surf, text_rect)


# ============================================================
# SPAWNING
# ============================================================


def spawn_car_for_direction(direction):
    true_intent = random.choice(INTENTS)

    # Decide whether this car will lie or not based on the global USE_LIARS flag.
    liar_prob = LIAR_PROBABILITY if USE_LIARS else 0.0
    liar = random.random() < liar_prob
    if liar:
        # Lying policy: usually claim a right turn (safer-looking) instead of
        # the true intent.
        if true_intent != "right":
            reported_intent = "right"
        else:
            reported_intent = "straight"
    else:
        reported_intent = true_intent

    return Car(direction, true_intent, reported_intent, liar=liar)



def spawn_four_right_turns_honest():
    # 4 honest cars, all truly turning right from their perspective
    return [Car(d, "right", "right", liar=False) for d in DIRECTIONS]


def spawn_four_with_liar():
    cars = []
    for d in DIRECTIONS:
        if d == "E":
            cars.append(Car(d, "straight", "right", liar=True))
        else:
            cars.append(Car(d, "right", "right", liar=False))
    return cars


# ============================================================
# COLLISION DETECTION
# ============================================================


def check_collisions(cars, tm: TrafficManager, crash_records):
    """Detect collisions, mark cars as crashed, and log crash context."""
    crashed = False
    for i in range(len(cars)):
        a = cars[i]
        if a.finished or a.crashed:
            continue
        ax, ay = a.pos
        for j in range(i + 1, len(cars)):
            b = cars[j]
            if b.finished or b.crashed:
                continue
            bx, by = b.pos
            if abs(ax - bx) < CAR_SIZE and abs(ay - by) < CAR_SIZE:
                a.crashed = True
                b.crashed = True
                crashed = True

                # determine which car (if any) is the liar
                if a.liar and not b.liar:
                    liar_role = "A"
                elif b.liar and not a.liar:
                    liar_role = "B"
                elif a.liar and b.liar:
                    liar_role = "both"
                else:
                    liar_role = "none"

                record = {
                    "sim_time": tm.time,
                    "phase_index": tm.phase_index,
                    "phase_name": PHASE_NAMES[tm.phase_index] if 0 <= tm.phase_index < len(PHASE_NAMES) else "unknown",
                    "allowed_movements": ";".join([f"{d}-{m}" for (d, m) in tm.current_phase]),
                    "liar_role": liar_role,

                    "carA_id": a.id,
                    "carA_direction": a.direction,
                    "carA_true_intent": a.true_intent,
                    "carA_reported_intent": a.reported_intent,
                    "carA_liar": a.liar,
                    "carA_tms_state": getattr(a, "tms_state", None),

                    "carB_id": b.id,
                    "carB_direction": b.direction,
                    "carB_true_intent": b.true_intent,
                    "carB_reported_intent": b.reported_intent,
                    "carB_liar": b.liar,
                    "carB_tms_state": getattr(b, "tms_state", None),
                }
                crash_records.append(record)
    return crashed


# ============================================================
# DRAWING
# ============================================================


def draw_roads(surface):
    road_width = 200
    h_rect = pygame.Rect(0, CENTER_Y - road_width // 2, SCREEN_WIDTH, road_width)
    pygame.draw.rect(surface, ROAD_COLOR, h_rect)
    v_rect = pygame.Rect(CENTER_X - road_width // 2, 0, road_width, SCREEN_HEIGHT)
    pygame.draw.rect(surface, ROAD_COLOR, v_rect)

    inter_rect = pygame.Rect(
        CENTER_X - INTERSECTION_RADIUS,
        CENTER_Y - INTERSECTION_RADIUS,
        INTERSECTION_RADIUS * 2,
        INTERSECTION_RADIUS * 2,
    )
    pygame.draw.rect(surface, INTERSECTION_COLOR, inter_rect)

    # --- removed the center crosshair lines ---
    # pygame.draw.line(surface, LANE_LINE_COLOR, (CENTER_X, 0), (CENTER_X, SCREEN_HEIGHT), 2)
    # pygame.draw.line(surface, LANE_LINE_COLOR, (0, CENTER_Y), (SCREEN_WIDTH, CENTER_Y), 2)

    # Keep only the two lane lines per direction (±LANE_OFFSET)
    pygame.draw.line(surface, LANE_LINE_COLOR, (CENTER_X - LANE_OFFSET, 0), (CENTER_X - LANE_OFFSET, SCREEN_HEIGHT), 1)
    pygame.draw.line(surface, LANE_LINE_COLOR, (CENTER_X + LANE_OFFSET, 0), (CENTER_X + LANE_OFFSET, SCREEN_HEIGHT), 1)
    pygame.draw.line(surface, LANE_LINE_COLOR, (0, CENTER_Y - LANE_OFFSET), (SCREEN_WIDTH, CENTER_Y - LANE_OFFSET), 1)
    pygame.draw.line(surface, LANE_LINE_COLOR, (0, CENTER_Y + LANE_OFFSET), (SCREEN_WIDTH, CENTER_Y + LANE_OFFSET), 1)

    # Stop lines
    zebra_color = (240, 240, 240)
    pygame.draw.line(
        surface,
        zebra_color,
        (CENTER_X - road_width // 2, CENTER_Y - INTERSECTION_RADIUS),
        (CENTER_X + road_width // 2, CENTER_Y - INTERSECTION_RADIUS),
        3,
    )
    pygame.draw.line(
        surface,
        zebra_color,
        (CENTER_X - road_width // 2, CENTER_Y + INTERSECTION_RADIUS),
        (CENTER_X + road_width // 2, CENTER_Y + INTERSECTION_RADIUS),
        3,
    )
    pygame.draw.line(
        surface,
        zebra_color,
        (CENTER_X - INTERSECTION_RADIUS, CENTER_Y - road_width // 2),
        (CENTER_X - INTERSECTION_RADIUS, CENTER_Y + road_width // 2),
        3,
    )
    pygame.draw.line(
        surface,
        zebra_color,
        (CENTER_X + INTERSECTION_RADIUS, CENTER_Y - road_width // 2),
        (CENTER_X + INTERSECTION_RADIUS, CENTER_Y + road_width // 2),
        3,
    )


def draw_signals(surface, tm: TrafficManager):
    size = 10
    margin = 4

    def color_for_state(state):
        return (0, 200, 0) if state == "G" else (200, 0, 0)

    # North
    x = CENTER_X - size - margin
    y = CENTER_Y - INTERSECTION_RADIUS - 25
    for intent in ["straight", "right", "left"]:
        state = tm.get_signal_state("N", intent)
        rect = pygame.Rect(x, y, size, size)
        pygame.draw.rect(surface, color_for_state(state), rect)
        y += size + 2

    # South
    x = CENTER_X + margin
    y = CENTER_Y + INTERSECTION_RADIUS + 5
    for intent in ["straight", "right", "left"]:
        state = tm.get_signal_state("S", intent)
        rect = pygame.Rect(x, y, size, size)
        pygame.draw.rect(surface, color_for_state(state), rect)
        y += size + 2

    # West
    x = CENTER_X - INTERSECTION_RADIUS - 25
    y = CENTER_Y + margin
    for intent in ["straight", "right", "left"]:
        state = tm.get_signal_state("W", intent)
        rect = pygame.Rect(x, y, size, size)
        pygame.draw.rect(surface, color_for_state(state), rect)
        x += size + 2

    # East
    x = CENTER_X + INTERSECTION_RADIUS + 5
    y = CENTER_Y - size - margin
    for intent in ["straight", "right", "left"]:
        state = tm.get_signal_state("E", intent)
        rect = pygame.Rect(x, y, size, size)
        pygame.draw.rect(surface, color_for_state(state), rect)
        x += size + 2


def draw_hud(surface, font, cars, tm: TrafficManager, crash_happened, total_spawned):
    honest = sum(1 for c in cars if not c.liar)
    liars = sum(1 for c in cars if c.liar)
    crashes = sum(1 for c in cars if c.crashed)

    finished_ok = sum(1 for c in cars if c.finished and not c.crashed)
    lines = [
        f"Honest cars (blue): {honest}",
        f"Lying cars (red): {liars}",
        f"Crashes: {crashes}",
        f"Finished (no crash): {finished_ok}",
        f"Total spawned this run: {total_spawned} / {TOTAL_CARS}",
        f"Phase: {PHASE_NAMES[tm.phase_index]}",
        f"Headway per movement: {MIN_HEADWAY:.1f}s",
        "Letter on car = TRUE intent (S/R/L).",
        "Border = reported intent: white=S, green=R, orange=L.",
        "Press '1': honest-only mode (no liars, 40 total cars).",
        "Press '2': liar mode (liars allowed, 40 total cars).",
    ]
    if crash_happened:
        lines.append("CRASH detected: simulation paused.")

    y = 10
    for line in lines:
        surf = font.render(line, True, TEXT_COLOR)
        surface.blit(surf, (10, y))
        y += surf.get_height() + 2


# ============================================================
# MAIN LOOP
# ============================================================


def main():
    global USE_LIARS
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Traffic Management System – correct right/left geometry & lying cars")
    clock = pygame.time.Clock()
    hud_font = pygame.font.SysFont("consolas", 14)
    car_font = pygame.font.SysFont("consolas", 12, bold=True)

    tm = TrafficManager()
    cars = []

    crash_happened = False
    crash_records = []  # list of crash context dicts
    spawn_timers = {d: 0.0 for d in DIRECTIONS}
    total_spawned = 0

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_1:
                    # Mode 1: all truthful cars, no new liars.
                    USE_LIARS = False
                    # Reset simulation state for a clean run.
                    cars = []
                    tm = TrafficManager()
                    crash_happened = False
                    spawn_timers = {d: 0.0 for d in DIRECTIONS}
                    total_spawned = 0
                elif event.key == pygame.K_2:
                    # Mode 2: allow lying cars according to LIAR_PROBABILITY.
                    USE_LIARS = True
                    cars = []
                    tm = TrafficManager()
                    crash_happened = False
                    spawn_timers = {d: 0.0 for d in DIRECTIONS}
                    total_spawned = 0
        if not crash_happened:
            # Spawn traffic up to TOTAL_CARS per run
            for d in DIRECTIONS:
                if total_spawned >= TOTAL_CARS:
                    break
                spawn_timers[d] += dt
                if spawn_timers[d] >= SPAWN_INTERVAL:
                    active_dir_cars = [c for c in cars
                                       if c.direction == d and not c.finished and not c.crashed]
                    if len(active_dir_cars) < MAX_CARS_PER_DIR and total_spawned < TOTAL_CARS:
                        cars.append(spawn_car_for_direction(d))
                        total_spawned += 1
                    spawn_timers[d] = 0.0

            tm.update(dt)

            for car in cars:
                car.update(dt, tm, cars)

            # if check_collisions(cars, tm, crash_records):
            #     crash_happened = True
            # Detect & log; DO NOT pause the simulation
            _ = check_collisions(cars, tm, crash_records)

            cars = [c for c in cars if not c.finished or c.crashed]

        screen.fill(BG_COLOR)
        draw_roads(screen)
        draw_signals(screen, tm)

        for car in cars:
            car.draw(screen, car_font)

        draw_hud(screen, hud_font, cars, tm, crash_happened, total_spawned)

        pygame.display.flip()

    # After window is closed, dump any crash records to CSV for offline analysis.
    if crash_records:
        base_dir = r"D:/UF_Documents/UF_Coursework/Fall_2025/EEL5632_Safety_and_Security_of_Vehicular_Electronic_Systems/Project_EEL5632_Fall2025/traffic_sim_Motor_Knights/trial3_tms_dir_vel"
        os.makedirs(base_dir, exist_ok=True)
        file_path = os.path.join(base_dir, "tms_dir_crash_log.csv")
    try:
        import pandas as pd
        df = pd.DataFrame(crash_records)
        df.to_csv(file_path, index=False)
        print(f"Saved {len(crash_records)} crash records to {file_path}")
    except Exception as e:
        # Fallback: write a very simple CSV if pandas is unavailable.
        keys = crash_records[0].keys()
        with open(file_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(crash_records)
        print(f"Saved {len(crash_records)} crash records to {file_path} (csv module fallback)")
        print('Pandas error was:', e)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
