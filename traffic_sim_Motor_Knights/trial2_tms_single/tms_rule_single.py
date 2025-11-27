# pygame-ce 2.5.6 (SDL 2.32.10, Python 3.12.7 (base))
import math
import random
import sys
import csv
import json
from pathlib import Path
import pygame

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
CAR_SIZE = 20
CENTER_X = SCREEN_WIDTH // 2
CENTER_Y = SCREEN_HEIGHT // 2

# Intersection modeled roughly as a circle
INTERSECTION_RADIUS = 70
EXIT_MARGIN = 30

# Sampling and speed
PATH_STEP = 6.0
CAR_SPEED_PPS = 120.0

# Continuous spawn
SPAWN_INTERVAL = 2.5
MAX_CARS_PER_DIR = 10
LIAR_PROBABILITY = 0.2

# Same-lane following safety
FOLLOW_DISTANCE = CAR_SIZE * 1.5
MIN_HEADWAY = 1.5

# Directions and intents
DIRECTIONS = ["N", "E", "S", "W"]
INTENTS = ["straight", "right", "left"]

# Lane offset from road center (right-hand traffic)
LANE_OFFSET = 35

# Signal phases
PHASES = [
    {("N", "straight"), ("N", "right"),
     ("S", "straight"), ("S", "right")},
    {("E", "straight"), ("E", "right"),
     ("W", "straight"), ("W", "right")},
    {("N", "left"), ("S", "left")},
    {("E", "left"), ("W", "left")},
]
PHASE_NAMES = [
    "N-S straight+right",
    "E-W straight+right",
    "N-S left turns",
    "E-W left turns",
]
PHASE_DURATION = 6.0

# Experiment controls and CSV target directory
EXPERIMENT_MODE = False
DISABLE_CONTINUOUS_SPAWN = False
CSV_FILENAME = r"D:\UF_Documents\UF_Coursework\Fall_2025\EEL5632_Safety_and_Security_of_Vehicular_Electronic_Systems\Project_EEL5632_Fall2025\traffic_sim_Motor_Knights\trial2_tms_single\experiment_results_tms_rule_single.csv"

# Lie types to iterate in experiments (true_intent, reported_intent)
LIE_TYPES = [
    ("straight", "right"),
    ("right", "straight"),
    ("left", "straight"),
    ("straight", "left"),
    ("left", "right"),
    ("right", "left"),
]

# ============================================================
# GEOMETRY UTILITIES
# ============================================================

def distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

def heading_for_approach(approach_dir):
    if approach_dir == "N": return -math.pi / 2  # down
    if approach_dir == "S": return math.pi / 2   # up
    if approach_dir == "E": return 0             # left
    if approach_dir == "W": return math.pi       # right
    return 0

def exit_axis_from_heading(theta):
    t = math.atan2(math.sin(theta), math.cos(theta))
    c, s = math.cos(t), math.sin(t)
    if abs(c) >= abs(s):  # horizontal
        return "horizontal_right" if c > 0 else "horizontal_left"
    else:  # vertical
        return "vertical_down" if s > 0 else "vertical_up"

# ============================================================
# PATH GENERATION (LANE CENTERLINES)
# ============================================================

def generate_routes():
    """
    Generate lane-centerline routes per (direction, intent).
    Straight routes use approach lane centers.
    Turn routes use an arc of radius R' tangent to approach lane and exit lane:
      - Right turns: inner lane (R' = INTERSECTION_RADIUS - LANE_OFFSET)
      - Left turns: outer lane (R' = INTERSECTION_RADIUS + LANE_OFFSET)
    """
    routes = {d: {} for d in DIRECTIONS}
    SPAWN_OFFSET = 100
    ARC_STEPS = 20

    # STRAIGHT PATHS (lane centers)
    # N (moving down)
    points, entry_index = [], None
    x = CENTER_X - LANE_OFFSET
    y = -SPAWN_OFFSET
    while y <= SCREEN_HEIGHT + SPAWN_OFFSET:
        if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
            entry_index = len(points)
        points.append((x, y))
        y += PATH_STEP
    routes["N"]["straight"] = (points, entry_index)

    # S (moving up)
    points, entry_index = [], None
    x = CENTER_X + LANE_OFFSET
    y = SCREEN_HEIGHT + SPAWN_OFFSET
    while y >= -SPAWN_OFFSET:
        if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
            entry_index = len(points)
        points.append((x, y))
        y -= PATH_STEP
    routes["S"]["straight"] = (points, entry_index)

    # W (moving right)
    points, entry_index = [], None
    y = CENTER_Y + LANE_OFFSET
    x = -SPAWN_OFFSET
    while x <= SCREEN_WIDTH + SPAWN_OFFSET:
        if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
            entry_index = len(points)
        points.append((x, y))
        x += PATH_STEP
    routes["W"]["straight"] = (points, entry_index)

    # E (moving left)
    points, entry_index = [], None
    y = CENTER_Y - LANE_OFFSET
    x = SCREEN_WIDTH + SPAWN_OFFSET
    while x >= -SPAWN_OFFSET:
        if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
            entry_index = len(points)
        points.append((x, y))
        x -= PATH_STEP
    routes["E"]["straight"] = (points, entry_index)

    # Helper to build a turn: approach (lane center) -> arc (R') -> exit (lane center)
    def build_turn_route(approach_dir, intent):
        points = []
        entry_index = None
        R_turn = INTERSECTION_RADIUS - LANE_OFFSET if intent == "right" else INTERSECTION_RADIUS + LANE_OFFSET

        if approach_dir == "N":
            x = CENTER_X - LANE_OFFSET
            y = -SPAWN_OFFSET
            while y <= CENTER_Y - R_turn:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                y += PATH_STEP
        elif approach_dir == "S":
            x = CENTER_X + LANE_OFFSET
            y = SCREEN_HEIGHT + SPAWN_OFFSET
            while y >= CENTER_Y + R_turn:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                y -= PATH_STEP
        elif approach_dir == "W":
            y = CENTER_Y + LANE_OFFSET
            x = -SPAWN_OFFSET
            while x <= CENTER_X - R_turn:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                x += PATH_STEP
        elif approach_dir == "E":
            y = CENTER_Y - LANE_OFFSET
            x = SCREEN_WIDTH + SPAWN_OFFSET
            while x >= CENTER_X + R_turn:
                if entry_index is None and distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                    entry_index = len(points)
                points.append((x, y))
                x -= PATH_STEP

        theta_start = heading_for_approach(approach_dir)
        theta_end = theta_start + (math.pi / 2 if intent == "left" else -math.pi / 2)
        step = (theta_end - theta_start) / ARC_STEPS
        for i in range(ARC_STEPS + 1):
            theta = theta_start + step * i
            ax = CENTER_X + R_turn * math.cos(theta)
            ay = CENTER_Y + R_turn * math.sin(theta)
            if entry_index is None and distance(ax, ay, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
                entry_index = len(points)
            points.append((ax, ay))

        axis = exit_axis_from_heading(theta_end)
        if axis == "horizontal_right":
            y_const = CENTER_Y + LANE_OFFSET
            x = points[-1][0]
            while x <= SCREEN_WIDTH + SPAWN_OFFSET:
                points.append((x, y_const))
                x += PATH_STEP
        elif axis == "horizontal_left":
            y_const = CENTER_Y - LANE_OFFSET
            x = points[-1][0]
            while x >= -SPAWN_OFFSET:
                points.append((x, y_const))
                x -= PATH_STEP
        elif axis == "vertical_down":
            x_const = CENTER_X - LANE_OFFSET
            y = points[-1][1]
            while y <= SCREEN_HEIGHT + SPAWN_OFFSET:
                points.append((x_const, y))
                y += PATH_STEP
        elif axis == "vertical_up":
            x_const = CENTER_X + LANE_OFFSET
            y = points[-1][1]
            while y >= -SPAWN_OFFSET:
                points.append((x_const, y))
                y -= PATH_STEP

        return points, entry_index

    # Turns (driver perspective)
    routes["N"]["right"] = build_turn_route("N", "right")
    routes["N"]["left"]  = build_turn_route("N", "left")
    routes["S"]["right"] = build_turn_route("S", "right")
    routes["S"]["left"]  = build_turn_route("S", "left")
    routes["E"]["right"] = build_turn_route("E", "right")
    routes["E"]["left"]  = build_turn_route("E", "left")
    routes["W"]["right"] = build_turn_route("W", "right")
    routes["W"]["left"]  = build_turn_route("W", "left")

    return routes

ROUTES = generate_routes()

# ============================================================
# TRAFFIC MANAGER WITH SIGNALS
# ============================================================

class TrafficManager:
    """
    Car-level controller:
      - One car per (direction,intent) in intersection at a time
      - MIN_HEADWAY between successive cars of same movement
    Supports fixed-phase mode for experiments.
    """
    def __init__(self):
        self.queues = {(d, m): [] for d in DIRECTIONS for m in INTENTS}
        self.phase_index = 0
        self.phase_timer = 0.0
        self.current_phase = PHASES[0]
        self.active_counts = {(d, m): 0 for d in DIRECTIONS for m in INTENTS}
        self.time = 0.0
        self.last_release_time = {(d, m): -1e9 for d in DIRECTIONS for m in INTENTS}
        self.fixed_phase = False

    def set_fixed_phase(self, idx):
        self.fixed_phase = True
        self.phase_index = idx % len(PHASES)
        self.current_phase = PHASES[self.phase_index]
        self.phase_timer = 0.0

    def clear_fixed_phase(self):
        self.fixed_phase = False
        self.phase_timer = 0.0

    def register_waiting(self, car):
        key = (car.direction, car.reported_intent)
        q = self.queues[key]
        if car not in q:
            q.append(car)
        car.can_move = False
        car.tms_state = "waiting"
        if getattr(car, "wait_start_time", None) is None:
            car.wait_start_time = self.time

    def notify_exit(self, car):
        key = (car.direction, car.reported_intent)
        if self.active_counts[key] > 0:
            self.active_counts[key] -= 1
        car.in_intersection = False
        car.tms_state = "passed"

    def update(self, dt):
        self.time += dt
        if not self.fixed_phase:
            self.phase_timer += dt
            if self.phase_timer >= PHASE_DURATION:
                self._choose_new_phase()
                self.phase_timer = 0.0
        else:
            self.current_phase = PHASES[self.phase_index]

        for key in self.current_phase:
            if self.active_counts[key] > 0:
                continue
            if self.time - self.last_release_time[key] < MIN_HEADWAY:
                continue
            q = self.queues[key]
            while q and (q[0].finished or q[0].crashed):
                q.pop(0)
            if q:
                car = q[0]
                if not car.can_move:
                    car.can_move = True
                    car.in_intersection = True
                    car.tms_state = "in_intersection"
                    car.release_time = self.time
                    self.active_counts[key] += 1
                    self.last_release_time[key] = self.time
                    q.pop(0)

    def _choose_new_phase(self):
        best_idx = self.phase_index
        best_score = -1
        for idx, phase in enumerate(PHASES):
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
    def __init__(self, direction, true_intent, reported_intent, liar=False):
        self.direction = direction
        self.true_intent = true_intent
        self.reported_intent = reported_intent
        self.liar = liar
        self.color = LIAR_COLOR if liar else HONEST_COLOR

        self.base_route_points, self.entry_index = ROUTES[direction][true_intent]
        self.route_len = len(self.base_route_points)

        self.route_pos = 0.0
        self.speed_idx_per_sec = CAR_SPEED_PPS / PATH_STEP

        x0, y0 = self.base_route_points[0]
        self.pos = (x0, y0)

        self.can_move = True
        self.in_intersection = False
        self.has_queued = False
        self.tms_state = "approaching"
        self.finished = False
        self.crashed = False

        # Experiment metrics
        self.wait_start_time = None
        self.release_time = None

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
        cand_x = x0 * (1 - alpha) + x1 * alpha
        cand_y = y0 * (1 - alpha) + y1 * alpha

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
        fill_color = CRASH_COLOR if self.crashed else self.color
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
    liar = random.random() < LIAR_PROBABILITY
    if liar:
        if true_intent != "right":
            reported_intent = "right"
        else:
            reported_intent = "straight"
    else:
        reported_intent = true_intent
    return Car(direction, true_intent, reported_intent, liar=liar)

def spawn_four_right_turns_honest():
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

def check_collisions(cars):
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
    return crashed

def crashed_pairs(cars):
    pairs = []
    used = set()
    for i in range(len(cars)):
        a = cars[i]
        if not a.crashed:
            continue
        for j in range(i + 1, len(cars)):
            b = cars[j]
            if not b.crashed:
                continue
            if abs(a.pos[0] - b.pos[0]) < CAR_SIZE and abs(a.pos[1] - b.pos[1]) < CAR_SIZE:
                key = (i, j)
                if key not in used:
                    pairs.append((a, b))
                    used.add(key)
    return pairs

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
    pygame.draw.line(surface, LANE_LINE_COLOR, (CENTER_X, 0), (CENTER_X, SCREEN_HEIGHT), 2)
    pygame.draw.line(surface, LANE_LINE_COLOR, (0, CENTER_Y), (SCREEN_WIDTH, CENTER_Y), 2)
    pygame.draw.line(surface, LANE_LINE_COLOR, (CENTER_X - LANE_OFFSET, 0), (CENTER_X - LANE_OFFSET, SCREEN_HEIGHT), 1)
    pygame.draw.line(surface, LANE_LINE_COLOR, (CENTER_X + LANE_OFFSET, 0), (CENTER_X + LANE_OFFSET, SCREEN_HEIGHT), 1)
    pygame.draw.line(surface, LANE_LINE_COLOR, (0, CENTER_Y - LANE_OFFSET), (SCREEN_WIDTH, CENTER_Y - LANE_OFFSET), 1)
    pygame.draw.line(surface, LANE_LINE_COLOR, (0, CENTER_Y + LANE_OFFSET), (SCREEN_WIDTH, CENTER_Y + LANE_OFFSET), 1)

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

def draw_hud(surface, font, cars, tm: TrafficManager, crash_happened, exp_status_text=""):
    honest = sum(1 for c in cars if not c.liar)
    liars = sum(1 for c in cars if c.liar)
    crashes = sum(1 for c in cars if c.crashed)
    lines = [
        f"Honest cars (blue): {honest}",
        f"Lying cars (red): {liars}",
        f"Crashes: {crashes}",
        f"Phase: {PHASE_NAMES[tm.phase_index]}{' [FIXED]' if tm.fixed_phase else ''}",
        f"Headway per movement: {MIN_HEADWAY:.1f}s",
        f"Continuous spawn: {'OFF' if DISABLE_CONTINUOUS_SPAWN else 'ON'}",
        "Letter on car = TRUE intent (S/R/L).",
        "Border = reported intent: white=S, green=R, orange=L.",
        "Press '1': 4 honest right-turn cars.",
        "Press '2': 4 right-reporting cars (E lies).",
        "Press '0-3': Fix phase to index 0..3; 'U' unfix.",
        "Press 'C': toggle continuous spawn, 'X': run experiments and save CSV.",
    ]
    if crash_happened:
        lines.append("CRASH detected: simulation paused.")
    if exp_status_text:
        lines.append(exp_status_text)

    y = 10
    for line in lines:
        surf = font.render(line, True, TEXT_COLOR)
        surface.blit(surf, (10, y))
        y += surf.get_height() + 2

# ============================================================
# EXPERIMENT HARNESS + CSV EXPORT
# ============================================================

def simulate_trial(phase_idx, liar_dir, liar_true, liar_reported, max_time=20.0, dt=1/120.0):
    """
    Run a fixed-phase trial:
      - Honest cars for allowed movements in phase
      - One test car (liar) vs baseline (honest-report)
    Returns detailed metrics including per-movement waits and both true/reported conflicts.
    """
    def run_case(liar_is_lie):
        tm = TrafficManager()
        tm.set_fixed_phase(phase_idx)
        cars = []
        # Honest cars for allowed movements
        for (d, m) in PHASES[phase_idx]:
            cars.append(Car(d, m, m, liar=False))
        # Test car
        if liar_is_lie:
            cars.append(Car(liar_dir, liar_true, liar_reported, liar=True))
        else:
            cars.append(Car(liar_dir, liar_true, liar_true, liar=False))

        crash = False
        t = 0.0
        while t < max_time and not crash:
            tm.update(dt)
            for car in cars:
                car.update(dt, tm, cars)
            crash = check_collisions(cars)
            if all(c.finished or c.crashed for c in cars):
                break
            t += dt

        # Per-movement wait times
        honest_waits = {}
        for c in cars:
            if not c.liar:
                key = (c.direction, c.true_intent)
                if c.wait_start_time is not None and c.release_time is not None:
                    honest_waits[key] = c.release_time - c.wait_start_time
                else:
                    honest_waits[key] = 0.0

        # Conflicts
        conflicts_true = []
        conflicts_reported = []
        if crash:
            for a, b in crashed_pairs(cars):
                conflicts_true.append(((a.direction, a.true_intent), (b.direction, b.true_intent)))
                conflicts_reported.append(((a.direction, a.reported_intent), (b.direction, b.reported_intent)))

        # Wasted capacity flag
        liar_obj = next((c for c in cars if c.liar), None)
        liar_wasted = False
        if liar_obj:
            true_key = (liar_obj.direction, liar_obj.true_intent)
            if true_key not in PHASES[phase_idx] and liar_obj.finished and not liar_obj.crashed:
                liar_wasted = True

        return {
            "crash": crash,
            "conflicts_true": conflicts_true,
            "conflicts_reported": conflicts_reported,
            "honest_waits": honest_waits,
            "liar_wasted": liar_wasted,
        }

    baseline = run_case(liar_is_lie=False)
    lie_run = run_case(liar_is_lie=True)

    # Per-movement extra wait
    extra_waits = {}
    for key, w in lie_run["honest_waits"].items():
        base_w = baseline["honest_waits"].get(key, 0.0)
        extra_waits[key] = max(0.0, w - base_w)
    extra_wait_total = sum(extra_waits.values())

    summary = {
        "phase_index": phase_idx,
        "phase": PHASE_NAMES[phase_idx],
        "liar_dir": liar_dir,
        "liar_true": liar_true,
        "liar_reported": liar_reported,
        "crash": lie_run["crash"],
        "conflicts_true": lie_run["conflicts_true"],
        "conflicts_reported": lie_run["conflicts_reported"],
        "liar_wasted": lie_run["liar_wasted"],
        "honest_waits_baseline": baseline["honest_waits"],
        "honest_waits_lie": lie_run["honest_waits"],
        "extra_waits": extra_waits,
        "extra_honest_wait": extra_wait_total,
    }
    return summary

def run_experiment_suite(save_csv=False, filename=CSV_FILENAME):
    results = []
    for p_idx in range(len(PHASES)):
        for d in DIRECTIONS:
            for (t_intent, r_intent) in LIE_TYPES:
                if t_intent == r_intent:
                    continue
                summary = simulate_trial(p_idx, d, t_intent, r_intent)
                results.append(summary)
                # Console log
                print(f"[{summary['phase']}] liar {d}: {t_intent}->{r_intent} | "
                      f"crash={summary['crash']} "
                      f"conflicts_true={summary['conflicts_true']} "
                      f"conflicts_reported={summary['conflicts_reported']} "
                      f"wasted={summary['liar_wasted']} "
                      f"extra_wait_total={summary['extra_honest_wait']:.2f}s "
                      f"extra_waits={summary['extra_waits']}")
    if save_csv and results:
        export_results_to_csv(results, filename)
    return results

def export_results_to_csv(results, filename):
    p = Path(filename)
    p.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "phase_index",
        "phase",
        "liar_dir",
        "liar_true",
        "liar_reported",
        "crash",
        "liar_wasted",
        "extra_honest_wait",
        "conflicts_true",
        "conflicts_reported",
        "honest_waits_baseline",
        "honest_waits_lie",
        "extra_waits",
    ]
    with p.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            row = {
                "phase_index": r["phase_index"],
                "phase": r["phase"],
                "liar_dir": r["liar_dir"],
                "liar_true": r["liar_true"],
                "liar_reported": r["liar_reported"],
                "crash": r["crash"],
                "liar_wasted": r["liar_wasted"],
                "extra_honest_wait": f"{r['extra_honest_wait']:.6f}",
                "conflicts_true": json.dumps(r["conflicts_true"]),
                "conflicts_reported": json.dumps(r["conflicts_reported"]),
                "honest_waits_baseline": json.dumps({f"{k[0]}-{k[1]}": v for k, v in r["honest_waits_baseline"].items()}),
                "honest_waits_lie": json.dumps({f"{k[0]}-{k[1]}": v for k, v in r["honest_waits_lie"].items()}),
                "extra_waits": json.dumps({f"{k[0]}-{k[1]}": v for k, v in r["extra_waits"].items()}),
            }
            writer.writerow(row)
    print(f"CSV saved to {p}")

# ============================================================
# MAIN LOOP
# ============================================================

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Traffic Management System – experiment mode + CSV export")
    clock = pygame.time.Clock()
    hud_font = pygame.font.SysFont("consolas", 14)
    car_font = pygame.font.SysFont("consolas", 12, bold=True)

    tm = TrafficManager()
    cars = []
    crash_happened = False
    spawn_timers = {d: 0.0 for d in DIRECTIONS}
    exp_status_text = ""

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
                    cars.extend(spawn_four_right_turns_honest())
                elif event.key == pygame.K_2:
                    cars.extend(spawn_four_with_liar())
                elif event.key in (pygame.K_0, pygame.K_1, pygame.K_2, pygame.K_3):
                    idx = event.key - pygame.K_0
                    tm.set_fixed_phase(idx)
                    exp_status_text = f"Fixed phase set to {PHASE_NAMES[idx]}"
                elif event.key == pygame.K_u:  # unfix phase
                    tm.clear_fixed_phase()
                    exp_status_text = "Phase unfixed (adaptive)"
                elif event.key == pygame.K_c:
                    global DISABLE_CONTINUOUS_SPAWN
                    DISABLE_CONTINUOUS_SPAWN = not DISABLE_CONTINUOUS_SPAWN
                    exp_status_text = f"Continuous spawn: {'OFF' if DISABLE_CONTINUOUS_SPAWN else 'ON'}"
                elif event.key == pygame.K_x:
                    exp_status_text = "Running experiments + CSV export... (see console)"
                    pygame.display.set_caption("Running experiment suite...")
                    results = run_experiment_suite(save_csv=True, filename=CSV_FILENAME)
                    total_crashes = sum(1 for r in results if r["crash"])
                    total_wasted = sum(1 for r in results if r["liar_wasted"])
                    avg_extra_wait = (sum(r["extra_honest_wait"] for r in results) / max(1, len(results)))
                    exp_status_text = f"Done: crashes={total_crashes}, wasted={total_wasted}, avg extra wait={avg_extra_wait:.2f}s, CSV saved to: {CSV_FILENAME}"
                    pygame.display.set_caption("Traffic Management System – experiment mode + CSV export")

        if not crash_happened:
            if not DISABLE_CONTINUOUS_SPAWN:
                for d in DIRECTIONS:
                    spawn_timers[d] += dt
                    if spawn_timers[d] >= SPAWN_INTERVAL:
                        active_dir_cars = [c for c in cars if c.direction == d and not c.finished and not c.crashed]
                        if len(active_dir_cars) < MAX_CARS_PER_DIR:
                            cars.append(spawn_car_for_direction(d))
                        spawn_timers[d] = 0.0

            tm.update(dt)
            for car in cars:
                car.update(dt, tm, cars)

            if check_collisions(cars):
                crash_happened = True

            cars = [c for c in cars if not c.finished or c.crashed]

        screen.fill(BG_COLOR)
        draw_roads(screen)
        draw_signals(screen, tm)
        for car in cars:
            car.draw(screen, car_font)
        draw_hud(screen, hud_font, cars, tm, crash_happened, exp_status_text)
        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()