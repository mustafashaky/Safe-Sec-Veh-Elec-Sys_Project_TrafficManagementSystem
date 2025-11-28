# pygame-ce 2.5.6 (SDL 2.32.10, Python 3.12.7 (base))
# tms_rule_platoons_signal_strict_no_overlap_hud_csv_liars_diverge_fix_entry_queue.py
import math
import random
import sys
import csv
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

CAR_COLOR = (0, 150, 255)      # blue for cars (HUD counts liars separately)
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
CAR_SPEED_PPS = 120.0  # pixels per second along path centerline

# Spawning
SPAWN_INTERVAL = 2.5
MAX_PLATOONS_PER_DIR = 5
DISABLE_CONTINUOUS_SPAWN = False

# Platoon parameters (treat each platoon as a unit)
PLATOON_MIN = 3             # cars per platoon (min)
PLATOON_MAX = 5             # cars per platoon (max)
INTRA_PLATOON_HEADWAY = 0.5 # seconds between cars of same platoon as they enter the box
INTER_PLATOON_HEADWAY = 1.5 # seconds between successive platoons from the same direction

# Alias for HUD compatibility with original wording
MIN_HEADWAY = INTER_PLATOON_HEADWAY

# Same-lane following safety (visualization spacing only)
FOLLOW_DISTANCE = CAR_SIZE * 1.5

# Queue staging gap (in path indices) between successive queued platoons at the stop line
QUEUE_PLATOON_GAP_IDX = int(PLATOON_MAX * ((CAR_SPEED_PPS * INTRA_PLATOON_HEADWAY) / PATH_STEP)) + 4

# Directions and intents
DIRECTIONS = ["N", "E", "S", "W"]
INTENTS = ["straight", "right", "left"]

# Lane offset from road center (right-hand traffic)
LANE_OFFSET = 35

# Signal phases (which movements are green together)
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

# CSV target directory (requested)
CSV_FILENAME = r"D:\UF_Documents\UF_Coursework\Fall_2025\EEL5632_Safety_and_Security_of_Vehicular_Electronic_Systems\Project_EEL5632_Fall2025\traffic_sim_Motor_Knights\trial2_tms_platoon\experiment_results_tms_rule_platoon.csv"

# ============================================================
# UTILS
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

def find_entry_index(points):
    # First point that lies inside the intersection
    for i, (x, y) in enumerate(points):
        if distance(x, y, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS:
            return i
    # Fallback: closest point to the center (robust if path just grazes)
    best_i, best_d = 0, float("inf")
    for i, (x, y) in enumerate(points):
        d = distance(x, y, CENTER_X, CENTER_Y)
        if d < best_d:
            best_d, best_i = d, i
    return best_i

# ============================================================
# ROUTES (LANE CENTERLINES)
# ============================================================
def generate_routes():
    routes = {d: {} for d in DIRECTIONS}
    SPAWN_OFFSET = 120
    ARC_STEPS = 20

    # STRAIGHT
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

    # TURNS
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

    for d in DIRECTIONS:
        routes[d]["right"] = build_turn_route(d, "right")
        routes[d]["left"]  = build_turn_route(d, "left")

    return routes

ROUTES = generate_routes()

# ============================================================
# CAR + PLATOON
# ============================================================
class Car:
    def __init__(self, direction, true_intent, reported_intent, liar=False):
        self.direction = direction
        self.true_intent = true_intent           # physical route
        self.reported_intent = reported_intent   # claimed movement
        self.liar = liar

        # Attach true route geometry for divergence after release
        self.true_points, self.true_entry_index = ROUTES[direction][true_intent]
        if self.true_entry_index is None:
            self.true_entry_index = find_entry_index(self.true_points)

        self.pos = (0.0, 0.0)
        self.in_intersection = False
        self.crashed = False
        self.finished = False

    def draw(self, surface, font):
        x, y = self.pos
        rect = pygame.Rect(0, 0, CAR_SIZE, CAR_SIZE)
        rect.center = (int(x), int(y))
        # Border color encodes REPORTED intent (white=S, green=R, orange=L)
        border_color = (255, 255, 255) if self.reported_intent == "straight" else ((0, 255, 0) if self.reported_intent == "right" else (255, 165, 0))
        fill_color = CRASH_COLOR if self.crashed else CAR_COLOR
        pygame.draw.rect(surface, border_color, rect)
        inner = rect.inflate(-4, -4)
        pygame.draw.rect(surface, fill_color, inner)
        # Letter shows TRUE intent (S/R/L)
        label = {"straight": "S", "right": "R", "left": "L"}[self.true_intent]
        surf = font.render(label, True, (0, 0, 0))
        surface.blit(surf, surf.get_rect(center=rect.center))

class Platoon:
    """
    3–5 cars as a unit:
      - Shared (direction, reported_intent)
      - Approach on reported movement lane; queue; cross when reported movement is green
      - After release, honest cars follow reported route; liars follow their true route through the box
      - Liars thus sometimes get away (no conflicting flow) and sometimes crash (conflict in box)
    """
    def __init__(self, direction, reported_intent, size=None, liar_ratio=0.0, liar_count=None):
        self.direction = direction
        self.reported_intent = reported_intent

        # Reported movement route for approach/waiting and for honest cars while moving
        self.rep_points, self.rep_entry_index = ROUTES[direction][reported_intent]
        if self.rep_entry_index is None:
            self.rep_entry_index = find_entry_index(self.rep_points)
        self.speed_idx_per_sec = CAR_SPEED_PPS / PATH_STEP

        self.size = size if size is not None else random.randint(PLATOON_MIN, PLATOON_MAX)
        self.gap_idx = max(1.0, (CAR_SPEED_PPS * INTRA_PLATOON_HEADWAY) / PATH_STEP)

        # Leader indices
        self.leader_idx = 0.0            # while approaching/waiting (path index along reported route)
        self.leader_progress = 0.0       # after release (distance along route from entry)

        self.state = "approaching"  # "approaching" -> "waiting" -> "moving" -> "passed"

        # Decide liar indexes deterministically if liar_count provided, else ratio
        liar_set = set()
        if isinstance(liar_count, int) and liar_count > 0:
            liar_count = min(liar_count, self.size)
            liar_indices = list(range(self.size))
            random.shuffle(liar_indices)
            liar_set = set(liar_indices[:liar_count])

        # Build cars
        self.cars = []
        for idx in range(self.size):
            liar = (idx in liar_set) if liar_set else (random.random() < liar_ratio)
            if liar:
                candidates = [i for i in INTENTS if i != reported_intent]
                true_intent = random.choice(candidates)
            else:
                true_intent = reported_intent
            self.cars.append(Car(direction, true_intent, reported_intent, liar=liar))

        self._place_cars_reported()  # place along reported route initially

    def _interp_on(self, points, idx):
        idx = max(0.0, min(idx, len(points) - 1))
        i0 = int(idx)
        i1 = min(i0 + 1, len(points) - 1)
        alpha = idx - i0
        x0, y0 = points[i0]
        x1, y1 = points[i1]
        return (x0 * (1 - alpha) + x1 * alpha, y0 * (1 - alpha) + y1 * alpha)

    def _place_cars_reported(self):
        # Position cars behind leader along REPORTED route (approach/waiting visuals)
        for i, c in enumerate(self.cars):
            idx = max(0.0, self.leader_idx - i * self.gap_idx)
            c.pos = self._interp_on(self.rep_points, idx)

    def _place_cars_moving(self):
        # After release, honest cars follow reported route; liars follow their true route
        for i, c in enumerate(self.cars):
            along = self.leader_progress - i * self.gap_idx
            if c.true_intent == self.reported_intent:
                idx = self.rep_entry_index + max(0.0, along)
                c.pos = self._interp_on(self.rep_points, idx)
            else:
                idx = c.true_entry_index + max(0.0, along)
                c.pos = self._interp_on(c.true_points, idx)
            cx, cy = c.pos
            c.in_intersection = (distance(cx, cy, CENTER_X, CENTER_Y) <= INTERSECTION_RADIUS + EXIT_MARGIN)

    def update(self, dt):
        if self.state == "passed":
            return

        if self.state == "approaching":
            self.leader_idx = min(len(self.rep_points) - 1, self.leader_idx + self.speed_idx_per_sec * dt)
            if self.rep_entry_index is not None and self.leader_idx >= self.rep_entry_index:
                # Freeze at the stop line; TrafficManager will stage this platoon behind others in the queue
                self.leader_idx = float(self.rep_entry_index)
                self.state = "waiting"
            self._place_cars_reported()
            return

        if self.state == "waiting":
            # Stay staged at whatever leader_idx TrafficManager assigned (stop line or behind)
            self._place_cars_reported()
            return

        if self.state == "moving":
            self.leader_progress += self.speed_idx_per_sec * dt
            self._place_cars_moving()

            # Leader clearance check using its active route
            leader_car = self.cars[0]
            if leader_car.true_intent == self.reported_intent:
                idx = self.rep_entry_index + self.leader_progress
                lx, ly = self._interp_on(self.rep_points, idx)
            else:
                idx = leader_car.true_entry_index + self.leader_progress
                lx, ly = self._interp_on(leader_car.true_points, idx)

            if (self.leader_progress > 0.0 and
                distance(lx, ly, CENTER_X, CENTER_Y) > INTERSECTION_RADIUS + EXIT_MARGIN):
                self.state = "passed"
            return

    def at_stop_line(self):
        return self.state == "waiting"

    def start_crossing(self):
        self.state = "moving"
        self.leader_progress = 0.0

# ============================================================
# TRAFFIC MANAGER (STRICT SIGNAL, NO OVERLAP, QUEUE STAGING)
# ============================================================
class TrafficManager:
    """
    Strict signal gating and direction-level no-overlap:
      - Queue platoons per movement (direction, reported_intent)
      - Only movements in current green phase may start crossing
      - At most one platoon per direction in the intersection at a time
      - Enforce INTER_PLATOON_HEADWAY between successive platoons from the same direction
      - Stage queued platoons behind the stop line to prevent overlap (visual queue)
    """
    def __init__(self):
        self.phase_index = 0
        self.phase_timer = 0.0
        self.current_phase = PHASES[self.phase_index]
        self.time = 0.0
        self.fixed_phase = False

        # Queues per movement
        self.queues = {(d, m): [] for d in DIRECTIONS for m in INTENTS}

        # Direction-level gating to avoid overlap within a direction
        self.dir_busy = {d: False for d in DIRECTIONS}
        self.dir_active_platoon = {d: None for d in DIRECTIONS}
        self.last_dir_release_time = {d: -1e9 for d in DIRECTIONS}

    def set_fixed_phase(self, idx):
        self.fixed_phase = True
        self.phase_index = idx % len(PHASES)
        self.current_phase = PHASES[self.phase_index]
        self.phase_timer = 0.0

    def clear_fixed_phase(self):
        self.fixed_phase = False
        self.phase_timer = 0.0

    def enqueue_platoon(self, platoon: Platoon):
        key = (platoon.direction, platoon.reported_intent)
        q = self.queues[key]
        if platoon not in q:
            q.append(platoon)
        # Stage this platoon behind the stop line based on its queue slot to avoid overlap
        slot = len(q) - 1  # 0 = at stop line, 1 = first behind, etc.
        staged_idx = max(0.0, platoon.rep_entry_index - slot * QUEUE_PLATOON_GAP_IDX)
        platoon.leader_idx = staged_idx
        platoon.state = "waiting"
        platoon._place_cars_reported()

    def update(self, dt):
        self.time += dt

        # Phase timing
        if not self.fixed_phase:
            self.phase_timer += dt
            if self.phase_timer >= PHASE_DURATION:
                self.phase_timer = 0.0
                self._choose_new_phase()

        # Strict green movements set (exact direction+intent pairs)
        green_movements = set(self.current_phase)

        # Free directions whose active platoon has passed
        for d in DIRECTIONS:
            ap = self.dir_active_platoon[d]
            if ap is not None and ap.state == "passed":
                self.dir_busy[d] = False
                self.dir_active_platoon[d] = None
                self.last_dir_release_time[d] = self.time

        # For each direction, consider only movements that are green now
        for d in DIRECTIONS:
            if self.dir_busy[d]:
                continue
            if self.time - self.last_dir_release_time[d] < INTER_PLATOON_HEADWAY:
                continue

            # Movements allowed for this direction in current green
            allowed_keys = [k for k in green_movements if k[0] == d]
            if not allowed_keys:
                continue  # nothing green for this direction; do not release

            # Find a queued head-at-stop-line platoon among allowed (green) movements
            candidate = None
            for k in allowed_keys:
                q = self.queues[k]
                while q and q[0].state == "passed":
                    q.pop(0)
                if q and q[0].at_stop_line():
                    candidate = (k, q[0])
                    break

            # Release one platoon for this direction, only if its movement is green
            if candidate is not None:
                k, platoon = candidate
                if (platoon.direction, platoon.reported_intent) not in green_movements:
                    continue
                platoon.start_crossing()
                self.dir_busy[d] = True
                self.dir_active_platoon[d] = platoon

        # Tidy queues: remove passed heads
        for k in list(self.queues.keys()):
            q = self.queues[k]
            while q and q[0].state == "passed":
                q.pop(0)

    def _choose_new_phase(self):
        # Choose phase with most waiting platoons
        best_idx, best_len = 0, -1
        for idx, phase in enumerate(PHASES):
            count = 0
            for key in phase:
                count += sum(1 for p in self.queues[key] if p.state == "waiting")
            if count > best_len:
                best_idx, best_len = idx, count
        self.phase_index = best_idx
        self.current_phase = PHASES[self.phase_index]

    def get_signal_state(self, direction, intent):
        return "G" if (direction, intent) in self.current_phase else "R"

# ============================================================
# COLLISION DETECTION (only inside intersection, and only if lying present)
# ============================================================
def check_collisions(platoons):
    """
    Register crashes only if at least one car in the colliding pair is lying
    (reported_intent != true_intent).
    """
    cars = [c for p in platoons for c in p.cars if c.in_intersection]
    crashed = False
    for i in range(len(cars)):
        a = cars[i]
        if a.crashed:
            continue
        ax, ay = a.pos
        for j in range(i + 1, len(cars)):
            b = cars[j]
            if b.crashed:
                continue
            bx, by = b.pos
            liar_involved = (a.reported_intent != a.true_intent) or (b.reported_intent != b.true_intent)
            if not liar_involved:
                continue
            if abs(ax - bx) < CAR_SIZE and abs(ay - by) < CAR_SIZE:
                a.crashed = b.crashed = True
                crashed = True
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
    pygame.draw.line(surface, LANE_LINE_COLOR, (CENTER_X, 0), (CENTER_X, SCREEN_HEIGHT), 2)
    pygame.draw.line(surface, LANE_LINE_COLOR, (0, CENTER_Y), (SCREEN_WIDTH, CENTER_Y), 2)
    pygame.draw.line(surface, LANE_LINE_COLOR, (CENTER_X - LANE_OFFSET, 0), (CENTER_X - LANE_OFFSET, SCREEN_HEIGHT), 1)
    pygame.draw.line(surface, LANE_LINE_COLOR, (CENTER_X + LANE_OFFSET, 0), (CENTER_X + LANE_OFFSET, SCREEN_HEIGHT), 1)
    pygame.draw.line(surface, LANE_LINE_COLOR, (0, CENTER_Y - LANE_OFFSET), (SCREEN_WIDTH, CENTER_Y - LANE_OFFSET), 1)
    pygame.draw.line(surface, LANE_LINE_COLOR, (0, CENTER_Y + LANE_OFFSET), (SCREEN_WIDTH, CENTER_Y + LANE_OFFSET), 1)

def draw_signals(surface, tm: TrafficManager):
    size = 10
    margin = 4
    def color_for_state(s): return (0, 200, 0) if s == "G" else (200, 0, 0)
    # North
    x = CENTER_X - size - margin
    y = CENTER_Y - INTERSECTION_RADIUS - 25
    for intent in ["straight", "right", "left"]:
        rect = pygame.Rect(x, y, size, size)
        pygame.draw.rect(surface, color_for_state(tm.get_signal_state("N", intent)), rect)
        y += size + 2
    # South
    x = CENTER_X + margin
    y = CENTER_Y + INTERSECTION_RADIUS + 5
    for intent in ["straight", "right", "left"]:
        rect = pygame.Rect(x, y, size, size)
        pygame.draw.rect(surface, color_for_state(tm.get_signal_state("S", intent)), rect)
        y += size + 2
    # West
    x = CENTER_X - INTERSECTION_RADIUS - 25
    y = CENTER_Y + margin
    for intent in ["straight", "right", "left"]:
        rect = pygame.Rect(x, y, size, size)
        pygame.draw.rect(surface, color_for_state(tm.get_signal_state("W", intent)), rect)
        x += size + 2
    # East
    x = CENTER_X + INTERSECTION_RADIUS + 5
    y = CENTER_Y - size - margin
    for intent in ["straight", "right", "left"]:
        rect = pygame.Rect(x, y, size, size)
        pygame.draw.rect(surface, color_for_state(tm.get_signal_state("E", intent)), rect)
        x += size + 2

def draw_hud(surface, font, platoons, tm: TrafficManager, crash_happened, exp_status_text=""):
    cars = [c for p in platoons for c in p.cars]
    honest = sum(1 for c in cars if c.reported_intent == c.true_intent)
    liars = sum(1 for c in cars if c.reported_intent != c.true_intent)
    crashes = sum(1 for c in cars if c.crashed)

    lines = [
        f"Honest cars (blue): {honest}",
        f"Lying cars (red): {liars}",
        f"Crashes: {crashes}",
        f"Phase: {PHASE_NAMES[tm.phase_index]}{' [FIXED]' if tm.fixed_phase else ''}",
        f"Headway per movement: {MIN_HEADWAY:.1f}s, platoon {PLATOON_MIN}-{PLATOON_MAX}",
        f"Continuous spawn: {'OFF' if DISABLE_CONTINUOUS_SPAWN else 'ON'}",
        "Letter on car = TRUE intent (S/R/L).",
        "Border = reported intent: white=S, green=R, orange=L.",
        "Press '1': 4 honest right-turn cars.",
        "Press '2': 4 right-reporting cars (E lies).",
        "Press '3': liar platoons (1–3 liars per platoon across all directions).",
        "Press '4': one liar platoon (random direction, 2 liars).",
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
# SPAWNING (PLATOONS)
# ============================================================
def spawn_platoon(direction, reported_intent=None, liar_ratio=0.0, liar_count=None):
    if reported_intent is None:
        reported_intent = random.choice(INTENTS)
    size = random.randint(PLATOON_MIN, PLATOON_MAX)
    return Platoon(direction, reported_intent, size=size, liar_ratio=liar_ratio, liar_count=liar_count)

def spawn_four_right_turn_platoons():
    return [spawn_platoon(d, reported_intent="right", liar_ratio=0.0) for d in DIRECTIONS]

def spawn_four_with_east_liar():
    ps = []
    for d in DIRECTIONS:
        if d == "E":
            ps.append(spawn_platoon(d, reported_intent="right", liar_count=2))
        else:
            ps.append(spawn_platoon(d, reported_intent="right", liar_ratio=0.0))
    return ps

def spawn_liar_platoons_all_dirs():
    ps = []
    for d in DIRECTIONS:
        liar_count = random.randint(1, min(3, PLATOON_MAX - 1))
        ps.append(spawn_platoon(d, reported_intent=random.choice(INTENTS), liar_count=liar_count))
    return ps

def spawn_one_liar_platoon():
    d = random.choice(DIRECTIONS)
    return [spawn_platoon(d, reported_intent=random.choice(INTENTS), liar_count=2)]

# ============================================================
# EXPERIMENTS + CSV EXPORT (PLATOON MODE)
# ============================================================
def simulate_trial(phase_idx, direction, reported_intent, size=None, liar_ratio=0.0, liar_count=None, max_time=20.0, dt=1/120.0):
    tm = TrafficManager()
    tm.set_fixed_phase(phase_idx)
    p = Platoon(direction, reported_intent, size=size, liar_ratio=liar_ratio, liar_count=liar_count)
    release_time = None
    clear_time = None
    crash = False
    t = 0.0
    while t < max_time and not crash and p.state != "passed":
        tm.update(dt)
        prev_state = p.state
        p.update(dt)
        if prev_state == "approaching" and p.state == "waiting":
            tm.enqueue_platoon(p)
        if prev_state == "waiting" and p.state == "moving" and release_time is None:
            release_time = t
        crash = check_collisions([p])
        t += dt
    if p.state == "passed":
        clear_time = t
    return {
        "phase_index": phase_idx,
        "phase": PHASE_NAMES[phase_idx],
        "direction": direction,
        "reported_intent": reported_intent,
        "platoon_size": p.size,
        "liar_ratio": liar_ratio,
        "liar_count": liar_count if liar_count is not None else 0,
        "released": release_time is not None,
        "cleared": clear_time is not None,
        "release_time": round(release_time, 4) if release_time is not None else None,
        "clear_time": round(clear_time, 4) if clear_time is not None else None,
        "crash": crash,
    }

def run_experiment_suite(save_csv=False, filename=CSV_FILENAME):
    results = []
    for p_idx, phase in enumerate(PHASES):
        for (d, m) in sorted(list(phase)):
            # honest, liar_ratio, and fixed liar_count variants
            results.append(simulate_trial(p_idx, d, m, size=PLATOON_MIN, liar_ratio=0.0))
            results.append(simulate_trial(p_idx, d, m, size=PLATOON_MIN, liar_ratio=0.5))
            results.append(simulate_trial(p_idx, d, m, size=PLATOON_MIN, liar_count=2))
            print(f"[{PHASE_NAMES[p_idx]}] {d}-{m} | trials added (honest, liar_ratio=0.5, liar_count=2)")
    if save_csv and results:
        export_results_to_csv(results, filename)
    return results

def export_results_to_csv(results, filename):
    p = Path(filename)
    p.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "phase_index", "phase", "direction", "reported_intent",
        "platoon_size", "liar_ratio", "liar_count", "released", "cleared", "release_time", "clear_time", "crash",
    ]
    with p.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            writer.writerow(r)
    print(f"CSV saved to {p}")

# ============================================================
# MAIN
# ============================================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Traffic Management System – Platoons per Signal (Liars Diverge, Strict Green, No Overlap, HUD+CSV)")
    clock = pygame.time.Clock()
    hud_font = pygame.font.SysFont("consolas", 14)
    car_font = pygame.font.SysFont("consolas", 12, bold=True)

    tm = TrafficManager()
    platoons = []
    crash_happened = False
    exp_status_text = ""
    spawn_timers = {d: 0.0 for d in DIRECTIONS}
    running = True

    # Seed one honest platoon per direction so traffic is visible immediately
    for d in DIRECTIONS:
        platoons.append(spawn_platoon(d))

    while running:
        dt = clock.tick(FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_c:
                    global DISABLE_CONTINUOUS_SPAWN
                    DISABLE_CONTINUOUS_SPAWN = not DISABLE_CONTINUOUS_SPAWN
                    exp_status_text = f"Continuous spawn: {'OFF' if DISABLE_CONTINUOUS_SPAWN else 'ON'}"
                elif event.key in (pygame.K_0, pygame.K_1, pygame.K_2, pygame.K_3):
                    idx = event.key - pygame.K_0
                    tm.set_fixed_phase(idx)
                    exp_status_text = f"Fixed phase set to {PHASE_NAMES[idx]}"
                elif event.key == pygame.K_u:
                    tm.clear_fixed_phase()
                    exp_status_text = "Phase unfixed (adaptive)"
                elif event.key == pygame.K_1:
                    platoons.extend(spawn_four_right_turn_platoons())
                    exp_status_text = "Spawned: 4 honest right-turn platoons (N/E/S/W)."
                elif event.key == pygame.K_2:
                    platoons.extend(spawn_four_with_east_liar())
                    exp_status_text = "Spawned: 3 honest + 1 (E) liar right-turn platoons."
                elif event.key == pygame.K_3:
                    platoons.extend(spawn_liar_platoons_all_dirs())
                    exp_status_text = "Spawned liar platoons across all directions (1–3 liars each)."
                elif event.key == pygame.K_4:
                    platoons.extend(spawn_one_liar_platoon())
                    exp_status_text = "Spawned one liar platoon (random direction, 2 liars)."
                elif event.key == pygame.K_x:
                    exp_status_text = "Running experiments + CSV export... (see console)"
                    pygame.display.set_caption("Running experiment suite...")
                    results = run_experiment_suite(save_csv=True, filename=CSV_FILENAME)
                    total = len(results)
                    rel = sum(1 for r in results if r["released"])
                    clr = sum(1 for r in results if r["cleared"])
                    cr = sum(1 for r in results if r["crash"])
                    exp_status_text = f"Done: trials={total}, released={rel}, cleared={clr}, crashes={cr}, CSV: {CSV_FILENAME}"
                    pygame.display.set_caption("Traffic Management System – Platoons per Signal (Liars Diverge, Strict Green, No Overlap, HUD+CSV)")

        if not crash_happened:
            # Spawn platoons per direction
            if not DISABLE_CONTINUOUS_SPAWN:
                for d in DIRECTIONS:
                    spawn_timers[d] += dt
                    if spawn_timers[d] >= SPAWN_INTERVAL:
                        active_dir_platoons = [p for p in platoons if p.direction == d and p.state in ("approaching", "waiting", "moving")]
                        if len(active_dir_platoons) < MAX_PLATOONS_PER_DIR:
                            # Occasionally spawn liar platoons in continuous mode
                            if random.random() < 0.4:
                                liar_count = random.randint(1, min(3, PLATOON_MAX - 1))
                                platoons.append(spawn_platoon(d, reported_intent=random.choice(INTENTS), liar_count=liar_count))
                            else:
                                platoons.append(spawn_platoon(d))
                        spawn_timers[d] = 0.0

            # Update TMS (phases + direction-gated releases, strict green)
            tm.update(dt)

            # Move platoons; enqueue when they reach or are staged at the stop line
            for p in platoons:
                prev_state = p.state
                p.update(dt)
                if prev_state == "approaching" and p.state == "waiting":
                    tm.enqueue_platoon(p)

            # Collision check (only inside intersection, and only if lying present)
            if check_collisions(platoons):
                crash_happened = True

            # Keep active platoons; remove passed ones
            platoons = [p for p in platoons if p.state != "passed"]

        # Draw
        screen.fill(BG_COLOR)
        draw_roads(screen)
        draw_signals(screen, tm)
        for p in platoons:
            for c in p.cars:
                c.draw(screen, car_font)
        draw_hud(screen, hud_font, platoons, tm, crash_happened, exp_status_text)
        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()