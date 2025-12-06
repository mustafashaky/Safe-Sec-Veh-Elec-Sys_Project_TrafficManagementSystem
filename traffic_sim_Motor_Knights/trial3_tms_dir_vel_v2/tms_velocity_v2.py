import math
import random
import sys
import csv

import pygame
import os
# ============================================================
# CONFIG
# ============================================================

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 800
FPS = 60

BG_COLOR = (30, 30, 30)
ROAD_COLOR = (60, 60, 60)
INTERSECTION_COLOR = (80, 80, 80)
PATH_COLOR = (120, 160, 220)       # lines for routes in center

TRUE_COLOR = (0, 170, 255)         # blue = truthful
LIAR_COLOR = (230, 60, 60)         # red = liar
CRASH_COLOR = (255, 255, 0)        # yellow on crash

CAR_SIZE = 16

CENTER_X = SCREEN_WIDTH // 2
CENTER_Y = SCREEN_HEIGHT // 2
INTERSECTION_RADIUS = 70

DIRECTIONS = ["N", "E", "S", "W"]
INTENTS = ["straight", "right", "left"]

LANE_OFFSET = 35      # distance from road center to lane center
PATH_STEP = 5.0

MIN_PREF_SPEED = 80.0
MAX_PREF_SPEED = 160.0

LIAR_FRACTION = 0.35

# Time staggering toward center (baseline)
CENTER_TIME_GAP = 0.7

# For dynamic prediction
PREDICT_STEP = 0.1       # seconds per prediction step
PREDICT_HORIZON = 0.8    # seconds ahead
MIN_SAFE_DIST = CAR_SIZE * 2.0

# Spawn spacing per direction
MIN_DIRECTION_SPAWN_GAP = 0.5   # minimum time between spawns from same direction

RANDOM_BURST_CAR_COUNT = 10
RANDOM_BURST_WINDOW = 4.0

# ============================================================
# GEOMETRY / ROUTES
# ============================================================

def distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def build_polyline(points):
    """
    Turn control points into sampled polyline with PATH_STEP spacing.
    Return:
        sampled_points, entry_index
    where entry_index is first index inside the intersection radius.
    """
    sampled = []
    entry_index = None

    for seg_idx in range(len(points) - 1):
        x0, y0 = points[seg_idx]
        x1, y1 = points[seg_idx + 1]
        dx = x1 - x0
        dy = y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len == 0:
            continue

        steps = max(1, int(seg_len / PATH_STEP))
        for i in range(steps + 1):
            if seg_idx > 0 and i == 0:
                continue
            t = i / steps
            x = x0 + dx * t
            y = y0 + dy * t
            sampled.append((x, y))
            if entry_index is None and abs(x - CENTER_X) <= INTERSECTION_RADIUS and abs(y - CENTER_Y) <= INTERSECTION_RADIUS:
                entry_index = len(sampled) - 1

    return sampled, entry_index


def generate_routes():
    """
    Build routes using four lane centerlines:
      - vertical: N->S (x = CX - L), S->N (x = CX + L)
      - horizontal: W->E (y = CY + L), E->W (y = CY - L)
    """
    routes = {d: {} for d in DIRECTIONS}

    SPAWN_OFFSET = 200
    X_LEFT_OUT = -SPAWN_OFFSET
    X_RIGHT_OUT = SCREEN_WIDTH + SPAWN_OFFSET
    Y_TOP_OUT = -SPAWN_OFFSET
    Y_BOTTOM_OUT = SCREEN_HEIGHT + SPAWN_OFFSET

    cx, cy = CENTER_X, CENTER_Y

    # Lane center positions
    x_NS_down  = cx - LANE_OFFSET      # N->S lane
    x_SN_up    = cx + LANE_OFFSET      # S->N lane
    y_WE_right = cy + LANE_OFFSET      # W->E lane
    y_EW_left  = cy - LANE_OFFSET      # E->W lane

    # ----- STRAIGHT MOVEMENTS -----
    routes["N"]["straight"] = build_polyline([
        (x_NS_down, Y_TOP_OUT),
        (x_NS_down, Y_BOTTOM_OUT),
    ])

    routes["S"]["straight"] = build_polyline([
        (x_SN_up, Y_BOTTOM_OUT),
        (x_SN_up, Y_TOP_OUT),
    ])

    routes["W"]["straight"] = build_polyline([
        (X_LEFT_OUT, y_WE_right),
        (X_RIGHT_OUT, y_WE_right),
    ])

    routes["E"]["straight"] = build_polyline([
        (X_RIGHT_OUT, y_EW_left),
        (X_LEFT_OUT, y_EW_left),
    ])

    # ----- TURNS -----
    # From N (coming down on x_NS_down)
    routes["N"]["right"] = build_polyline([
        (x_NS_down, Y_TOP_OUT),
        (x_NS_down, y_EW_left),
        (X_LEFT_OUT, y_EW_left),
    ])

    routes["N"]["left"] = build_polyline([
        (x_NS_down, Y_TOP_OUT),
        (x_NS_down, y_WE_right),
        (X_RIGHT_OUT, y_WE_right),
    ])

    # From S (coming up on x_SN_up)
    routes["S"]["right"] = build_polyline([
        (x_SN_up, Y_BOTTOM_OUT),
        (x_SN_up, y_WE_right),
        (X_RIGHT_OUT, y_WE_right),
    ])

    routes["S"]["left"] = build_polyline([
        (x_SN_up, Y_BOTTOM_OUT),
        (x_SN_up, y_EW_left),
        (X_LEFT_OUT, y_EW_left),
    ])

    # From W (coming right on y_WE_right)
    routes["W"]["right"] = build_polyline([
        (X_LEFT_OUT, y_WE_right),
        (x_NS_down, y_WE_right),
        (x_NS_down, Y_BOTTOM_OUT),
    ])

    routes["W"]["left"] = build_polyline([
        (X_LEFT_OUT, y_WE_right),
        (x_SN_up, y_WE_right),
        (x_SN_up, Y_TOP_OUT),
    ])

    # From E (coming left on y_EW_left)
    routes["E"]["right"] = build_polyline([
        (X_RIGHT_OUT, y_EW_left),
        (x_SN_up, y_EW_left),
        (x_SN_up, Y_TOP_OUT),
    ])

    routes["E"]["left"] = build_polyline([
        (X_RIGHT_OUT, y_EW_left),
        (x_NS_down, y_EW_left),
        (x_NS_down, Y_BOTTOM_OUT),
    ])

    return routes


ROUTES = generate_routes()

# For drawing visible paths in center (segments within intersection)
CENTER_PATHS = []
for d in DIRECTIONS:
    for intent in INTENTS:
        pts, _ = ROUTES[d][intent]
        filtered = [
            (x, y)
            for (x, y) in pts
            if abs(x - CENTER_X) <= INTERSECTION_RADIUS
            and abs(y - CENTER_Y) <= INTERSECTION_RADIUS
        ]
        if len(filtered) >= 2:
            CENTER_PATHS.append([(int(x), int(y)) for (x, y) in filtered])

# ============================================================
# CAR
# ============================================================

class Car:
    _next_id = 0

    def __init__(
        self,
        direction,
        true_intent,
        true_pref_speed,
        is_truthful,
        reported_intent,
        reported_speed,
        spawn_time,
        scenario_index,
        scenario_mode,
        burst_index,
    ):
        self.id = Car._next_id
        Car._next_id += 1

        self.direction = direction
        self.true_intent = true_intent
        self.true_pref_speed = true_pref_speed
        self.is_truthful = is_truthful
        self.reported_intent = reported_intent
        self.reported_speed = reported_speed
        self.spawn_time = spawn_time

        self.scenario_index = scenario_index
        self.scenario_mode = scenario_mode
        self.burst_index = burst_index

        # Actual route is based on TRUE intent (physical motion)
        base_route_points, entry_idx_true = ROUTES[direction][true_intent]
        self.base_route_points = base_route_points
        self.entry_idx_true = entry_idx_true
        self.route_len = len(self.base_route_points)

        self.route_pos = 0.0
        self.assigned_speed = 0.0   # baseline speed from TMS schedule
        self.current_speed = 0.0    # speed actually used this frame
        self.speed_idx_per_sec = 0.0

        self.finished = False
        self.crashed = False

        self.pos = self._sample_position(self.route_pos)

        # --- instrumentation: per-car metrics ---
        self.target_entry_time = None
        self.entry_time = None
        self.exit_time = None
        self.shield_stops = 0
        self.shield_stop_time = 0.0
        self._shield_stopped_this_frame = False
        # --- end instrumentation ---

    def _sample_position(self, route_pos):
        if self.route_len < 2:
            x, y = self.base_route_points[0]
            return x, y

        i0 = int(route_pos)
        if i0 >= self.route_len - 1:
            i0 = self.route_len - 2
        i1 = i0 + 1
        alpha = route_pos - i0
        x0, y0 = self.base_route_points[i0]
        x1, y1 = self.base_route_points[i1]
        x = x0 * (1 - alpha) + x1 * alpha
        y = y0 * (1 - alpha) + y1 * alpha
        return x, y

    def set_baseline_speed(self, speed):
        self.assigned_speed = speed
        self.current_speed = speed
        self.speed_idx_per_sec = self.current_speed / PATH_STEP

    def update_with_current_speed(self, dt):
        if self.finished or self.current_speed <= 0.0:
            return
        self.speed_idx_per_sec = self.current_speed / PATH_STEP
        self.route_pos += self.speed_idx_per_sec * dt
        if self.route_pos >= self.route_len - 1:
            self.finished = True
            return
        self.pos = self._sample_position(self.route_pos)

    def rect_at(self, future_dt):
        """
        Rectangle at time t + future_dt assuming current_speed constant.
        """
        if self.finished:
            x, y = self.pos
        else:
            delta = (self.current_speed / PATH_STEP) * future_dt
            future_pos = self.route_pos + delta
            if future_pos >= self.route_len - 1:
                x, y = self._sample_position(self.route_len - 1)
            else:
                x, y = self._sample_position(future_pos)
        r = pygame.Rect(0, 0, CAR_SIZE, CAR_SIZE)
        r.center = (int(x), int(y))
        return r

    def current_rect(self):
        r = pygame.Rect(0, 0, CAR_SIZE, CAR_SIZE)
        r.center = (int(self.pos[0]), int(self.pos[1]))
        return r

    def draw(self, surface):
        if self.finished:
            return
        if self.crashed:
            color = CRASH_COLOR
        else:
            color = TRUE_COLOR if self.is_truthful else LIAR_COLOR
        pygame.draw.rect(surface, color, self.current_rect())


# ============================================================
# TMS (single logic, with truthful safety shield)
# ============================================================

class DynamicTMS:
    def __init__(self):
        self.center_schedule = []   # not strictly needed now, but kept for possible use

    def assign_baseline_speeds_for_burst(self, cars):
        """
        TMS uses ONLY reported intent + reported speed to set a baseline speed
        that gives some spacing toward the intersection. Same for all scenarios.
        """
        info = []
        for car in cars:
            rep_points, entry_rep = ROUTES[car.direction][car.reported_intent]

            if entry_rep is None:
                dist_to_center = len(rep_points) * PATH_STEP
            else:
                dist_to_center = entry_rep * PATH_STEP

            base_speed = max(car.reported_speed, MIN_PREF_SPEED * 0.5)
            spawn_time = car.spawn_time
            preferred_time = spawn_time + dist_to_center / base_speed if base_speed > 0 else spawn_time + 9999.0

            info.append((car, dist_to_center, preferred_time))

        info.sort(key=lambda x: x[2])

        last_entry_time = 0.0
        for car, dist_to_center, preferred_time in info:
            # simple spacing in "preferred arrival at center"
            target_entry_time = max(preferred_time, last_entry_time + CENTER_TIME_GAP)
            travel_time = max(target_entry_time - car.spawn_time, 0.1)

            speed = dist_to_center / travel_time if travel_time > 0 else MIN_PREF_SPEED
            speed = max(MIN_PREF_SPEED * 0.5, min(speed, MAX_PREF_SPEED * 1.5))

            car.set_baseline_speed(speed)
            car.target_entry_time = target_entry_time  # instrumentation
            last_entry_time = car.spawn_time + dist_to_center / speed if speed > 0 else last_entry_time

    def apply_truthful_safety(self, cars):
        """
        Strong safety shield for truthful cars:
        - Predict future positions up to PREDICT_HORIZON.
        - If two truthful cars would overlap, force the "follower" to stop.
        - For pairs involving a liar, we do nothing (let them potentially crash).
        """
        active = [c for c in cars if not c.finished and not c.crashed]
        if not active:
            return

        # Start from baseline each frame
        for c in active:
            c.current_speed = c.assigned_speed
            c.speed_idx_per_sec = c.current_speed / PATH_STEP if c.current_speed > 0 else 0.0

        steps = int(PREDICT_HORIZON / PREDICT_STEP)
        if steps < 1:
            steps = 1

        MAX_ITERS = 10
        for _ in range(MAX_ITERS):
            changed = False

            for i in range(len(active)):
                for j in range(i + 1, len(active)):
                    ci = active[i]
                    cj = active[j]

                    # Only apply shield when both are truthful
                    if not (ci.is_truthful and cj.is_truthful):
                        continue

                    # Predict future overlap
                    will_overlap = False
                    for k in range(1, steps + 1):
                        dt = k * PREDICT_STEP
                        ri = ci.rect_at(dt)
                        rj = cj.rect_at(dt)
                        if ri.colliderect(rj):
                            will_overlap = True
                            break

                    if will_overlap:
                        # Choose follower: later spawn_time, tie-break on higher ID
                        if ci.spawn_time > cj.spawn_time:
                            follower = ci
                        elif cj.spawn_time > ci.spawn_time:
                            follower = cj
                        else:
                            follower = ci if ci.id > cj.id else cj

                        if follower.current_speed > 0:
                            follower.current_speed = 0.0
                            follower.speed_idx_per_sec = 0.0
                            # instrumentation:
                            follower._shield_stopped_this_frame = True
                            follower.shield_stops += 1
                            changed = True

            if not changed:
                break


# ============================================================
# SPAWN / EVENTS
# ============================================================

def make_events_timeline():
    events = []
    t = 0.5
    modes = ["all_true", "mixed"]

    for scenario_index, mode in enumerate(modes):
        # bursts 1–3: structured 4-car bursts
        for burst_index in range(1, 4):
            events.append({
                "time": t,
                "mode": mode,
                "kind": "burst",
                "scenario_index": scenario_index,
                "burst_index": burst_index,
            })
            t += 3.0

        # burst 4: random single-car spawns
        base = t
        for _ in range(RANDOM_BURST_CAR_COUNT):
            spawn_time = base + random.uniform(0.0, RANDOM_BURST_WINDOW)
            events.append({
                "time": spawn_time,
                "mode": mode,
                "kind": "random",
                "scenario_index": scenario_index,
                "burst_index": 4,
            })
        t = base + RANDOM_BURST_WINDOW + 4.0

    events.sort(key=lambda e: e["time"])
    return events


def spawn_car(
    mode,
    direction,
    current_time,
    scenario_index,
    burst_index,
    min_spawn_time_for_dir,
):
    """
    Create a car, but enforce per-direction spawn time gap so they
    don't appear on top of each other.
    """
    # enforce direction-based spawn gap
    effective_spawn_time = max(current_time, min_spawn_time_for_dir[direction])
    min_spawn_time_for_dir[direction] = effective_spawn_time + MIN_DIRECTION_SPAWN_GAP

    true_intent = random.choice(INTENTS)
    true_pref_speed = random.uniform(MIN_PREF_SPEED, MAX_PREF_SPEED)

    if mode == "all_true":
        is_truthful = True
        rep_intent = true_intent
        rep_speed = true_pref_speed
    else:
        if random.random() < LIAR_FRACTION:
            is_truthful = False
            other_intents = [i for i in INTENTS if i != true_intent]
            # rep_intent = random.choice(other_intents)
            rep_intent = true_intent
            rep_speed = true_pref_speed * random.uniform(0.5, 1.5)
        else:
            is_truthful = True
            rep_intent = true_intent
            rep_speed = true_pref_speed

    car = Car(
        direction=direction,
        true_intent=true_intent,
        true_pref_speed=true_pref_speed,
        is_truthful=is_truthful,
        reported_intent=rep_intent,
        reported_speed=rep_speed,
        spawn_time=effective_spawn_time,
        scenario_index=scenario_index,
        scenario_mode=mode,
        burst_index=burst_index,
    )
    return car


def spawn_standard_burst(mode, current_time, scenario_index, burst_index, min_spawn_time_for_dir):
    cars = []
    for direction in DIRECTIONS:
        car = spawn_car(
            mode,
            direction,
            current_time,
            scenario_index,
            burst_index,
            min_spawn_time_for_dir,
        )
        cars.append(car)
    return cars


def spawn_random_car(mode, current_time, scenario_index, burst_index, min_spawn_time_for_dir):
    direction = random.choice(DIRECTIONS)
    car = spawn_car(
        mode,
        direction,
        current_time,
        scenario_index,
        burst_index,
        min_spawn_time_for_dir,
    )
    return car


# ============================================================
# DRAWING
# ============================================================

def draw_roads(surface):
    road_width = LANE_OFFSET * 4

    v_rect = pygame.Rect(
        CENTER_X - road_width // 2,
        0,
        road_width,
        SCREEN_HEIGHT,
    )
    pygame.draw.rect(surface, ROAD_COLOR, v_rect)

    h_rect = pygame.Rect(
        0,
        CENTER_Y - road_width // 2,
        SCREEN_WIDTH,
        road_width,
    )
    pygame.draw.rect(surface, ROAD_COLOR, h_rect)

    inter_size = INTERSECTION_RADIUS * 2
    inter_rect = pygame.Rect(
        CENTER_X - inter_size // 2,
        CENTER_Y - inter_size // 2,
        inter_size,
        inter_size,
    )
    pygame.draw.rect(surface, INTERSECTION_COLOR, inter_rect)

    # Draw visible route lines in center square
    for path in CENTER_PATHS:
        if len(path) < 2:
            continue
        pygame.draw.lines(surface, PATH_COLOR, False, path, 1)


# ============================================================
# CRASH DETECTION & LOGGING
# ============================================================

def detect_and_log_crashes(cars, sim_time, crash_events, crashed_pairs, total_crashes):
    active = [c for c in cars if not c.finished]

    for i in range(len(active)):
        for j in range(i + 1, len(active)):
            ci = active[i]
            cj = active[j]
            ri = ci.current_rect()
            rj = cj.current_rect()

            if ri.colliderect(rj):
                pair_key = tuple(sorted((ci.id, cj.id)))
                if pair_key in crashed_pairs:
                    continue

                crashed_pairs.add(pair_key)
                total_crashes += 1

                ci.crashed = True
                cj.crashed = True
                ci.current_speed = 0.0
                cj.current_speed = 0.0
                ci.finished = True
                cj.finished = True

                total_liars = sum((not c.is_truthful) and (not c.finished) for c in cars)

                event = {
                    "sim_time": sim_time,
                    "total_liars": total_liars,
                    "pair_id": f"{ci.id}-{cj.id}",
                    "car_a_id": ci.id,
                    "car_a_direction": ci.direction,
                    "car_a_truthful": ci.is_truthful,
                    "car_a_scenario": ci.scenario_mode,
                    "car_a_burst": ci.burst_index,
                    "car_a_true_intent": ci.true_intent,
                    "car_a_reported_intent": ci.reported_intent,
                    "car_a_true_pref_speed": ci.true_pref_speed,
                    "car_a_reported_speed": ci.reported_speed,
                    "car_a_assigned_speed": ci.assigned_speed,
                    "car_a_current_speed": ci.current_speed,
                    "car_b_id": cj.id,
                    "car_b_direction": cj.direction,
                    "car_b_truthful": cj.is_truthful,
                    "car_b_scenario": cj.scenario_mode,
                    "car_b_burst": cj.burst_index,
                    "car_b_true_intent": cj.true_intent,
                    "car_b_reported_intent": cj.reported_intent,
                    "car_b_true_pref_speed": cj.true_pref_speed,
                    "car_b_reported_speed": cj.reported_speed,
                    "car_b_assigned_speed": cj.assigned_speed,
                    "car_b_current_speed": cj.current_speed,
                    # A-side instrumentation
                    "car_a_target_entry_time": getattr(ci, "target_entry_time", None),
                    "car_a_entry_time": getattr(ci, "entry_time", None),
                    "car_a_exit_time": getattr(ci, "exit_time", None),
                    "car_a_shield_stops": getattr(ci, "shield_stops", 0),
                    "car_a_shield_stop_time": getattr(ci, "shield_stop_time", 0.0),
                    # B-side instrumentation
                    "car_b_target_entry_time": getattr(cj, "target_entry_time", None),
                    "car_b_entry_time": getattr(cj, "entry_time", None),
                    "car_b_exit_time": getattr(cj, "exit_time", None),
                    "car_b_shield_stops": getattr(cj, "shield_stops", 0),
                    "car_b_shield_stop_time": getattr(cj, "shield_stop_time", 0.0),
                }
                crash_events.append(event)

    return total_crashes


def write_crash_log(filename, crash_events):
    if not crash_events:
        return
    fieldnames = list(crash_events[0].keys())
    with open(filename, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for ev in crash_events:
            writer.writerow(ev)


# ============================================================
# MAIN
# ============================================================

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("TMS: Truthful Safe, Liar Risky")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)

    tms = DynamicTMS()
    cars = []

    events = make_events_timeline()
    event_idx = 0

    sim_time = 0.0
    total_crashes = 0
    crash_events = []
    crashed_pairs = set()

    current_mode = events[0]["mode"] if events else "all_true"
    current_scenario_index = 0

    # per-direction next allowed spawn time
    min_spawn_time_for_dir = {d: 0.0 for d in DIRECTIONS}

    running = True

    congestion = []  # instrumentation: active vehicles over time

    trajectories = []  # per-frame logs for case study

    while running:
        dt = clock.tick(FPS) / 1000.0
        sim_time += dt

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Spawn events when their time is reached
        while event_idx < len(events) and sim_time >= events[event_idx]["time"]:
            ev = events[event_idx]
            current_mode = ev["mode"]
            current_scenario_index = ev["scenario_index"]

            if ev["kind"] == "burst":
                new_cars = spawn_standard_burst(
                    current_mode, sim_time, ev["scenario_index"], ev["burst_index"], min_spawn_time_for_dir
                )
            else:
                new_cars = [
                    spawn_random_car(
                        current_mode, sim_time, ev["scenario_index"], ev["burst_index"], min_spawn_time_for_dir
                    )
                ]

            cars.extend(new_cars)
            tms.assign_baseline_speeds_for_burst(new_cars)
            event_idx += 1

        # Apply TMS: baseline + truthful safety shield
        tms.apply_truthful_safety(cars)

        # instrumentation: accumulate shield stop time for this frame
        for car in cars:
            if getattr(car, "_shield_stopped_this_frame", False):
                car.shield_stop_time += dt
                car._shield_stopped_this_frame = False

        # Log per-frame data for all cars (you'll filter to the crash pair later)
        for car in cars:
            dx = car.pos[0] - CENTER_X
            dy = car.pos[1] - CENTER_Y
            trajectories.append({
                "time": sim_time,
                "car_id": car.id,
                "truthful": car.is_truthful,
                "direction": car.direction,
                "current_speed": car.current_speed,
                "assigned_speed": car.assigned_speed,
                "distance_to_center": (dx*dx + dy*dy) ** 0.5,
                "shield_stopped": getattr(car, "_shield_stopped_this_frame", False),
                "finished": car.finished
            })

        # Move cars
        for car in cars:
            car.update_with_current_speed(dt)

        # instrumentation: stamp first time at intersection entry
        for car in cars:
            if (not car.finished and car.entry_time is None and
                car.entry_idx_true is not None and car.route_pos >= car.entry_idx_true):
                car.entry_time = sim_time

        # Crash detect
        total_crashes = detect_and_log_crashes(
            cars, sim_time, crash_events, crashed_pairs, total_crashes
        )

        # instrumentation: stamp exit time for vehicles that just finished
        for car in cars:
            if car.finished and car.exit_time is None:
                car.exit_time = sim_time

        # Remove finished cars
        cars = [c for c in cars if not c.finished]

        # instrumentation: record congestion
        congestion.append((round(sim_time, 3), len(cars)))  # note the tuple

        # Auto-stop after all events + cars done
        if event_idx >= len(events) and not cars:
            running = False

        # Drawing
        screen.fill(BG_COLOR)
        draw_roads(screen)
        for car in cars:
            car.draw(screen)

        scenario_label = (
            "Scenario 1: All True"
            if current_scenario_index == 0
            else "Scenario 2: Mixed (Liars)"
        )
        text1 = font.render(
            f"{scenario_label}   Mode: {current_mode}",
            True,
            (230, 230, 230),
        )
        screen.blit(text1, (10, 10))

        active_liars = sum((not c.is_truthful) and (not c.finished) for c in cars)
        text2 = font.render(
            f"Time: {sim_time:4.1f}s   Active cars: {len(cars)}   Active liars: {active_liars}",
            True,
            (230, 230, 230),
        )
        screen.blit(text2, (10, 35))

        text3 = font.render(f"Total crashes: {total_crashes}", True, (255, 220, 0))
        screen.blit(text3, (10, 60))

        pygame.display.flip()

    pygame.quit()
    base_dir = r"D:/UF_Documents/UF_Coursework/Fall_2025/EEL5632_Safety_and_Security_of_Vehicular_Electronic_Systems/Project_EEL5632_Fall2025/traffic_sim_Motor_Knights/trial3_tms_dir_vel"
    os.makedirs(base_dir, exist_ok=True)  # ensure the directory exists
    file_path = os.path.join(base_dir, "tms_vel_crash_log.csv")
    # instrumentation: write congestion time series
    ts_path = os.path.join(base_dir, "active_cars_timeseries.csv")
    with open(ts_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time", "active_cars"])
        for row in congestion:            # iterate rows explicitly
            w.writerow(row)
    print(f"Saved congestion time series to: {ts_path}")

    # run only for trajectory case study
    write_crash_log(file_path, crash_events)
    traj_path = os.path.join(base_dir, "trajectories_case.csv")
    with open(traj_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(trajectories[0].keys()))
        w.writeheader()
        w.writerows(trajectories)
    print(f"Saved trajectories to: {traj_path}")
    sys.exit() 


if __name__ == "__main__":
    main()
