import math
import random
import sys
import csv
import os
from typing import Dict, List
import pygame
import matplotlib
matplotlib.use("Agg")  # for headless save after pygame closes
import matplotlib.pyplot as plt
# ============================================================
# CONFIG
# ============================================================
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 800
FPS = 60
CENTER_X = SCREEN_WIDTH // 2
CENTER_Y = SCREEN_HEIGHT // 2
BG_COLOR = (25, 25, 25)
ROAD_COLOR = (60, 60, 60)
LINE_COLOR = (220, 220, 220)
CAR_COLOR = (80, 160, 255)  # all blue, as requested
CRASH_COLOR = (220, 50, 50)
TEXT_COLOR = (240, 240, 240)
# Geometry
ROAD_WIDTH = 220
CAR_SIZE = 18  # square cars
SPAWN_OFFSET = 260    # distance from center where cars initially queue
CAR_GAP = 6          # gap between queued cars (pixels)
CAR_SPEED = 160.0  # px / second across the intersection
# Experiment config
DIRECTIONS = ["N", "E", "S", "W"]
NUM_CARS_PER_DIR = 10
ARRIVAL_TIME_MAX = 60.0
RANDOM_SEED = 1234
# TMS decision cadence (seconds)
DECISION_DT = 1.0
# Case 1: fixed green duration per direction
FIXED_PHASE_DURATION = 20.0
INTENTS = ["S", "L", "R"]  # straight, left, right
# Output directory
OUTPUT_DIR = r"D:/UF_Documents/UF_Coursework/Fall_2025/EEL5632_Safety_and_Security_of_Vehicular_Electronic_Systems/Project_EEL5632_Fall2025/traffic_sim_Motor_Knights/trial3_tms_dir_vel"

# ============================================================
# DATA STRUCTURES
# ============================================================
class CarSpec:
    """Specification of a single car: shared across policies."""
    def __init__(self, direction: str, arrival_time: float, intent: str, cid: int):
        self.direction = direction
        self.arrival_time = arrival_time
        self.intent = intent
        self.cid = cid

class Car:
    """
    Physical/visual car in the Pygame simulation.
    States:
        - "waiting"  : sitting in queue before intersection
        - "moving"   : traversing intersection / exiting
        - "done"     : off screen
        - "crashed"  : has collided with another car
    """
    def __init__(self, spec: CarSpec):
        self.spec = spec
        self.direction = spec.direction
        self.intent = spec.intent
        self.state = "waiting"
        self.s = 0.0  # distance from spawn line along lane, increases toward and through intersection
        self.queue_index = 0
        self.color = CAR_COLOR
    def set_queue_index(self, idx: int):
        self.queue_index = idx
    def start_moving(self):
        if self.state == "waiting":
            self.state = "moving"
    def mark_crashed(self):
        self.state = "crashed"
        self.color = CRASH_COLOR
    def update(self, dt: float):
        if self.state == "moving":
            self.s += CAR_SPEED * dt
            # Once sufficiently far past center, mark done
            if self.s > SPAWN_OFFSET + 260:
                self.state = "done"
    def _base_pos_waiting(self):
        """
        Position for a waiting car, stacked behind intersection.
        queue_index 0 is the front car closest to the center.
        We offset cars backward from the "entry" to keep them from overlapping.
        """
        # distance from the intersection boundary along lane
        back_offset = (self.queue_index + 1) * (CAR_SIZE + CAR_GAP)
        if self.direction == "N":
            x = CENTER_X - ROAD_WIDTH * 0.25
            y = CENTER_Y - SPAWN_OFFSET - back_offset
        elif self.direction == "S":
            x = CENTER_X + ROAD_WIDTH * 0.25
            y = CENTER_Y + SPAWN_OFFSET + back_offset
        elif self.direction == "E":
            x = CENTER_X + SPAWN_OFFSET + back_offset
            y = CENTER_Y - ROAD_WIDTH * 0.25
        else:  # "W"
            x = CENTER_X - SPAWN_OFFSET - back_offset
            y = CENTER_Y + ROAD_WIDTH * 0.25
        return x, y
    def _base_pos_moving(self):
        """
        Position for a moving car: simple straight-line center paths.
        NOTE: for this experiment, we don't visually differentiate left/right/straight;
        intent influences which movements can go together, not the geometric path.
        """
        if self.direction == "N":
            x = CENTER_X - ROAD_WIDTH * 0.25
            y = CENTER_Y - SPAWN_OFFSET + self.s
        elif self.direction == "S":
            x = CENTER_X + ROAD_WIDTH * 0.25
            y = CENTER_Y + SPAWN_OFFSET - self.s
        elif self.direction == "E":
            x = CENTER_X + SPAWN_OFFSET - self.s
            y = CENTER_Y - ROAD_WIDTH * 0.25
        else:  # "W"
            x = CENTER_X - SPAWN_OFFSET + self.s
            y = CENTER_Y + ROAD_WIDTH * 0.25
        return x, y
    def get_rect(self):
        if self.state == "waiting":
            x, y = self._base_pos_waiting()
        else:
            x, y = self._base_pos_moving()
        rect = pygame.Rect(0, 0, CAR_SIZE, CAR_SIZE)
        rect.center = (int(x), int(y))
        return rect
    def draw(self, screen):
        rect = self.get_rect()
        pygame.draw.rect(screen, self.color, rect)

# ============================================================
# EXPERIMENT SETUP
# ============================================================
def generate_car_specs() -> Dict[str, List[CarSpec]]:
    """
    Generate car specifications for each direction:
      - direction in {N,E,S,W}
      - arrival_time ~ Uniform(0, ARRIVAL_TIME_MAX)
      - intent in {S,L,R}
    """
    random.seed(RANDOM_SEED)
    specs_by_dir: Dict[str, List[CarSpec]] = {d: [] for d in DIRECTIONS}
    cid = 0
    for d in DIRECTIONS:
        for _ in range(NUM_CARS_PER_DIR):
            t_arr = random.uniform(0.0, ARRIVAL_TIME_MAX)
            intent = random.choice(INTENTS)
            specs_by_dir[d].append(CarSpec(d, t_arr, intent, cid))
            cid += 1
    for d in DIRECTIONS:
        specs_by_dir[d].sort(key=lambda s: s.arrival_time)
    return specs_by_dir

# ============================================================
# COLLISION DETECTION
# ============================================================
def detect_collisions(cars: List[Car]) -> bool:
    """
    Returns True if any two moving cars overlap (crash).
    """
    crashed = False
    moving = [c for c in cars if c.state in ("moving", "crashed")]
    for i in range(len(moving)):
        rect_i = moving[i].get_rect()
        for j in range(i + 1, len(moving)):
            rect_j = moving[j].get_rect()
            if rect_i.colliderect(rect_j):
                if moving[i].state != "crashed":
                    moving[i].mark_crashed()
                if moving[j].state != "crashed":
                    moving[j].mark_crashed()
                crashed = True
    return crashed

# ============================================================
# EXPERIMENT CASE / POLICY
# ============================================================
class ExperimentCase:
    """
    A single policy/case using shared car specs.
    mode:
      1 -> fixed-time 4-phase
      2 -> max-queue selection
      3 -> intent-aware cooperative (opposite straight flows)
    """
    def __init__(self, mode: int, specs_by_dir: Dict[str, List[CarSpec]]):
        self.mode = mode
        self.specs_by_dir = specs_by_dir
        self.title = {
            1: "Case 1: Fixed 4-phase (20s each)",
            2: "Case 2: Max-Queue per second",
            3: "Case 3: Intent-aware (opp. straights)",
        }[mode]
        # Timekeeping
        self.time = 0.0
        self.dec_timer = 0.0
        self.finished = False
        self.crash_happened = False
        self.crash_time = None
        # Spawning
        self.next_spec_idx = {d: 0 for d in DIRECTIONS}
        self.queues: Dict[str, List[Car]] = {d: [] for d in DIRECTIONS}
        self.moving_cars: List[Car] = []
        # For fixed-phase policy
        self.phase_order = ["N", "E", "S", "W"]
        self.phase_idx = 0
        self.phase_time = 0.0
        # Metrics
        self.arrival_time: Dict[int, float] = {
            spec.cid: spec.arrival_time
            for d in DIRECTIONS
            for spec in specs_by_dir[d]
        }
        self.departure_time: Dict[int, float] = {}
        self.intent_by_cid: Dict[int, str] = {
            spec.cid: spec.intent for d in DIRECTIONS for spec in specs_by_dir[d]
        }
        self.dir_by_cid: Dict[int, str] = {
            spec.cid: spec.direction for d in DIRECTIONS for spec in specs_by_dir[d]
        }
        # Per-step logging (decision times)
        self.per_step_log = []  # list of dicts
        # For collecting per-step departures
        self.departures_this_step: Dict[str, int] = {d: 0 for d in DIRECTIONS}
    # ---------- helpers ----------
    def _spawn_arrivals(self):
        """Spawn cars whose arrival_time <= self.time."""
        for d in DIRECTIONS:
            specs = self.specs_by_dir[d]
            while self.next_spec_idx[d] < len(specs) and specs[self.next_spec_idx[d]].arrival_time <= self.time:
                spec = specs[self.next_spec_idx[d]]
                car = Car(spec)
                self.queues[d].append(car)
                self.next_spec_idx[d] += 1
    def _update_queue_indices(self):
        for d in DIRECTIONS:
            for idx, car in enumerate(self.queues[d]):
                car.set_queue_index(idx)
    def _all_cars_done(self) -> bool:
        all_spawned = all(self.next_spec_idx[d] >= len(self.specs_by_dir[d]) for d in DIRECTIONS)
        if not all_spawned:
            return False
        if any(self.queues[d] for d in DIRECTIONS):
            return False
        if any(c.state not in ("done", "crashed") for c in self.moving_cars):
            return False
        return True
    # ---------- TMS policies ----------
    def _serve_fixed_phase(self):
        """Case 1: only the active direction gets a 'green' for this decision interval."""
        active_dir = self.phase_order[self.phase_idx]
        # At each decision, allow at most 1 new car from active_dir if queue non-empty
        if self.queues[active_dir]:
            car = self.queues[active_dir].pop(0)
            car.start_moving()
            self.moving_cars.append(car)
            self.departure_time[car.spec.cid] = self.time
            self.departures_this_step[active_dir] += 1
        # Update phase timer
        self.phase_time += DECISION_DT
        if self.phase_time >= FIXED_PHASE_DURATION:
            self.phase_time = 0.0
            self.phase_idx = (self.phase_idx + 1) % len(self.phase_order)
    def _serve_max_queue(self):
        """Case 2: at each decision, choose direction with max queue length."""
        best_dir = None
        best_len = -1
        for d in DIRECTIONS:
            qlen = len(self.queues[d])
            if qlen > best_len:
                best_len = qlen
                best_dir = d
        if best_dir is not None and best_len > 0:
            car = self.queues[best_dir].pop(0)
            car.start_moving()
            self.moving_cars.append(car)
            self.departure_time[car.spec.cid] = self.time
            self.departures_this_step[best_dir] += 1
    def _serve_intent_aware(self):
        """
        Case 3: intent-aware.
        - If N & S both at front and both intent 'S', serve both.
        - Else if E & W both at front and both intent 'S', serve both.
        - Else serve the single car whose front arrival_time is earliest.
        """
        def front(d: str):
            return self.queues[d][0] if self.queues[d] else None
        frontN = front("N")
        frontS = front("S")
        frontE = front("E")
        frontW = front("W")
        # Try opposite straights
        if frontN and frontS and frontN.intent == "S" and frontS.intent == "S":
            for car in (frontN, frontS):
                self.queues[car.direction].pop(0)
                car.start_moving()
                self.moving_cars.append(car)
                self.departure_time[car.spec.cid] = self.time
                self.departures_this_step[car.direction] += 1
            return
        if frontE and frontW and frontE.intent == "S" and frontW.intent == "S":
            for car in (frontE, frontW):
                self.queues[car.direction].pop(0)
                car.start_moving()
                self.moving_cars.append(car)
                self.departure_time[car.spec.cid] = self.time
                self.departures_this_step[car.direction] += 1
            return
        # Otherwise, pick earliest-arrival front car
        best_car = None
        best_arr = math.inf
        for d in DIRECTIONS:
            c = front(d)
            if c is not None and c.spec.arrival_time < best_arr:
                best_arr = c.spec.arrival_time
                best_car = c
        if best_car:
            self.queues[best_car.direction].pop(0)
            best_car.start_moving()
            self.moving_cars.append(best_car)
            self.departure_time[best_car.spec.cid] = self.time
            self.departures_this_step[best_car.direction] += 1
    def _tms_decision_step(self):
        # Reset departures count
        for d in DIRECTIONS:
            self.departures_this_step[d] = 0
        if self.mode == 1:
            self._serve_fixed_phase()
        elif self.mode == 2:
            self._serve_max_queue()
        else:
            self._serve_intent_aware()
        # Log state at this decision time
        snapshot = {
            "time": self.time,
        }
        for d in DIRECTIONS:
            snapshot[f"q_{d}"] = len(self.queues[d])
            snapshot[f"depart_{d}"] = self.departures_this_step[d]
        snapshot["crash"] = 1 if self.crash_happened else 0
        self.per_step_log.append(snapshot)
    # ---------- MAIN UPDATE ----------
    def update(self, dt: float):
        if self.finished:
            return
        # advance time
        self.time += dt
        self.dec_timer += dt
        # spawn new cars
        self._spawn_arrivals()
        # TMS step at discrete intervals
        while self.dec_timer >= DECISION_DT:
            self.dec_timer -= DECISION_DT
            if not self.crash_happened:  # stop releasing cars after crash
                self._tms_decision_step()
        # update queue indexing for drawing
        self._update_queue_indices()
        # update moving cars
        for car in list(self.moving_cars):
            car.update(dt)
            if car.state == "done":
                self.moving_cars.remove(car)
        # collision detection
        if not self.crash_happened:
            if detect_collisions(self.moving_cars):
                self.crash_happened = True
                self.crash_time = self.time
        # termination condition
        if self._all_cars_done():
            self.finished = True
    # ---------- DRAW ----------
    def draw(self, screen, hud_font):
        # Queues
        for d in DIRECTIONS:
            for car in self.queues[d]:
                car.draw(screen)
        # Moving
        for car in self.moving_cars:
            car.draw(screen)
        # HUD
        title_surf = hud_font.render(self.title, True, TEXT_COLOR)
        screen.blit(title_surf, (10, 10))
        t_surf = hud_font.render(f"t = {self.time:5.1f}s", True, TEXT_COLOR)
        screen.blit(t_surf, (10, 30))
        y = 55
        for d in DIRECTIONS:
            txt = f"{d}: queue={len(self.queues[d])}"
            surf = hud_font.render(txt, True, TEXT_COLOR)
            screen.blit(surf, (10, y))
            y += 18
        if self.crash_happened:
            msg = f"CRASH at t={self.crash_time:5.1f}s"
            surf = hud_font.render(msg, True, CRASH_COLOR)
            screen.blit(surf, (10, y + 10))
    # ---------- METRICS & EXPORT ----------
    def build_car_log(self):
        rows = []
        for cid, arr in self.arrival_time.items():
            dep = self.departure_time.get(cid, self.time)
            wait = dep - arr
            rows.append(
                {
                    "car_id": cid,
                    "direction": self.dir_by_cid[cid],
                    "arrival_time": arr,
                    "departure_time": dep,
                    "wait_time": wait,
                    "intent": self.intent_by_cid[cid],
                }
            )
        return rows

# ============================================================
# DRAWING HELPERS
# ============================================================
def draw_roads(screen):
    screen.fill(BG_COLOR)
    # vertical
    pygame.draw.rect(
        screen,
        ROAD_COLOR,
        pygame.Rect(CENTER_X - ROAD_WIDTH // 2, 0, ROAD_WIDTH, SCREEN_HEIGHT),
    )
    # horizontal
    pygame.draw.rect(
        screen,
        ROAD_COLOR,
        pygame.Rect(0, CENTER_Y - ROAD_WIDTH // 2, SCREEN_WIDTH, ROAD_WIDTH),
    )
    # intersection box
    inter = pygame.Rect(
        CENTER_X - ROAD_WIDTH // 2,
        CENTER_Y - ROAD_WIDTH // 2,
        ROAD_WIDTH,
        ROAD_WIDTH,
    )
    pygame.draw.rect(screen, (50, 50, 50), inter)
    # center cross lines
    pygame.draw.line(screen, LINE_COLOR, (CENTER_X, 0), (CENTER_X, SCREEN_HEIGHT), 2)
    pygame.draw.line(screen, LINE_COLOR, (0, CENTER_Y), (SCREEN_WIDTH, CENTER_Y), 2)

# ============================================================
# METRICS + CSV + PLOTS
# ============================================================
def compute_metrics(case_name: str, case: ExperimentCase):
    # per-car stats
    rows = case.build_car_log()
    waits = [r["wait_time"] for r in rows]
    avg_wait = sum(waits) / len(waits)
    max_wait = max(waits)
    per_dir_avg = {}
    for d in DIRECTIONS:
        w = [r["wait_time"] for r in rows if r["direction"] == d]
        per_dir_avg[d] = sum(w) / len(w) if w else 0.0
    total_time = case.time
    crash = case.crash_happened
    print(f"{case_name}:")
    print(f"  total_time = {total_time:.1f}s")
    print(f"  avg_wait   = {avg_wait:.2f}s")
    print(f"  max_wait   = {max_wait:.2f}s")
    print(f"  crashed    = {crash}")
    for d in DIRECTIONS:
        print(f"    {d}: avg_wait = {per_dir_avg[d]:.2f}s")
    print()
    return {
        "case_name": case_name,
        "total_time": total_time,
        "avg_wait": avg_wait,
        "max_wait": max_wait,
        "crashed": crash,
        "per_dir_avg": per_dir_avg,
        "rows": rows,
        "per_step": case.per_step_log,
    }

def save_csv_and_plots(results: List[dict]):
    # Ensure output directory exists
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Save CSV logs
    for res in results:
        base = res["case_name"].replace(" ", "_").replace(":", "")
        car_csv = os.path.join(OUTPUT_DIR, f"{base}_cars.csv")
        step_csv = os.path.join(OUTPUT_DIR, f"{base}_steps.csv")
        with open(car_csv, "w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "car_id",
                    "direction",
                    "arrival_time",
                    "departure_time",
                    "wait_time",
                    "intent",
                ],
            )
            writer.writeheader()
            for row in res["rows"]:
                writer.writerow(row)
        with open(step_csv, "w", newline="") as f:
            # infer keys from first row
            if res["per_step"]:
                fieldnames = list(res["per_step"][0].keys())
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for row in res["per_step"]:
                    writer.writerow(row)

    # Comparison plots
    labels = [r["case_name"] for r in results]
    total_times = [r["total_time"] for r in results]
    avg_waits = [r["avg_wait"] for r in results]
    crashed_flags = [1 if r["crashed"] else 0 for r in results]
    x = range(len(labels))

    plt.figure(figsize=(6, 4))
    plt.bar(x, total_times)
    plt.xticks(x, labels, rotation=15)
    plt.ylabel("Total completion time (s)")
    plt.title("Total Completion Time by TMS Policy")
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "exp_total_time.png"), dpi=150)

    plt.figure(figsize=(6, 4))
    plt.bar(x, avg_waits)
    plt.xticks(x, labels, rotation=15)
    plt.ylabel("Average waiting time (s)")
    plt.title("Average Waiting Time by TMS Policy")
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "exp_avg_wait.png"), dpi=150)

    plt.figure(figsize=(6, 4))
    plt.bar(x, crashed_flags)
    plt.xticks(x, labels, rotation=15)
    plt.ylabel("Crash occurred (0/1)")
    plt.title("Crash Outcomes by Policy")
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "exp_crashes.png"), dpi=150)

# ============================================================
# MAIN – RUN ALL THREE CASES IN PYGAME, THEN EXPORT
# ============================================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("4-way TMS Experiment – 3 Policies")
    clock = pygame.time.Clock()
    hud_font = pygame.font.SysFont("consolas", 18)
    specs_by_dir = generate_car_specs()
    cases = [
        ExperimentCase(1, specs_by_dir),
        ExperimentCase(2, specs_by_dir),
        ExperimentCase(3, specs_by_dir),
    ]
    case_names = [
        "Case1_Fixed",
        "Case2_MaxQueue",
        "Case3_IntentAware",
    ]
    current_index = 0
    current_case = cases[current_index]
    pause_after_finish = 2.0
    pause_timer = 0.0
    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        if current_case.finished:
            pause_timer += dt
            if pause_timer >= pause_after_finish:
                current_index += 1
                if current_index >= len(cases):
                    running = False
                else:
                    current_case = cases[current_index]
                    pause_timer = 0.0
        else:
            current_case.update(dt)
        draw_roads(screen)
        current_case.draw(screen, hud_font)
        pygame.display.flip()
    pygame.quit()
    # After visualization, compute metrics + save CSVs + plots
    results = []
    for case_name, case in zip(case_names, cases):
        res = compute_metrics(case_name, case)
        results.append(res)
    save_csv_and_plots(results)
    print(f"Experiment finished. CSVs and plots have been generated in:\n{OUTPUT_DIR}")
    sys.exit()

if __name__ == "__main__":
    main()