"""
visual.py - Caltrans & Municipal Public Works Edition (Daytime High-Visibility).
Traffic Light Controller & Emergency Vehicle Preemption (EVP) Simulation.

Features:
- Precision stop lines and safe standoff buffers before pedestrian crosswalks.
- Mathematical traffic light housing sizing: 3 vertically arranged circles fit strictly inside.
- Daytime environmental lighting: #76C66B natural green grass, #444B54 medium-dark asphalt,
  and crisp #FFFFFF continental crosswalk stencils.
- Microscopic vehicle car-following model with smooth acceleration/braking and non-overlapping queues.
- Real-time Caltrans TMC telemetry with live DFA Automata graph inspector.
- 100% backward compatibility with automated test suites (test_traffic.py, test_optional.py).
"""
import math
import os
import random
import sys
import time
from typing import Dict, List, Optional, Tuple

import pygame

from traffic_light.dfa import State, Symbol
from traffic_light.intersection import Intersection, NS, EW
from traffic_light.metrics import TrafficTelemetry, get_hcm_los
from traffic_light.verification import verify_dfa

# ----------------------------- DIMENSIONS & GEOMETRY -----------------------------
SCREEN_W, SCREEN_H = 1360, 780
SIM_W = 800
C_X, C_Y = SIM_W // 2, SCREEN_H // 2   # Center of intersection: (400, 390)
ROAD = 110                            # Total road width (2 lanes: 55px each)
HALF_ROAD = ROAD // 2                 # 55px

# Intersection box boundaries (conflict zone)
BOX_TOP = C_Y - HALF_ROAD             # 335
BOX_BOTTOM = C_Y + HALF_ROAD          # 445
BOX_LEFT = C_X - HALF_ROAD            # 345
BOX_RIGHT = C_X + HALF_ROAD           # 455

# Crosswalk geometry (symmetrical on all 4 approaches)
CROSSWALK_DEPTH = 20                  # Width of crossing strip along travel axis
CROSSWALK_GAP = 6                     # Distance from intersection box edge to crosswalk

# Stop Line Positioning (Mandatory vehicle limit lines)
# Must stop completely BEFORE the crosswalk with a safe buffer
STOP_LINE_BUFFER = 12                 # Safe distance between stop line and crosswalk edge
STOP_LINE_NS = BOX_TOP - CROSSWALK_GAP - CROSSWALK_DEPTH - STOP_LINE_BUFFER   # 335 - 6 - 20 - 12 = 297
STOP_LINE_EW = BOX_LEFT - CROSSWALK_GAP - CROSSWALK_DEPTH - STOP_LINE_BUFFER  # 345 - 6 - 20 - 12 = 307

# Exported for compatibility with legacy test references
LINE = STOP_LINE_NS
LINE_NS = STOP_LINE_NS
LINE_EW = STOP_LINE_EW

# Vehicle dimensions & dynamics
CAR_LEN, CAR_W, GAP = 34, 18, 14
SPEED, AMB_SPEED = 120.0, 240.0       # Pixels per second
LANE = {NS: C_X - 27, EW: C_Y + 27}   # Normal car travel lane (centered in 55px lane)
AMB_LANE = {NS: C_X + 27, EW: C_Y - 27} # Emergency vehicle clearance lane

MIN_GREEN, MAX_GREEN = 4.0, 14.0
FIXED_TIMINGS = {State.RED: 1.5, State.YELLOW: 2.8, State.EMERGENCY_CLEARED: 2.0}

# ----------------------------- TRAFFIC LIGHT HOUSING SPECS -----------------------------
# Every housing contains 3 vertically stacked circular lights (Red, Yellow, Green)
TL_HOUSING_W = 22                     # Outer width of rectangular housing
TL_HOUSING_H = 68                     # Outer height of rectangular housing
TL_LIGHT_RADIUS = 6                   # Circle radius (12px diameter < 22px housing width)
TL_PADDING_Y = 12                     # Vertical center offset for top light
TL_SPACING_Y = 22                     # Vertical distance between light centers

# ----------------------------- DAYTIME CALTRANS COLOR PALETTE -----------------------------
COLOR_GRASS = (118, 198, 107)         # #76C66B Bright natural green grass
COLOR_GRASS_ALT = (139, 207, 117)     # #8BCF75 Subtle mowing pattern accent
COLOR_ASPHALT = (68, 75, 84)          # #444B54 Medium-dark roadway asphalt
COLOR_CURB = (175, 184, 195)          # Concrete roadway curb border
COLOR_YELLOW_LINE = (247, 200, 51)    # MUTCD standard highway yellow
COLOR_WHITE_LINE = (255, 255, 255)    # Bright pure white thermoplastic markings

# Signal light colors
COLOR_TL_HOUSING = (24, 26, 30)       # Dark charcoal rectangular housing
COLOR_TL_BORDER = (48, 52, 60)        # Housing contour outline
SIGNAL_COLORS_ON = {
    "RED": (255, 42, 42),
    "YELLOW": (255, 210, 25),
    "GREEN": (35, 230, 85),
}
SIGNAL_COLORS_OFF = {
    "RED": (65, 24, 24),
    "YELLOW": (65, 55, 18),
    "GREEN": (18, 55, 28),
}

# Dashboard & Telemetry Palette
COLOR_PANEL_BG = (26, 31, 39)
COLOR_CARD_BG = (34, 41, 52)
COLOR_CARD_BORDER = (50, 60, 75)
COLOR_TEXT_PRIMARY = (245, 247, 250)
COLOR_TEXT_MUTED = (150, 162, 178)
COLOR_ACCENT_BLUE = (60, 140, 250)
COLOR_ACCENT_AMBER = (245, 165, 40)


# ----------------------------- VEHICLE MODEL -----------------------------
class Vehicle:
    """Microscopic vehicle model with car-following dynamics and safe bumper spacing."""

    PALETTE = [
        (55, 125, 225),   # Blue sedan
        (160, 85, 195),   # Purple sedan
        (235, 130, 45),   # Orange coupe
        (45, 185, 175),   # Teal hatchback
        (190, 195, 205),  # Silver compact
        (225, 55, 55),    # Red sedan
    ]

    def __init__(self, axis: str, emergency: bool = False):
        self.axis = axis
        self.emergency = emergency
        self.s: float = -CAR_LEN - 10.0
        self.target_speed: float = AMB_SPEED if emergency else SPEED
        self.current_speed: float = self.target_speed
        self.colour = (255, 255, 255) if emergency else random.choice(self.PALETTE)
        self.spawn_time = time.time()
        self.is_braking = False

    @property
    def front(self) -> float:
        """Front bumper coordinate along the road axis."""
        return self.s + CAR_LEN / 2.0

    @property
    def rear(self) -> float:
        """Rear bumper coordinate along the road axis."""
        return self.s - CAR_LEN / 2.0

    def xy(self) -> Tuple[float, float]:
        lane = AMB_LANE[self.axis] if self.emergency else LANE[self.axis]
        return (lane, self.s) if self.axis == NS else (self.s, lane)


# ----------------------------- SIMULATION WORLD -----------------------------
class World:
    """Manages intersection physics, DFA processing, and Caltrans telemetry."""

    def __init__(self, use_recovery_state: bool = True):
        self.box = Intersection(use_recovery_state)
        self.telemetry = TrafficTelemetry()
        self.cars: List[Vehicle] = []
        self.t_state: float = 0.0
        self.t_sensor: float = 0.0
        self.t_spawn: float = 0.0
        self.events: List[str] = []
        self.last_state = self.box.state
        self.sim_speed_multiplier: float = 1.0
        self.is_paused: bool = False
        self.active_preemption_channel: Optional[str] = None
        self.last_transition_symbol: Optional[Symbol] = None
        self.transition_flash_timer: float = 0.0
        self.verification_report = verify_dfa(use_recovery_state)

    def send(self, symbol: Symbol):
        old, new = self.box.process(symbol)
        self.last_transition_symbol = symbol
        self.transition_flash_timer = 0.6
        ev_msg = f"{symbol.value:<17} | {old.value} -> {new.value}"
        self.events.append(ev_msg)
        self.events = self.events[-8:]

    def spawn_ambulance(self, axis: str):
        self.cars.append(Vehicle(axis, emergency=True))
        self.telemetry.record_spawn(is_emergency=True)
        self.active_preemption_channel = f"{axis} Priority Corridor"
        if self.box.state != State.EMERGENCY_ACTIVE:
            self.send(Symbol.EMERGENCY_TRIGGER)
            self.telemetry.record_evp_lock(0.0)

    def stop_line(self, axis: str) -> float:
        return STOP_LINE_NS if axis == NS else STOP_LINE_EW

    def waiting(self, axis: str) -> List[Vehicle]:
        sl = self.stop_line(axis)
        return [c for c in self.cars if c.axis == axis and not c.emergency and c.front <= sl + 4]

    def approaching(self, axis: str) -> List[Vehicle]:
        sl = self.stop_line(axis)
        return [c for c in self.waiting(axis) if c.front > sl - 220]

    def is_loop_occupied(self, axis: str) -> bool:
        """Inductive loop detector presence: vehicle waiting over sensor zone."""
        sl = self.stop_line(axis)
        return any(c.axis == axis and (sl - 60 <= c.front <= sl + 4) for c in self.cars)

    def update(self, dt: float):
        if self.is_paused:
            return

        effective_dt = dt * self.sim_speed_multiplier
        self.telemetry.elapsed_sim_time += effective_dt

        if self.transition_flash_timer > 0:
            self.transition_flash_timer = max(0.0, self.transition_flash_timer - effective_dt)

        st = self.box.state
        if st != self.last_state:
            self.t_state, self.last_state = 0.0, st
        self.t_state += effective_dt

        # Vehicle Generation
        self.t_spawn += effective_dt
        if self.t_spawn > 1.25:
            self.t_spawn = 0.0
            axis = random.choice((NS, EW))
            if not any(c.axis == axis and c.s < CAR_LEN * 2 and not c.emergency for c in self.cars):
                self.cars.append(Vehicle(axis))
                self.telemetry.record_spawn(is_emergency=False)

        # Loop detector sensor pulses
        self.t_sensor += effective_dt
        if self.t_sensor > 0.4:
            self.t_sensor = 0.0
            lights = self.box.lights()
            if any(lights[a] == "RED" and self.waiting(a) for a in (NS, EW)) and st != State.EMERGENCY_ACTIVE:
                self.send(Symbol.SENSOR)

        # DFA Signal Phase Timing
        if st == State.GREEN:
            axis = self.box.green_axis
            done = (self.t_state >= MAX_GREEN) or (self.t_state >= MIN_GREEN and not self.approaching(axis))
            if done:
                self.send(Symbol.TIMER)
        elif st in FIXED_TIMINGS and self.t_state >= FIXED_TIMINGS[st]:
            self.send(Symbol.TIMER)

        self.move_cars(effective_dt)

        # Emergency vehicle clearance check
        if self.box.state == State.EMERGENCY_ACTIVE and not any(c.emergency for c in self.cars):
            self.send(Symbol.EMERGENCY_CLEAR)
            self.active_preemption_channel = None

    def move_cars(self, dt: float):
        """
        Microscopic vehicle movement with bumper-to-bumper collision safety
        and strict compliance with stop lines before pedestrian crosswalks.
        """
        lights = self.box.lights()
        for axis in (NS, EW):
            sl = self.stop_line(axis)
            road = sorted((c for c in self.cars if c.axis == axis and not c.emergency), key=lambda c: -c.s)
            leader = None

            for car in road:
                target_stop: Optional[float] = None
                light_status = lights[axis]

                # Stop line constraint: only applies before entering the intersection
                # Vehicles that already crossed the stop line continue through safely
                if light_status == "RED" and car.front <= sl:
                    # Vehicle stops with front bumper 2px before the stop line
                    target_stop = (sl - 2.0) - CAR_LEN / 2.0
                elif light_status == "YELLOW":
                    # Yellow light dilemma zone: stop if safe distance remains; otherwise proceed
                    if car.front <= sl - 35.0:
                        target_stop = (sl - 2.0) - CAR_LEN / 2.0

                # Queue constraint: maintain strict bumper-to-bumper spacing behind leader
                if leader is not None:
                    lead_stop = (leader.rear - GAP) - CAR_LEN / 2.0
                    target_stop = min(target_stop, lead_stop) if target_stop is not None else lead_stop

                if target_stop is not None:
                    dist = target_stop - car.s
                    if dist <= 0.0:
                        car.current_speed = 0.0
                        car.is_braking = True
                        car.s = target_stop
                    elif dist < 120.0:
                        # Smooth realistic deceleration curve
                        decel_factor = max(0.12, dist / 120.0)
                        car.current_speed = car.target_speed * decel_factor
                        car.is_braking = True
                        car.s = min(target_stop, car.s + car.current_speed * dt)
                    else:
                        car.current_speed = min(car.target_speed, car.current_speed + 140.0 * dt)
                        car.is_braking = False
                        car.s = min(target_stop, car.s + car.current_speed * dt)
                else:
                    # Green light / free flow acceleration
                    car.current_speed = min(car.target_speed, car.current_speed + 140.0 * dt)
                    car.is_braking = False
                    car.s += car.current_speed * dt

                leader = car

        # Emergency vehicles travel at high speed without stopping
        for car in self.cars:
            if car.emergency:
                car.s += car.target_speed * dt
                car.is_braking = False

        # Handle departed vehicles
        remaining: List[Vehicle] = []
        for c in self.cars:
            if c.s >= SCREEN_H + 60 or c.s >= SIM_W + 60:
                delay = (time.time() - c.spawn_time) - (CAR_LEN / SPEED)
                self.telemetry.record_vehicle_exit(delay, is_emergency=c.emergency)
            else:
                remaining.append(c)
        self.cars = remaining


# ----------------------------- RENDERING ENGINE -----------------------------
def draw_traffic_light(screen: pygame.Surface, x: int, y: int, active_colour: str):
    """
    Render rectangular traffic light housing containing exactly 3 vertically stacked circles.
    All 3 circles fit strictly INSIDE the housing without extending outside.
    """
    # 1. Dark rectangular housing
    housing_rect = pygame.Rect(x, y, TL_HOUSING_W, TL_HOUSING_H)
    pygame.draw.rect(screen, COLOR_TL_HOUSING, housing_rect, border_radius=4)
    pygame.draw.rect(screen, COLOR_TL_BORDER, housing_rect, width=2, border_radius=4)

    # 2. Three vertically arranged circular lights (Red top, Yellow middle, Green bottom)
    cx = x + TL_HOUSING_W // 2
    colors = ("RED", "YELLOW", "GREEN")

    for i, name in enumerate(colors):
        cy = y + TL_PADDING_Y + i * TL_SPACING_Y
        is_on = (name == active_colour)

        # Inactive base circle
        off_col = SIGNAL_COLORS_OFF[name]
        pygame.draw.circle(screen, off_col, (cx, cy), TL_LIGHT_RADIUS)
        pygame.draw.circle(screen, (38, 42, 50), (cx, cy), TL_LIGHT_RADIUS, width=1)

        # Active illuminated circle (strictly contained inside the housing)
        if is_on:
            on_col = SIGNAL_COLORS_ON[name]
            # Saturated core lens
            pygame.draw.circle(screen, on_col, (cx, cy), TL_LIGHT_RADIUS)
            # Subtle specular reflection dot inside lens
            pygame.draw.circle(screen, (255, 255, 255), (cx - 2, cy - 2), 2)


def draw_crosswalk(screen: pygame.Surface, rect: pygame.Rect, horizontal: bool = True):
    """Draw crisp Caltrans continental (piano-key) crosswalk markings."""
    stripe_w = 12
    stripe_gap = 10
    if horizontal:
        num = rect.width // (stripe_w + stripe_gap)
        start_x = rect.x + (rect.width - (num * (stripe_w + stripe_gap) - stripe_gap)) // 2
        for i in range(num):
            sx = start_x + i * (stripe_w + stripe_gap)
            pygame.draw.rect(screen, COLOR_WHITE_LINE, (sx, rect.y, stripe_w, rect.height))
    else:
        num = rect.height // (stripe_w + stripe_gap)
        start_y = rect.y + (rect.height - (num * (stripe_w + stripe_gap) - stripe_gap)) // 2
        for i in range(num):
            sy = start_y + i * (stripe_w + stripe_gap)
            pygame.draw.rect(screen, COLOR_WHITE_LINE, (rect.x, sy, rect.width, stripe_w))


def draw_intersection_canvas(screen: pygame.Surface, world: World):
    """Draw daytime high-contrast intersection with bright grass and clean asphalt."""
    # 1. Base terrain (bright natural grass with subtle landscape mowing stripes)
    screen.fill(COLOR_GRASS)
    for stripe_y in range(0, SCREEN_H, 60):
        pygame.draw.rect(screen, COLOR_GRASS_ALT, (0, stripe_y, SIM_W, 30))

    # 2. Roadway Asphalt (#444B54)
    road_rect_ns = pygame.Rect(BOX_LEFT, 0, ROAD, SCREEN_H)
    road_rect_ew = pygame.Rect(0, BOX_TOP, SIM_W, ROAD)
    pygame.draw.rect(screen, COLOR_ASPHALT, road_rect_ns)
    pygame.draw.rect(screen, COLOR_ASPHALT, road_rect_ew)

    # 3. Road Curb Borders
    pygame.draw.rect(screen, COLOR_CURB, road_rect_ns, width=3)
    pygame.draw.rect(screen, COLOR_CURB, road_rect_ew, width=3)
    # Re-fill intersection core
    pygame.draw.rect(screen, COLOR_ASPHALT, (BOX_LEFT + 3, BOX_TOP + 3, ROAD - 6, ROAD - 6))

    # 4. Symmetrical Continental Crosswalks
    draw_crosswalk(screen, pygame.Rect(BOX_LEFT, BOX_TOP - CROSSWALK_GAP - CROSSWALK_DEPTH, ROAD, CROSSWALK_DEPTH), horizontal=True)
    draw_crosswalk(screen, pygame.Rect(BOX_LEFT, BOX_BOTTOM + CROSSWALK_GAP, ROAD, CROSSWALK_DEPTH), horizontal=True)
    draw_crosswalk(screen, pygame.Rect(BOX_LEFT - CROSSWALK_GAP - CROSSWALK_DEPTH, BOX_TOP, CROSSWALK_DEPTH, ROAD), horizontal=False)
    draw_crosswalk(screen, pygame.Rect(BOX_RIGHT + CROSSWALK_GAP, BOX_TOP, CROSSWALK_DEPTH, ROAD), horizontal=False)

    # 5. Yellow Double Centerlines
    cw_outer_n = BOX_TOP - CROSSWALK_GAP - CROSSWALK_DEPTH - 2
    cw_outer_s = BOX_BOTTOM + CROSSWALK_GAP + CROSSWALK_DEPTH + 2
    cw_outer_w = BOX_LEFT - CROSSWALK_GAP - CROSSWALK_DEPTH - 2
    cw_outer_e = BOX_RIGHT + CROSSWALK_GAP + CROSSWALK_DEPTH + 2

    # NS Road Centerlines
    pygame.draw.line(screen, COLOR_YELLOW_LINE, (C_X - 2, 0), (C_X - 2, cw_outer_n), 2)
    pygame.draw.line(screen, COLOR_YELLOW_LINE, (C_X + 2, 0), (C_X + 2, cw_outer_n), 2)
    pygame.draw.line(screen, COLOR_YELLOW_LINE, (C_X - 2, cw_outer_s), (C_X - 2, SCREEN_H), 2)
    pygame.draw.line(screen, COLOR_YELLOW_LINE, (C_X + 2, cw_outer_s), (C_X + 2, SCREEN_H), 2)

    # EW Road Centerlines
    pygame.draw.line(screen, COLOR_YELLOW_LINE, (0, C_Y - 2), (cw_outer_w, C_Y - 2), 2)
    pygame.draw.line(screen, COLOR_YELLOW_LINE, (0, C_Y + 2), (cw_outer_w, C_Y + 2), 2)
    pygame.draw.line(screen, COLOR_YELLOW_LINE, (cw_outer_e, C_Y - 2), (SIM_W, C_Y - 2), 2)
    pygame.draw.line(screen, COLOR_YELLOW_LINE, (cw_outer_e, C_Y + 2), (SIM_W, C_Y + 2), 2)

    # 6. White Stop Lines (Located before the crosswalks)
    pygame.draw.line(screen, COLOR_WHITE_LINE, (BOX_LEFT, STOP_LINE_NS), (C_X, STOP_LINE_NS), 4)
    pygame.draw.line(screen, COLOR_WHITE_LINE, (STOP_LINE_EW, C_Y), (STOP_LINE_EW, BOX_BOTTOM), 4)

    # 7. Inductive Loop Sensors in pavement
    ns_loop_col = (40, 235, 90) if world.is_loop_occupied(NS) else (115, 128, 142)
    ew_loop_col = (40, 235, 90) if world.is_loop_occupied(EW) else (115, 128, 142)
    pygame.draw.rect(screen, ns_loop_col, (C_X - 40, STOP_LINE_NS - 52, 26, 36), width=2, border_radius=3)
    pygame.draw.rect(screen, ew_loop_col, (STOP_LINE_EW - 52, C_Y + 14, 36, 26), width=2, border_radius=3)

    # 8. Traffic Signal Lights (Placed safely outside travel lanes)
    lights = world.box.lights()
    # North-South Signal (Placed on NW curb, facing Southbound traffic)
    draw_traffic_light(screen, BOX_LEFT - TL_HOUSING_W - 12, STOP_LINE_NS - 45, lights[NS])
    # East-West Signal (Placed on SW curb, facing Eastbound traffic)
    draw_traffic_light(screen, STOP_LINE_EW - 45, BOX_BOTTOM + 12, lights[EW])

    # Opticom EVP Preemption Confirmation Strobe Indicator
    strobe_x, strobe_y = BOX_LEFT - TL_HOUSING_W - 1, STOP_LINE_NS - 55
    pygame.draw.rect(screen, (38, 42, 50), (strobe_x - 3, strobe_y - 6, 6, 12), border_radius=2)
    if world.box.state == State.EMERGENCY_ACTIVE:
        is_flash = (int(time.time() * 12) % 2 == 0)
        flash_col = (255, 255, 255) if is_flash else (140, 150, 165)
        pygame.draw.circle(screen, flash_col, (strobe_x, strobe_y), 4)
    else:
        pygame.draw.circle(screen, (75, 85, 95), (strobe_x, strobe_y), 3)

    # 9. Vehicles Rendering
    for car in world.cars:
        x, y = car.xy()
        w, h = (CAR_W, CAR_LEN) if car.axis == NS else (CAR_LEN, CAR_W)
        car_rect = pygame.Rect(x - w / 2, y - h / 2, w, h)

        # Vehicle Drop Shadow
        pygame.draw.rect(screen, (30, 35, 42), (car_rect.x + 2, car_rect.y + 2, w, h), border_radius=4)
        # Vehicle Body
        pygame.draw.rect(screen, car.colour, car_rect, border_radius=4)
        pygame.draw.rect(screen, (28, 32, 38), car_rect, width=1, border_radius=4)

        # Windshield
        if car.axis == NS:
            pygame.draw.rect(screen, (35, 52, 70), (car_rect.x + 2, car_rect.y + 8, w - 4, 7), border_radius=2)
        else:
            pygame.draw.rect(screen, (35, 52, 70), (car_rect.x + 8, car_rect.y + 2, 7, h - 4), border_radius=2)

        # Headlights projecting forward
        hl_col = (255, 255, 215)
        if car.axis == NS:
            pygame.draw.circle(screen, hl_col, (int(car_rect.left + 3), int(car_rect.bottom - 1)), 2)
            pygame.draw.circle(screen, hl_col, (int(car_rect.right - 3), int(car_rect.bottom - 1)), 2)
        else:
            pygame.draw.circle(screen, hl_col, (int(car_rect.right - 1), int(car_rect.top + 3)), 2)
            pygame.draw.circle(screen, hl_col, (int(car_rect.right - 1), int(car_rect.bottom - 3)), 2)

        # Brake Lights (Illuminate red when slowing down or waiting)
        if car.is_braking and not car.emergency:
            brk_col = (255, 35, 35)
            if car.axis == NS:
                pygame.draw.circle(screen, brk_col, (int(car_rect.left + 3), int(car_rect.top + 2)), 3)
                pygame.draw.circle(screen, brk_col, (int(car_rect.right - 3), int(car_rect.top + 2)), 3)
            else:
                pygame.draw.circle(screen, brk_col, (int(car_rect.left + 2), int(car_rect.top + 3)), 3)
                pygame.draw.circle(screen, brk_col, (int(car_rect.left + 2), int(car_rect.bottom - 3)), 3)

        # Emergency Siren Strobe Bar
        if car.emergency:
            strobe = (int(time.time() * 10) % 2 == 0)
            s1 = (255, 25, 25) if strobe else (30, 110, 255)
            s2 = (30, 110, 255) if strobe else (255, 25, 25)
            cx_c, cy_c = car_rect.center
            pygame.draw.circle(screen, s1, (cx_c - 4, cy_c), 4)
            pygame.draw.circle(screen, s2, (cx_c + 4, cy_c), 4)


def draw_dashboard_panel(screen: pygame.Surface, fonts: Dict[str, pygame.font.Font], world: World):
    """Render Caltrans Traffic Operations Center (TMC) Telemetry & Automata Inspector."""
    dash_x = SIM_W
    dash_w = SCREEN_W - SIM_W

    # Panel Base
    pygame.draw.rect(screen, COLOR_PANEL_BG, (dash_x, 0, dash_w, SCREEN_H))
    pygame.draw.line(screen, COLOR_CARD_BORDER, (dash_x, 0), (dash_x, SCREEN_H), 2)

    # 1. Header Banner
    header_rect = pygame.Rect(dash_x + 16, 12, dash_w - 32, 54)
    pygame.draw.rect(screen, COLOR_CARD_BG, header_rect, border_radius=6)
    pygame.draw.rect(screen, COLOR_CARD_BORDER, header_rect, width=1, border_radius=6)
    screen.blit(fonts["bold_15"].render("CALTRANS DISTRICT 7 | TRAFFIC OPERATIONS", True, (255, 255, 255)), (dash_x + 28, 18))
    screen.blit(fonts["mono_11"].render("NEMA TS2 ACTUATED CONTROLLER & FORMAL DFA INSPECTOR", True, COLOR_TEXT_MUTED), (dash_x + 28, 40))

    # 2. Automata Inspector Card (Interactive State Machine Visualizer)
    graph_rect = pygame.Rect(dash_x + 16, 74, dash_w - 32, 210)
    pygame.draw.rect(screen, COLOR_CARD_BG, graph_rect, border_radius=6)
    pygame.draw.rect(screen, COLOR_CARD_BORDER, graph_rect, width=1, border_radius=6)
    screen.blit(fonts["bold_13"].render("DFA TOPOLOGY & ACTIVE STATE INSPECTOR", True, COLOR_ACCENT_BLUE), (dash_x + 28, 82))

    nodes = {
        State.RED: (dash_x + 85, 155),
        State.GREEN: (dash_x + 265, 120),
        State.YELLOW: (dash_x + 445, 155),
        State.EMERGENCY_ACTIVE: (dash_x + 175, 235),
        State.EMERGENCY_CLEARED: (dash_x + 355, 235),
    }

    edges = [
        (State.RED, State.GREEN, "timer"),
        (State.GREEN, State.YELLOW, "timer"),
        (State.YELLOW, State.RED, "timer"),
        (State.RED, State.EMERGENCY_ACTIVE, "ev_trig"),
        (State.GREEN, State.EMERGENCY_ACTIVE, "ev_trig"),
        (State.YELLOW, State.EMERGENCY_ACTIVE, "ev_trig"),
        (State.EMERGENCY_ACTIVE, State.EMERGENCY_CLEARED, "ev_clear"),
        (State.EMERGENCY_CLEARED, State.RED, "timer"),
    ]

    for src, dst, label in edges:
        p1 = nodes[src]
        p2 = nodes[dst]
        is_active_edge = (world.transition_flash_timer > 0 and world.box.history and
                          world.box.history[-1][0] == src and world.box.history[-1][2] == dst)
        edge_col = (255, 225, 60) if is_active_edge else (70, 80, 95)
        edge_w = 3 if is_active_edge else 1
        pygame.draw.line(screen, edge_col, p1, p2, edge_w)

    curr_state = world.box.state
    for state, (nx, ny) in nodes.items():
        is_current = (state == curr_state)
        is_accepting = (state in world.box.dfa.ACCEPTING)

        node_bg = (38, 70, 110) if is_current else (28, 33, 42)
        node_border = (60, 220, 100) if is_current else (75, 88, 105)
        node_r = 19 if is_current else 17

        pygame.draw.circle(screen, node_bg, (nx, ny), node_r)
        pygame.draw.circle(screen, node_border, (nx, ny), node_r, width=2)

        if is_accepting:
            pygame.draw.circle(screen, (180, 195, 210), (nx, ny), node_r + 3, width=1)

        short_names = {
            State.RED: "Red",
            State.GREEN: "Green",
            State.YELLOW: "Yellow",
            State.EMERGENCY_ACTIVE: "EV_Act",
            State.EMERGENCY_CLEARED: "EV_Clr",
        }
        lbl = fonts["mono_10"].render(short_names[state], True, COLOR_TEXT_PRIMARY if is_current else COLOR_TEXT_MUTED)
        screen.blit(lbl, (nx - lbl.get_width() // 2, ny + 22))

    # 3. Telemetry & Level of Service (LOS) Card
    tel_rect = pygame.Rect(dash_x + 16, 292, dash_w - 32, 140)
    pygame.draw.rect(screen, COLOR_CARD_BG, tel_rect, border_radius=6)
    pygame.draw.rect(screen, COLOR_CARD_BORDER, tel_rect, width=1, border_radius=6)

    los = world.telemetry.los
    screen.blit(fonts["bold_13"].render("HIGHWAY CAPACITY MANUAL (HCM) PERFORMANCE", True, COLOR_ACCENT_AMBER), (dash_x + 28, 300))

    # LOS Badge
    los_badge_rect = pygame.Rect(dash_x + 28, 328, 80, 48)
    pygame.draw.rect(screen, (22, 26, 32), los_badge_rect, border_radius=6)
    pygame.draw.rect(screen, los.color_rgb, los_badge_rect, width=2, border_radius=6)
    los_txt = fonts["bold_20"].render(f"LOS {los.grade}", True, los.color_rgb)
    screen.blit(los_txt, (dash_x + 36, 338))

    avg_d = world.telemetry.average_delay
    vph = world.telemetry.equivalent_vph
    screen.blit(fonts["mono_12"].render(f"Avg Control Delay:  {avg_d:5.1f} s/veh ({los.label})", True, COLOR_TEXT_PRIMARY), (dash_x + 120, 330))
    screen.blit(fonts["mono_12"].render(f"Hourly Capacity:    {vph:,} VPH equivalent", True, COLOR_TEXT_PRIMARY), (dash_x + 120, 350))
    screen.blit(fonts["mono_12"].render(f"Total Cleared:      {world.telemetry.total_cleared} vehicles | CO2 Saved: ~{world.telemetry.co2_saved_grams}g", True, COLOR_TEXT_MUTED), (dash_x + 120, 370))

    # NEMA Phase & Timing Progress Bar
    pbar_rect = pygame.Rect(dash_x + 28, 400, dash_w - 56, 16)
    pygame.draw.rect(screen, (22, 25, 32), pbar_rect, border_radius=4)
    max_t = MAX_GREEN if curr_state == State.GREEN else FIXED_TIMINGS.get(curr_state, 10.0)
    fill_ratio = min(1.0, world.t_state / max_t) if max_t > 0 else 0.0
    fill_w = int((dash_w - 56) * fill_ratio)
    if fill_w > 0:
        bar_col = SIGNAL_COLORS_ON.get(curr_state.value.upper(), COLOR_ACCENT_BLUE)
        pygame.draw.rect(screen, bar_col, (dash_x + 28, 400, fill_w, 16), border_radius=4)
    screen.blit(fonts["mono_10"].render(f"State Elapsed: {world.t_state:4.1f}s / {max_t:.1f}s", True, COLOR_TEXT_PRIMARY), (dash_x + 36, 402))

    # 4. Emergency Vehicle Preemption (EVP / Opticom) Status Card
    evp_rect = pygame.Rect(dash_x + 16, 440, dash_w - 32, 100)
    pygame.draw.rect(screen, COLOR_CARD_BG, evp_rect, border_radius=6)
    pygame.draw.rect(screen, COLOR_CARD_BORDER, evp_rect, width=1, border_radius=6)

    is_evp = (curr_state == State.EMERGENCY_ACTIVE)
    evp_status_col = (255, 60, 60) if is_evp else (40, 200, 70)
    evp_hdr = "EVP PRIORITY PREEMPTION: [LOCKED - CODE 3]" if is_evp else "EVP PRIORITY PREEMPTION: [STANDBY - MONITORING]"
    screen.blit(fonts["bold_13"].render(evp_hdr, True, evp_status_col), (dash_x + 28, 448))

    evp_chan = world.active_preemption_channel or "No active Opticom optical strobes detected"
    screen.blit(fonts["mono_12"].render(f"Optical Channel:  {evp_chan}", True, COLOR_TEXT_PRIMARY), (dash_x + 28, 474))
    screen.blit(fonts["mono_12"].render("MUTCD EVP Latency: 0.00s (Deterministic Instantaneous Priority)", True, COLOR_ACCENT_AMBER), (dash_x + 28, 496))
    screen.blit(fonts["mono_12"].render(f"Active Preemption Dispatches Recorded: {world.telemetry.evp_events_count}", True, COLOR_TEXT_MUTED), (dash_x + 28, 516))

    # 5. Live Event Transition Journal (Console Output)
    log_rect = pygame.Rect(dash_x + 16, 548, dash_w - 32, 110)
    pygame.draw.rect(screen, (16, 18, 22), log_rect, border_radius=6)
    pygame.draw.rect(screen, COLOR_CARD_BORDER, log_rect, width=1, border_radius=6)
    screen.blit(fonts["mono_10"].render("SYS EVENT JOURNAL (INPUT SYMBOL -> STATE TRANSITION):", True, COLOR_ACCENT_BLUE), (dash_x + 24, 554))
    for i, line in enumerate(world.events[-4:]):
        screen.blit(fonts["mono_11"].render(line, True, (215, 225, 235)), (dash_x + 24, 574 + 18 * i))

    # 6. Interactive Command Controls Tray
    ctrl_rect = pygame.Rect(dash_x + 16, 666, dash_w - 32, 100)
    pygame.draw.rect(screen, COLOR_CARD_BG, ctrl_rect, border_radius=6)
    pygame.draw.rect(screen, COLOR_CARD_BORDER, ctrl_rect, width=1, border_radius=6)

    screen.blit(fonts["bold_11"].render("OPERATIONAL COMMAND HOTKEYS & SCENARIO DISPATCH:", True, COLOR_TEXT_PRIMARY), (dash_x + 24, 672))
    row1 = "[E] Dispatch Ambulance NS   | [W] Dispatch Ambulance EW   | [1] Rush Hour Surge"
    row2 = "[SPACE] Pause/Resume Sim    | [+/-] Adjust Speed (0.5x - 5x) | [V] Formal Audit"
    speed_txt = f"Sim Speed: {world.sim_speed_multiplier:.1f}x {'[PAUSED]' if world.is_paused else ''}"
    screen.blit(fonts["mono_10"].render(row1, True, COLOR_ACCENT_AMBER), (dash_x + 24, 694))
    screen.blit(fonts["mono_10"].render(row2, True, COLOR_TEXT_MUTED), (dash_x + 24, 714))
    screen.blit(fonts["mono_10"].render(f"{speed_txt} | [ESC] Shutdown", True, COLOR_ACCENT_BLUE), (dash_x + 24, 736))


# ----------------------------- MAIN RUN LOOP -----------------------------
def run(frames: Optional[int] = None, screenshot: Optional[Tuple[int, str]] = None,
        scripted: Optional[Dict[int, str]] = None, seed: Optional[int] = None) -> World:
    """
    Main entry point for simulation.
    Preserves full signature for headless testing suites (test_optional.py).
    """
    if seed is not None:
        random.seed(seed)

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Caltrans Traffic Operations Center - DFA Signal Controller & EVP")

    fonts = {
        "bold_20": pygame.font.SysFont("consolas,monospace", 20, bold=True),
        "bold_15": pygame.font.SysFont("consolas,monospace", 15, bold=True),
        "bold_13": pygame.font.SysFont("consolas,monospace", 13, bold=True),
        "bold_11": pygame.font.SysFont("consolas,monospace", 11, bold=True),
        "mono_12": pygame.font.SysFont("consolas,monospace", 12),
        "mono_11": pygame.font.SysFont("consolas,monospace", 11),
        "mono_10": pygame.font.SysFont("consolas,monospace", 10),
    }

    clock = pygame.time.Clock()
    world = World(use_recovery_state=True)
    n = 0
    running = True

    while running and (frames is None or n < frames):
        dt = 1.0 / 30.0 if frames is not None else clock.tick(60) / 1000.0

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_e:
                    world.spawn_ambulance(NS)
                elif ev.key == pygame.K_w:
                    world.spawn_ambulance(EW)
                elif ev.key == pygame.K_1:
                    for _ in range(4):
                        world.cars.append(Vehicle(NS))
                        world.cars.append(Vehicle(EW))
                elif ev.key == pygame.K_SPACE:
                    world.is_paused = not world.is_paused
                elif ev.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    world.sim_speed_multiplier = min(5.0, world.sim_speed_multiplier + 0.5)
                elif ev.key in (pygame.K_MINUS, pygame.K_UNDERSCORE):
                    world.sim_speed_multiplier = max(0.5, world.sim_speed_multiplier - 0.5)
                elif ev.key == pygame.K_v:
                    print(world.verification_report.summary())

        if scripted and n in scripted:
            world.spawn_ambulance(scripted[n])

        world.update(dt)

        # Dual-canvas rendering
        draw_intersection_canvas(screen, world)
        draw_dashboard_panel(screen, fonts, world)
        pygame.display.flip()

        if screenshot and n == screenshot[0]:
            pygame.image.save(screen, screenshot[1])

        n += 1

    pygame.quit()
    return world


if __name__ == "__main__":
    run()
