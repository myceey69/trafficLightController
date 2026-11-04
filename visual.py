"""
visual.py - Caltrans & Municipal Public Works Edition.
Traffic Light Controller & Emergency Vehicle Preemption (EVP) System.

Features:
- Dual-canvas widescreen Caltrans Traffic Management Center (TMC) console (1360 x 780).
- Pure formal DFA core M = (Q, Sigma, delta, q0, F) with full totality and acceptance tracking.
- Highway Capacity Manual (HCM) Level of Service (LOS A - F) real-time analytics.
- Real-time interactive Automata Inspector with animated active states and transition edges.
- Realistic road geometry: asphalt textures, continental crosswalk markings, double-yellow
  centerlines, inductive loop sensors, smooth car deceleration, brake lights, and Opticom EVP strobe.
- One-click scenarios: Rush hour congestion, Code 3 priority, dual emergency arbitration.
- Retains 100% backward compatibility with headless automated tests (test_optional.py).
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
C_X, C_Y = SIM_W // 2, SCREEN_H // 2   # (400, 390)
ROAD = 110                            # road width
HALF_ROAD = ROAD // 2                 # 55

# Intersection box boundaries
BOX_TOP = C_Y - HALF_ROAD             # 335
BOX_BOTTOM = C_Y + HALF_ROAD          # 445
BOX_LEFT = C_X - HALF_ROAD            # 345
BOX_RIGHT = C_X + HALF_ROAD           # 455

# Stop line coordinates for backward-compatibility with tests
LINE = 335
LINE_NS = BOX_TOP
LINE_EW = BOX_LEFT

CAR_LEN, CAR_W, GAP = 34, 18, 12
SPEED, AMB_SPEED = 120, 240           # pixels per second
LANE = {NS: C_X - 26, EW: C_Y + 26}   # normal car lane
AMB_LANE = {NS: C_X + 26, EW: C_Y - 26} # emergency vehicle lane

MIN_GREEN, MAX_GREEN = 4.0, 14.0
FIXED_TIMINGS = {State.RED: 1.5, State.YELLOW: 2.8, State.EMERGENCY_CLEARED: 2.0}

# Professional Caltrans palette
COLOR_BG = (18, 22, 28)
COLOR_ASPHALT = (38, 41, 46)
COLOR_GRASS = (28, 48, 30)
COLOR_CURB = (58, 62, 70)
COLOR_YELLOW_LINE = (235, 195, 45)
COLOR_WHITE_LINE = (235, 238, 242)
COLOR_PANEL_BG = (22, 26, 33)
COLOR_CARD_BG = (30, 36, 46)
COLOR_CARD_BORDER = (46, 54, 68)
COLOR_TEXT_PRIMARY = (240, 242, 245)
COLOR_TEXT_MUTED = (145, 155, 170)
COLOR_ACCENT_BLUE = (50, 130, 240)
COLOR_ACCENT_AMBER = (245, 160, 35)

LIGHT_COLORS = {
    "RED": (245, 45, 45),
    "YELLOW": (245, 205, 30),
    "GREEN": (35, 215, 80),
}


# ----------------------------- VEHICLE MODEL -----------------------------
class Vehicle:
    """Microscopic vehicle model with smooth braking and lighting effects."""

    PALETTE = [
        (65, 120, 210),   # Oceanic blue
        (160, 85, 195),   # Royal violet
        (225, 130, 45),   # Sunset amber
        (65, 180, 170),   # Caltrans teal
        (185, 190, 200),  # Silver metallic
        (215, 55, 55),    # Crimson sedan
    ]

    def __init__(self, axis: str, emergency: bool = False):
        self.axis = axis
        self.emergency = emergency
        self.s: float = -CAR_LEN - 10.0
        self.target_speed: float = AMB_SPEED if emergency else SPEED
        self.current_speed: float = self.target_speed
        self.colour = (252, 252, 255) if emergency else random.choice(self.PALETTE)
        self.spawn_time = time.time()
        self.is_braking = False

    @property
    def front(self) -> float:
        return self.s + CAR_LEN / 2

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
        return LINE_NS if axis == NS else LINE_EW

    def waiting(self, axis: str) -> List[Vehicle]:
        sl = self.stop_line(axis)
        return [c for c in self.cars if c.axis == axis and not c.emergency and c.front <= sl + 4]

    def approaching(self, axis: str) -> List[Vehicle]:
        sl = self.stop_line(axis)
        return [c for c in self.waiting(axis) if c.front > sl - 220]

    def is_loop_occupied(self, axis: str) -> bool:
        """Inductive loop detector presence: vehicle over sensor zone."""
        sl = self.stop_line(axis)
        return any(c.axis == axis and (sl - 75 <= c.front <= sl + 5) for c in self.cars)

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
        if self.t_spawn > 1.2:
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
        lights = self.box.lights()
        for axis in (NS, EW):
            sl = self.stop_line(axis)
            road = sorted((c for c in self.cars if c.axis == axis and not c.emergency), key=lambda c: -c.s)
            leader = None
            for car in road:
                target_stop = None
                if lights[axis] != "GREEN" and car.front <= sl:
                    target_stop = sl - CAR_LEN / 2
                if leader is not None:
                    lead_stop = leader.s - CAR_LEN - GAP
                    target_stop = min(target_stop, lead_stop) if target_stop is not None else lead_stop

                if target_stop is not None:
                    dist = target_stop - car.s
                    if dist <= 0:
                        car.current_speed = 0.0
                        car.is_braking = True
                    elif dist < 130:
                        decel_factor = max(0.1, dist / 130.0)
                        car.current_speed = car.target_speed * decel_factor
                        car.is_braking = True
                    else:
                        car.current_speed = min(car.target_speed, car.current_speed + 160 * dt)
                        car.is_braking = False
                    car.s = min(target_stop, car.s + car.current_speed * dt)
                else:
                    car.current_speed = min(car.target_speed, car.current_speed + 160 * dt)
                    car.is_braking = False
                    car.s += car.current_speed * dt

                leader = car

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
def draw_halo_circle(surface: pygame.Surface, color: Tuple[int, int, int], center: Tuple[int, int], radius: int):
    """Draw a soft luminous halo with smooth radial alpha attenuation."""
    halo_surf = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
    cx, cy = radius, radius
    r, g, b = color
    for i in range(radius, 0, -2):
        alpha = int(35 * (1.0 - (i / radius)))
        pygame.draw.circle(halo_surf, (r, g, b, alpha), (cx, cy), i)
    surface.blit(halo_surf, (center[0] - cx, center[1] - cy), special_flags=pygame.BLEND_ADD)


def draw_light_head(screen: pygame.Surface, x: int, y: int, active_colour: str, vertical: bool = True):
    """Draw NEMA-standard traffic signal head with visor hoods and glowing lenses."""
    w, h = (24, 72) if vertical else (72, 24)
    # Signal backplate and housing
    pygame.draw.rect(screen, (16, 18, 22), (x - 2, y - 2, w + 4, h + 4), border_radius=5)
    pygame.draw.rect(screen, (45, 50, 60), (x, y, w, h), border_radius=4)

    names = ("RED", "YELLOW", "GREEN")
    for i, name in enumerate(names):
        if vertical:
            cx, cy = x + 12, y + 12 + 24 * i
        else:
            cx, cy = x + 12 + 24 * i, y + 12

        is_on = (name == active_colour)
        col = LIGHT_COLORS[name] if is_on else (40, 44, 50)

        if is_on:
            draw_halo_circle(screen, LIGHT_COLORS[name], (cx, cy), 15)
        pygame.draw.circle(screen, col, (cx, cy), 8)
        if is_on:
            pygame.draw.circle(screen, (255, 255, 255), (cx - 2, cy - 2), 2)


def draw_crosswalk(screen: pygame.Surface, rect: pygame.Rect, horizontal: bool = True):
    """Draw standard Caltrans continental (piano-key) crosswalk markings."""
    stripe_w = 14
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
    """Draw high-fidelity Caltrans intersection with asphalt, road markings, sensors, and vehicles."""
    # Base terrain
    pygame.draw.rect(screen, COLOR_GRASS, (0, 0, SIM_W, SCREEN_H))

    # Roadway asphalt
    road_rect_ns = pygame.Rect(BOX_LEFT, 0, ROAD, SCREEN_H)
    road_rect_ew = pygame.Rect(0, BOX_TOP, SIM_W, ROAD)
    pygame.draw.rect(screen, COLOR_ASPHALT, road_rect_ns)
    pygame.draw.rect(screen, COLOR_ASPHALT, road_rect_ew)

    # Concrete Curbs
    pygame.draw.rect(screen, COLOR_CURB, road_rect_ns, width=3)
    pygame.draw.rect(screen, COLOR_CURB, road_rect_ew, width=3)
    # Re-fill intersection center to eliminate inner curbs
    pygame.draw.rect(screen, COLOR_ASPHALT, (BOX_LEFT + 3, BOX_TOP + 3, ROAD - 6, ROAD - 6))

    # Caltrans Continental Crosswalks
    draw_crosswalk(screen, pygame.Rect(BOX_LEFT, BOX_TOP - 22, ROAD, 18), horizontal=True)
    draw_crosswalk(screen, pygame.Rect(BOX_LEFT, BOX_BOTTOM + 4, ROAD, 18), horizontal=True)
    draw_crosswalk(screen, pygame.Rect(BOX_LEFT - 22, BOX_TOP, 18, ROAD), horizontal=False)
    draw_crosswalk(screen, pygame.Rect(BOX_RIGHT + 4, BOX_TOP, 18, ROAD), horizontal=False)

    # Double Solid Yellow Centerlines
    pygame.draw.line(screen, COLOR_YELLOW_LINE, (C_X - 2, 0), (C_X - 2, BOX_TOP - 24), 2)
    pygame.draw.line(screen, COLOR_YELLOW_LINE, (C_X + 2, 0), (C_X + 2, BOX_TOP - 24), 2)
    pygame.draw.line(screen, COLOR_YELLOW_LINE, (C_X - 2, BOX_BOTTOM + 24), (C_X - 2, SCREEN_H), 2)
    pygame.draw.line(screen, COLOR_YELLOW_LINE, (C_X + 2, BOX_BOTTOM + 24), (C_X + 2, SCREEN_H), 2)

    pygame.draw.line(screen, COLOR_YELLOW_LINE, (0, C_Y - 2), (BOX_LEFT - 24, C_Y - 2), 2)
    pygame.draw.line(screen, COLOR_YELLOW_LINE, (0, C_Y + 2), (BOX_LEFT - 24, C_Y + 2), 2)
    pygame.draw.line(screen, COLOR_YELLOW_LINE, (BOX_RIGHT + 24, C_Y - 2), (SIM_W, C_Y - 2), 2)
    pygame.draw.line(screen, COLOR_YELLOW_LINE, (BOX_RIGHT + 24, C_Y + 2), (SIM_W, C_Y + 2), 2)

    # White Stop Bars (Caltrans Thermoplastic Stencils)
    pygame.draw.line(screen, COLOR_WHITE_LINE, (BOX_LEFT, LINE_NS), (C_X, LINE_NS), 5)
    pygame.draw.line(screen, COLOR_WHITE_LINE, (LINE_EW, C_Y), (LINE_EW, BOX_BOTTOM), 5)

    # Inductive Loop Detector Sensor Markings
    ns_loop_col = (50, 225, 110) if world.is_loop_occupied(NS) else (80, 95, 110)
    ew_loop_col = (50, 225, 110) if world.is_loop_occupied(EW) else (80, 95, 110)
    # NS Loop in normal lane
    pygame.draw.rect(screen, ns_loop_col, (C_X - 38, LINE_NS - 65, 24, 38), width=2, border_radius=3)
    # EW Loop in normal lane
    pygame.draw.rect(screen, ew_loop_col, (LINE_EW - 65, C_Y + 14, 38, 24), width=2, border_radius=3)

    # Traffic Signals
    lights = world.box.lights()
    draw_light_head(screen, BOX_LEFT - 32, LINE_NS - 78, lights[NS], vertical=True)
    draw_light_head(screen, LINE_EW - 78, BOX_BOTTOM + 10, lights[EW], vertical=False)

    # Opticom EVP Mast-Arm Detector & Strobe (Clean Caltrans optical receiver)
    strobe_x, strobe_y = BOX_LEFT - 20, LINE_NS - 92
    pygame.draw.rect(screen, (50, 55, 65), (strobe_x - 3, strobe_y - 8, 6, 16), border_radius=2)
    if world.box.state == State.EMERGENCY_ACTIVE:
        is_strobe_on = (int(time.time() * 12) % 2 == 0)
        strobe_col = (255, 255, 255) if is_strobe_on else (140, 150, 170)
        draw_halo_circle(screen, (255, 255, 255), (strobe_x, strobe_y), 10)
        pygame.draw.circle(screen, strobe_col, (strobe_x, strobe_y), 4)
    else:
        pygame.draw.circle(screen, (80, 90, 100), (strobe_x, strobe_y), 3)

    # Vehicles Drawing
    for car in world.cars:
        x, y = car.xy()
        w, h = (CAR_W, CAR_LEN) if car.axis == NS else (CAR_LEN, CAR_W)
        car_rect = pygame.Rect(x - w / 2, y - h / 2, w, h)

        # Vehicle Body Shadow & Base
        pygame.draw.rect(screen, (15, 18, 22), (car_rect.x + 3, car_rect.y + 3, w, h), border_radius=4)
        pygame.draw.rect(screen, car.colour, car_rect, border_radius=4)
        pygame.draw.rect(screen, (35, 40, 50), car_rect, width=1, border_radius=4)

        # Windshield
        if car.axis == NS:
            pygame.draw.rect(screen, (28, 42, 56), (car_rect.x + 2, car_rect.y + 8, w - 4, 7), border_radius=2)
        else:
            pygame.draw.rect(screen, (28, 42, 56), (car_rect.x + 8, car_rect.y + 2, 7, h - 4), border_radius=2)

        # Headlights projecting forward
        hl_color = (255, 255, 215)
        if car.axis == NS:
            pygame.draw.circle(screen, hl_color, (int(car_rect.left + 3), int(car_rect.bottom - 1)), 2)
            pygame.draw.circle(screen, hl_color, (int(car_rect.right - 3), int(car_rect.bottom - 1)), 2)
        else:
            pygame.draw.circle(screen, hl_color, (int(car_rect.right - 1), int(car_rect.top + 3)), 2)
            pygame.draw.circle(screen, hl_color, (int(car_rect.right - 1), int(car_rect.bottom - 3)), 2)

        # Brake Lights (Glow bright red when slowing down or waiting)
        if car.is_braking and not car.emergency:
            brake_col = (255, 30, 30)
            if car.axis == NS:
                pygame.draw.circle(screen, brake_col, (int(car_rect.left + 3), int(car_rect.top + 2)), 3)
                pygame.draw.circle(screen, brake_col, (int(car_rect.right - 3), int(car_rect.top + 2)), 3)
            else:
                pygame.draw.circle(screen, brake_col, (int(car_rect.left + 2), int(car_rect.top + 3)), 3)
                pygame.draw.circle(screen, brake_col, (int(car_rect.left + 2), int(car_rect.bottom - 3)), 3)

        # Emergency Siren Strobe Bar (Alternating Red/Blue High-Intensity Flashing)
        if car.emergency:
            strobe = (int(time.time() * 10) % 2 == 0)
            siren_col1 = (255, 20, 20) if strobe else (30, 100, 255)
            siren_col2 = (30, 100, 255) if strobe else (255, 20, 20)
            cx_c, cy_c = car_rect.center
            draw_halo_circle(screen, siren_col1, (cx_c - 4, cy_c), 12)
            draw_halo_circle(screen, siren_col2, (cx_c + 4, cy_c), 12)
            pygame.draw.circle(screen, siren_col1, (cx_c - 4, cy_c), 4)
            pygame.draw.circle(screen, siren_col2, (cx_c + 4, cy_c), 4)


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

    # Node Coordinates (Layout: Directed state cycle + Emergency branches)
    nodes = {
        State.RED: (dash_x + 85, 155),
        State.GREEN: (dash_x + 265, 120),
        State.YELLOW: (dash_x + 445, 155),
        State.EMERGENCY_ACTIVE: (dash_x + 175, 235),
        State.EMERGENCY_CLEARED: (dash_x + 355, 235),
    }

    # Draw Connecting Transition Edges
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

    # Draw State Nodes
    curr_state = world.box.state
    for state, (nx, ny) in nodes.items():
        is_current = (state == curr_state)
        is_accepting = (state in world.box.dfa.ACCEPTING)

        # Pulse halo for current active state
        if is_current:
            halo_color = (245, 50, 50) if "Emergency" in state.value else (50, 215, 90)
            pulse_rad = int(22 + 3 * math.sin(time.time() * 8))
            draw_halo_circle(screen, halo_color, (nx, ny), pulse_rad)

        node_bg = (38, 70, 110) if is_current else (28, 33, 42)
        node_border = (60, 200, 100) if is_current else (75, 88, 105)
        pygame.draw.circle(screen, node_bg, (nx, ny), 18)
        pygame.draw.circle(screen, node_border, (nx, ny), 18, width=2)

        # Double-ring for Formal Accepting States F per Automata Theory
        if is_accepting:
            pygame.draw.circle(screen, (180, 195, 210), (nx, ny), 21, width=1)

        # Node Label
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

    # Control Delay & Throughput Telemetry Columns
    avg_d = world.telemetry.average_delay
    vph = world.telemetry.equivalent_vph
    screen.blit(fonts["mono_12"].render(f"Avg Control Delay:  {avg_d:5.1f} s/veh ({los.label})", True, COLOR_TEXT_PRIMARY), (dash_x + 120, 330))
    screen.blit(fonts["mono_12"].render(f"Hourly Capacity:    {vph:,} VPH equivalent", True, COLOR_TEXT_PRIMARY), (dash_x + 120, 350))
    screen.blit(fonts["mono_12"].render(f"Total Cleared:      {world.telemetry.total_cleared} vehicles | CO2 Saved: ~{world.telemetry.co2_saved_grams}g", True, COLOR_TEXT_MUTED), (dash_x + 120, 370))

    # NEMA Phase & Timing Progress Bar
    pbar_rect = pygame.Rect(dash_x + 28, 400, dash_w - 56, 16)
    pygame.draw.rect(screen, (22, 25, 32), pbar_rect, border_radius=4)
    # Elapsed fill ratio
    max_t = MAX_GREEN if curr_state == State.GREEN else FIXED_TIMINGS.get(curr_state, 10.0)
    fill_ratio = min(1.0, world.t_state / max_t) if max_t > 0 else 0.0
    fill_w = int((dash_w - 56) * fill_ratio)
    if fill_w > 0:
        bar_col = LIGHT_COLORS.get(curr_state.value.upper(), COLOR_ACCENT_BLUE)
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

    # Font dictionary with resilient fallbacks
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

        # Scripted triggers for automated regression tests
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
