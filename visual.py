"""
visual.py - OPTIONAL Pygame visualization (Phase 6).

Run:   python visual.py
Keys:  E = ambulance on the North-South road
       W = ambulance on the East-West road
       ESC = quit

All signal decisions come from traffic_light.intersection.Intersection (the DFA).
Pygame only DRAWS the state and produces inputs (timer / sensor / emergency_*).
"""
import os
import random
import sys

import pygame

from traffic_light.dfa import Symbol, State
from traffic_light.intersection import Intersection, NS, EW

SIZE, C = 800, 400                 # window size and centre
ROAD = 100                         # road width
LINE = 340                         # stop line coordinate (before the intersection box)
CAR_LEN, GAP = 30, 8
SPEED, AMB_SPEED = 110, 230        # pixels / second
LANE = {NS: 375, EW: 425}          # fixed lane coordinate for normal cars
AMB_LANE = {NS: 425, EW: 375}      # ambulances use the other lane
MIN_GREEN, MAX_GREEN = 4, 12
FIXED = {State.RED: 1.5, State.YELLOW: 2.5, State.EMERGENCY_CLEARED: 2.0}
COLOURS = {"RED": (220, 40, 40), "YELLOW": (240, 200, 30), "GREEN": (40, 200, 70)}


class Vehicle:
    def __init__(self, axis, emergency=False):
        self.axis, self.emergency = axis, emergency
        self.s = -CAR_LEN                                   # position along its road
        self.speed = AMB_SPEED if emergency else SPEED
        self.colour = (255, 255, 255) if emergency else random.choice(
            [(70, 130, 220), (150, 90, 200), (230, 140, 50), (90, 190, 190)])

    @property
    def front(self):
        return self.s + CAR_LEN / 2

    def xy(self):
        lane = AMB_LANE[self.axis] if self.emergency else LANE[self.axis]
        return (lane, self.s) if self.axis == NS else (self.s, lane)


class World:
    def __init__(self, use_recovery_state=True):
        self.box = Intersection(use_recovery_state)
        self.cars = []
        self.t_state = 0.0          # seconds spent in current DFA state
        self.t_sensor = 0.0
        self.t_spawn = 0.0
        self.events = []            # last few transitions, shown in the HUD
        self.last_state = self.box.state

    # ---- DFA plumbing ----
    def send(self, symbol):
        old, new = self.box.process(symbol)
        self.events.append(f"{symbol.value}: {old.value} -> {new.value}")
        self.events = self.events[-6:]

    def spawn_ambulance(self, axis):
        if self.box.state != State.EMERGENCY_ACTIVE:
            self.cars.append(Vehicle(axis, emergency=True))
            self.send(Symbol.EMERGENCY_TRIGGER)         # recognised immediately

    # ---- queries ----
    def waiting(self, axis):
        return [c for c in self.cars if c.axis == axis and not c.emergency and c.front <= LINE + 1]

    def approaching(self, axis):
        return [c for c in self.waiting(axis) if c.front > LINE - 200]

    # ---- one simulation step ----
    def update(self, dt):
        st = self.box.state
        if st != self.last_state:
            self.t_state, self.last_state = 0.0, st
        self.t_state += dt

        # random traffic
        self.t_spawn += dt
        if self.t_spawn > 1.3:
            self.t_spawn = 0
            axis = random.choice((NS, EW))
            if not any(c.axis == axis and c.s < CAR_LEN * 2 and not c.emergency for c in self.cars):
                self.cars.append(Vehicle(axis))

        # sensor pulse (loop detector): somebody waiting at a red light
        self.t_sensor += dt
        if self.t_sensor > 0.5:
            self.t_sensor = 0
            lights = self.box.lights()
            if any(lights[a] == "RED" and self.waiting(a) for a in (NS, EW)) \
                    and st != State.EMERGENCY_ACTIVE:
                self.send(Symbol.SENSOR)

        # timer logic (sensor-actuated green; the DFA is unchanged)
        if st == State.GREEN:
            axis = self.box.green_axis
            done = self.t_state >= MAX_GREEN or (
                self.t_state >= MIN_GREEN and not self.approaching(axis))
            if done:
                self.send(Symbol.TIMER)
        elif st in FIXED and self.t_state >= FIXED[st]:
            self.send(Symbol.TIMER)
        # Emergency_Active: no timer needed (DFA would ignore it anyway)

        self.move_cars(dt)

        # emergency over when every ambulance has left the screen
        if self.box.state == State.EMERGENCY_ACTIVE and not any(c.emergency for c in self.cars):
            self.send(Symbol.EMERGENCY_CLEAR)

    def move_cars(self, dt):
        lights = self.box.lights()
        for axis in (NS, EW):
            road = sorted((c for c in self.cars if c.axis == axis and not c.emergency),
                          key=lambda c: -c.s)
            leader = None
            for car in road:
                new_s = car.s + car.speed * dt
                if lights[axis] != "GREEN" and car.front <= LINE:       # red/yellow: stop at line
                    new_s = min(new_s, LINE - CAR_LEN / 2)
                if leader is not None:                                  # keep a gap
                    new_s = min(new_s, leader.s - CAR_LEN - GAP)
                car.s = max(car.s, new_s)
                leader = car
        for car in self.cars:
            if car.emergency:
                car.s += car.speed * dt
        self.cars = [c for c in self.cars if c.s < SIZE + 50]


# ------------------------------ drawing ------------------------------
def draw_light(screen, x, y, colour_name, vertical=True):
    pygame.draw.rect(screen, (30, 30, 30), (x, y, 22, 66) if vertical else (x, y, 66, 22), border_radius=4)
    for i, name in enumerate(("RED", "YELLOW", "GREEN")):
        cx, cy = (x + 11, y + 11 + 22 * i) if vertical else (x + 11 + 22 * i, y + 11)
        on = (name == colour_name)
        pygame.draw.circle(screen, COLOURS[name] if on else (70, 70, 70), (cx, cy), 8)


def draw(screen, font, world):
    screen.fill((60, 130, 60))
    pygame.draw.rect(screen, (50, 50, 50), (C - ROAD // 2, 0, ROAD, SIZE))
    pygame.draw.rect(screen, (50, 50, 50), (0, C - ROAD // 2, SIZE, ROAD))
    pygame.draw.line(screen, (240, 240, 240), (C - ROAD // 2, LINE), (C, LINE), 3)         # NS stop line
    pygame.draw.line(screen, (240, 240, 240), (LINE, C), (LINE, C + ROAD // 2), 3)         # EW stop line

    lights = world.box.lights()
    draw_light(screen, C - ROAD // 2 - 30, LINE - 70, lights[NS], True)
    draw_light(screen, LINE - 70, C + ROAD // 2 + 8, lights[EW], False)

    for car in world.cars:
        x, y = car.xy()
        w, h = (16, CAR_LEN) if car.axis == NS else (CAR_LEN, 16)
        pygame.draw.rect(screen, car.colour, (x - w / 2, y - h / 2, w, h), border_radius=4)
        if car.emergency:
            flash = (pygame.time.get_ticks() // 150) % 2
            pygame.draw.circle(screen, (255, 0, 0) if flash else (0, 80, 255), (int(x), int(y)), 5)

    st = world.box.state
    banner = (200, 30, 30) if "Emergency" in st.value else (20, 20, 20)
    pygame.draw.rect(screen, banner, (0, 0, SIZE, 30))
    screen.blit(font.render(f"State: {st.value}   NS: {lights[NS]}   EW: {lights[EW]}   "
                            f"accepting: {world.box.dfa.accepts()}", True, (255, 255, 255)), (8, 6))
    for i, line in enumerate(world.events[-6:]):
        screen.blit(font.render(line, True, (255, 255, 255)), (8, SIZE - 110 + 18 * i))
    screen.blit(font.render("E: ambulance NS | W: ambulance EW | ESC: quit", True, (255, 255, 255)), (8, 36))


def run(frames=None, screenshot=None, scripted=None, seed=None):
    """frames/screenshot/scripted allow headless testing: scripted = {frame_no: 'NS'|'EW'}."""
    if seed is not None:
        random.seed(seed)
    pygame.init()
    screen = pygame.display.set_mode((SIZE, SIZE))
    pygame.display.set_caption("Traffic Light Controller - DFA")
    font = pygame.font.SysFont("consolas,monospace", 14)
    clock = pygame.time.Clock()
    world = World()
    n, running = 0, True
    while running and (frames is None or n < frames):
        dt = 1 / 30 if frames is not None else clock.tick(60) / 1000
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_e:
                world.spawn_ambulance(NS)
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_w:
                world.spawn_ambulance(EW)
        if scripted and n in scripted:
            world.spawn_ambulance(scripted[n])
        world.update(dt)
        draw(screen, font, world)
        pygame.display.flip()
        if screenshot and n == screenshot[0]:
            pygame.image.save(screen, screenshot[1])
        n += 1
    pygame.quit()
    return world


if __name__ == "__main__":
    run()
