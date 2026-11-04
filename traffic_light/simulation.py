"""
simulation.py - OPTIONAL SimPy event-driven simulation (Phase 5).

SimPy only *generates events* (timer ticks, sensor pulses, emergencies).
Every decision about state is still made by the DFA through Intersection.process().
That is how sensor behaviour is added WITHOUT changing the DFA design:
the sensor-aware scheduler just decides WHEN to emit the `timer` symbol.
"""
import random
import simpy
from .dfa import Symbol, State
from .intersection import Intersection, NS, EW

# How long (simulated seconds) each state lasts before a `timer` is emitted.
BASE_DURATION = {State.RED: 2, State.GREEN: 8, State.YELLOW: 3, State.EMERGENCY_CLEARED: 2}
GREEN_EXTENSION = 4     # extra seconds if cars are still waiting on the green road
MAX_GREEN = 16


class Simulation:
    def __init__(self, seed=1, use_recovery_state=True, arrival_rate=0.4, verbose=False):
        self.env = simpy.Environment()
        self.box = Intersection(use_recovery_state)
        self.rng = random.Random(seed)
        self.arrival_rate = arrival_rate      # average cars/second (both roads combined)
        self.queues = {NS: 0, EW: 0}          # real queues; sensor reports them
        self.passed = 0
        self.log = []                         # (time, event, state)
        self.verbose = verbose
        self.emergency_response = []          # delay between trigger and Emergency_Active

    # ---- helpers ----
    def _send(self, symbol):
        old, new = self.box.process(symbol)
        self.log.append((round(self.env.now, 1), symbol.value, new.value))
        if self.verbose:
            print(f"t={self.env.now:5.1f}  {symbol.value:<18} {old.value:>17} -> {new.value}")

    # ---- processes ----
    def arrivals(self):
        """Cars arrive randomly on NS or EW and join a queue."""
        while True:
            yield self.env.timeout(self.rng.expovariate(self.arrival_rate))
            self.queues[self.rng.choice((NS, EW))] += 1

    def departures(self):
        """Every second, one car leaves the road that has a GREEN light."""
        while True:
            yield self.env.timeout(1)
            for axis, colour in self.box.lights().items():
                if colour == "GREEN" and self.queues[axis] > 0:
                    self.queues[axis] -= 1
                    self.passed += 1

    def sensor(self):
        """Loop detector: pulses `sensor` whenever somebody waits at a red light."""
        while True:
            yield self.env.timeout(1)
            if any(self.queues[a] > 0 and c == "RED" for a, c in self.box.lights().items()):
                self._send(Symbol.SENSOR)

    def scheduler(self):
        """Emits `timer`. Green is extended once if cars are still queued on the green road."""
        extended = 0
        while True:
            state = self.box.state
            if state == State.EMERGENCY_ACTIVE:
                yield self.env.timeout(1)       # timer is ignored by the DFA anyway
                continue
            duration = BASE_DURATION[state]
            yield self.env.timeout(duration)
            if self.box.state != state:         # an emergency interrupted us; re-evaluate
                extended = 0
                continue
            if state == State.GREEN:
                axis = self.box.green_axis
                if self.queues[axis] > 0 and extended + duration < MAX_GREEN:
                    extended += GREEN_EXTENSION
                    yield self.env.timeout(GREEN_EXTENSION)
                    if self.box.state != state:
                        extended = 0
                        continue
            extended = 0
            self._send(Symbol.TIMER)

    def emergency_vehicle(self, at, duration):
        yield self.env.timeout(at)
        before = self.env.now
        self._send(Symbol.EMERGENCY_TRIGGER)
        self.emergency_response.append(self.env.now - before)   # 0 = immediate
        yield self.env.timeout(duration)
        self._send(Symbol.EMERGENCY_CLEAR)

    def run(self, until=120, emergencies=((40, 10), (85, 6))):
        for p in (self.arrivals, self.departures, self.sensor, self.scheduler):
            self.env.process(p())
        for at, dur in emergencies:
            self.env.process(self.emergency_vehicle(at, dur))
        self.env.run(until=until)
        return {"passed": self.passed, "queues": dict(self.queues),
                "emergency_response": self.emergency_response}


if __name__ == "__main__":
    print(Simulation(verbose=True).run())
