"""
intersection.py - Extended-state wrapper around the DFA (optional extension).

The DFA itself stays 5-state. The extras live HERE, as "context" data:
  * green_axis      : which road (NS or EW) gets the next green
  * waiting         : vehicles reported by the sensor
  * outputs         : light colour for North-South and East-West
A safety check guarantees we never show conflicting greens.
"""
import logging
from .dfa import TrafficDFA, State, Symbol

log = logging.getLogger("traffic")

NS, EW = "NS", "EW"
RED, YELLOW, GREEN = "RED", "YELLOW", "GREEN"


class SafetyError(Exception):
    pass


class Intersection:
    def __init__(self, use_recovery_state=False):
        self.dfa = TrafficDFA(use_recovery_state)
        self.green_axis = NS      # axis that will receive the next green
        self.waiting = 0          # vehicles detected by the sensor
        self.history = []         # list of (old, symbol, new)

    @property
    def state(self):
        return self.dfa.state

    def lights(self):
        """Output function: DFA state (+ extended data) -> signal colours."""
        s, ax = self.state, self.green_axis
        other = EW if ax == NS else NS
        if s == State.GREEN:
            out = {ax: GREEN, other: RED}
        elif s == State.YELLOW:
            out = {ax: YELLOW, other: RED}
        else:  # Red, Emergency_Active, Emergency_Cleared -> everything stopped
            out = {NS: RED, EW: RED}
        return out

    def _check_safety(self):
        l = self.lights()
        if l[NS] != RED and l[EW] != RED:
            raise SafetyError(f"Unsafe configuration: {l}")

    def process(self, symbol):
        """Feed one symbol; returns (old_state, new_state)."""
        old, new = self.dfa.step(symbol)

        # --- extended-state updates ---
        if symbol == Symbol.SENSOR and new != State.EMERGENCY_ACTIVE:
            self.waiting += 1
        if old == State.YELLOW and new == State.RED:   # cycle finished -> swap roads
            self.green_axis = EW if self.green_axis == NS else NS
        if old == State.RED and new == State.GREEN:
            self.waiting = 0                           # waiting cars get to go

        self._check_safety()
        self.history.append((old, symbol, new))
        if old != new:
            log.info("%s --%s--> %s", old.value, symbol.value, new.value)
        else:
            log.info("%s stays (input: %s)", old.value, symbol.value)
        return old, new
