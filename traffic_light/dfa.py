"""
dfa.py - The pure DFA: M = (Q, Sigma, delta, q0, F).

No third-party libraries, no printing, no I/O. This is the "mathematical" core
you will present to your professor.
"""
from enum import Enum


class State(Enum):
    RED = "Red"
    GREEN = "Green"
    YELLOW = "Yellow"
    EMERGENCY_ACTIVE = "Emergency_Active"
    EMERGENCY_CLEARED = "Emergency_Cleared"


class Symbol(Enum):
    TIMER = "timer"
    SENSOR = "sensor"
    EMERGENCY_TRIGGER = "emergency_trigger"
    EMERGENCY_CLEAR = "emergency_clear"


S, A = State, Symbol


def build_delta(use_recovery_state=False):
    """
    Build the COMPLETE transition function delta: Q x Sigma -> Q (5 x 4 = 20 entries).

    use_recovery_state=False -> follows the assignment literally:
        Emergency_Active + emergency_clear -> Red
        (Emergency_Cleared is then unreachable; see README for the inconsistency.)
    use_recovery_state=True  -> proposed fix:
        Emergency_Active + emergency_clear -> Emergency_Cleared
        Emergency_Cleared + timer          -> Red
    """
    delta = {}

    # --- Required by the assignment ---
    delta[(S.RED, A.TIMER)] = S.GREEN
    delta[(S.GREEN, A.TIMER)] = S.YELLOW
    delta[(S.YELLOW, A.TIMER)] = S.RED
    for q in (S.RED, S.GREEN, S.YELLOW):
        delta[(q, A.EMERGENCY_TRIGGER)] = S.EMERGENCY_ACTIVE
    delta[(S.EMERGENCY_ACTIVE, A.TIMER)] = S.EMERGENCY_ACTIVE
    delta[(S.EMERGENCY_ACTIVE, A.SENSOR)] = S.EMERGENCY_ACTIVE
    delta[(S.EMERGENCY_ACTIVE, A.EMERGENCY_CLEAR)] = (
        S.EMERGENCY_CLEARED if use_recovery_state else S.RED
    )
    delta[(S.EMERGENCY_CLEARED, A.TIMER)] = S.RED

    # --- Our additions (the assignment left these undefined) ---
    # Sensor: does not change the state (it only updates extended data elsewhere).
    for q in (S.RED, S.GREEN, S.YELLOW, S.EMERGENCY_CLEARED):
        delta[(q, A.SENSOR)] = q
    # emergency_clear with no emergency active: nothing to clear -> stay.
    for q in (S.RED, S.GREEN, S.YELLOW, S.EMERGENCY_CLEARED):
        delta[(q, A.EMERGENCY_CLEAR)] = q
    # A new emergency while already active / while recovering.
    delta[(S.EMERGENCY_ACTIVE, A.EMERGENCY_TRIGGER)] = S.EMERGENCY_ACTIVE
    delta[(S.EMERGENCY_CLEARED, A.EMERGENCY_TRIGGER)] = S.EMERGENCY_ACTIVE

    assert len(delta) == len(State) * len(Symbol), "delta must be total"
    return delta


class TrafficDFA:
    """A deterministic, complete finite automaton."""

    START = State.RED
    ACCEPTING = frozenset({State.RED, State.GREEN, State.EMERGENCY_ACTIVE})  # per assignment

    def __init__(self, use_recovery_state=False):
        self.delta = build_delta(use_recovery_state)
        self.state = self.START

    def step(self, symbol):
        """Apply one input symbol. Returns (old_state, new_state)."""
        if not isinstance(symbol, Symbol):
            raise ValueError(f"{symbol!r} is not in the input alphabet")
        old = self.state
        self.state = self.delta[(old, symbol)]
        return old, self.state

    def accepts(self):
        return self.state in self.ACCEPTING

    def reset(self):
        self.state = self.START

    def reachable_states(self):
        """Breadth-first search from q0 (used to detect unreachable states)."""
        seen, todo = {self.START}, [self.START]
        while todo:
            q = todo.pop()
            for a in Symbol:
                nxt = self.delta[(q, a)]
                if nxt not in seen:
                    seen.add(nxt)
                    todo.append(nxt)
        return seen
