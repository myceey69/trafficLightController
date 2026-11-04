# Traffic Light Controller with Emergency Vehicle Priority — Short Report (draft)

## 1. Objective
Model an intersection controller as a Deterministic Finite Automaton (DFA) that cycles
Red → Green → Yellow, reacts to sensors, and gives absolute priority to emergency vehicles.

## 2. Formal definition
M = (Q, Σ, δ, q0, F)
- Q = {Red, Green, Yellow, Emergency_Active, Emergency_Cleared}
- Σ = {timer, sensor, emergency_trigger, emergency_clear}
- q0 = Red
- F = {Red, Green, Emergency_Active} (as specified in the assignment)
- δ: total function, 5 × 4 = 20 transitions (see `traffic_light/dfa.py`; diagrams in `docs/`).

## 3. Assignment inconsistency
Emergency_Cleared has an outgoing transition but no incoming one, so it is unreachable in the
literal table; it is also non-accepting. Proposed fix: Emergency_Active + emergency_clear →
Emergency_Cleared, then timer → Red (an all-red clearance interval before normal operation).
Both versions are implemented (`--recovery` flag) and tested (`reachable_states()`).

## 4. Completing δ
| State | sensor | emergency_clear | emergency_trigger |
|---|---|---|---|
| Red / Green / Yellow | self (counter++) | self (ignored) | Emergency_Active |
| Emergency_Active | self | Red (literal) or Emergency_Cleared | self |
| Emergency_Cleared | self | self | Emergency_Active |
Reason for self-loops: a DFA must be total; "no effect" is the safest behaviour for stray inputs.
Text outside Σ is rejected by the CLI (equivalent to a dead state that is never entered).

## 5. Extended state (optional extension)
Kept OUTSIDE the DFA so the five-state design is untouched:
- `green_axis` (NS or EW) alternates after each Yellow → Red;
- `waiting` vehicle counter from the sensor;
- output function: state → lights for both axes; an assertion forbids conflicting non-red lights.
Strictly, DFA + these variables is an extended FSM; the report should say so honestly.

## 6. Sensor integration
The sensor never changes the DFA's logic. Instead the controller (SimPy scheduler / Pygame world)
decides WHEN to emit `timer`: green lasts at least 4 s and ends early when nobody is approaching
on the green road; at most 12 s. This is "actuated" control built around an unchanged automaton.

## 7. Testing
`python -m unittest discover -v` — normal cycle, emergency from every normal state, ignoring
timer/sensor during emergency, both recovery variants, totality of δ, unreachable-state
detection, invalid input, safety under 2000 random inputs, deterministic seeded simulation,
headless Pygame run with an ambulance.

## 8. Limitations / future work
- Emergency direction unknown → everything red (could add direction-specific priority).
- Yellow before emergency red is skipped (immediate override, as specified).
- Real systems add minimum-green, all-red clearance and pedestrian phases.
