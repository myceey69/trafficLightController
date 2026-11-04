# Traffic Light Controller with Emergency Priority (DFA)

<img width="1701" height="966" alt="image" src="https://github.com/user-attachments/assets/ee81a3bc-6433-475b-bda1-32f7c696f916" />

## Folder structure
```
main.py                            CLI (core, no dependencies)
visual.py                          Caltrans TMC Widescreen Console & Automata Inspector (Pygame)
requirements.txt                   simpy, pygame (optional parts only)
traffic_light/dfa.py               pure DFA: (Q, Σ, δ, q0, F)
traffic_light/intersection.py      extended state, lights, safety check
traffic_light/simulation.py        SimPy event simulation
traffic_light/verification.py      formal automata verification & civil safety audit
traffic_light/metrics.py           Highway Capacity Manual (HCM) Level of Service (LOS) telemetry
tests/                             unittest suites
docs/                              state diagrams, report.md, Caltrans screenshots
```

## Running the System
```bash
# Core CLI and formal verification
python main.py --recovery
python -m traffic_light.verification

# Automated test suite (17 tests)
python -m unittest discover -v

# Interactive Caltrans TMC Widescreen Console (1360x780)
python visual.py
```

### Interactive Simulation Hotkeys
* `E`: Dispatch Emergency Ambulance on North-South corridor
* `W`: Dispatch Emergency Ambulance on East-West corridor
* `1`: Morning Rush Hour Traffic Surge
* `SPACE`: Pause / Resume simulation
* `+` / `-`: Increase / Decrease simulation speed (0.5x to 5.0x)
* `V`: Print formal automata audit report to terminal
* `ESC`: Clean shutdown
Optional tests skip themselves if simpy/pygame are not installed.

## The assignment inconsistency
Emergency_Cleared has `timer → Red` but nothing leads into it (Emergency_Active + emergency_clear
goes directly to Red), so it is unreachable. Fix proposed: Active + clear → Cleared, then timer → Red.
Default = assignment as written; `--recovery` (and the visual/simulation) use the fix. Confirm with
your professor. Details and the completed transition table: `docs/report.md`.

## Known limitations
- `visual.py` verified only headless (screenshot in docs/); run it once on your screen.
- Graphviz is needed only to regenerate diagrams; the PNGs are included.
