# Traffic Light Controller with Emergency Priority (DFA)

## Folder structure
```
main.py                 CLI (core, no dependencies)
visual.py               Pygame visualization (optional)
requirements.txt        simpy, pygame (optional parts only)
traffic_light/dfa.py            pure DFA: (Q, Σ, δ, q0, F)
traffic_light/intersection.py   extended state, lights, safety check
traffic_light/simulation.py     SimPy event simulation (optional)
tests/                  unittest suites
docs/                   state diagrams (PNG/DOT), report.md, screenshot
```

## Windows CMD
```
cd traffic_light_dfa
python main.py                 REM assignment-literal table
python main.py --recovery      REM with the Emergency_Cleared fix
python -m unittest discover -v

pip install -r requirements.txt
python -m traffic_light.simulation
python visual.py               REM E / W = ambulance on NS / EW road, ESC quits
python docs\make_diagram.py    REM needs Graphviz installed (dot)
```
Optional tests skip themselves if simpy/pygame are not installed.

## The assignment inconsistency
Emergency_Cleared has `timer → Red` but nothing leads into it (Emergency_Active + emergency_clear
goes directly to Red), so it is unreachable. Fix proposed: Active + clear → Cleared, then timer → Red.
Default = assignment as written; `--recovery` (and the visual/simulation) use the fix. Confirm with
your professor. Details and the completed transition table: `docs/report.md`.

## Known limitations
- `visual.py` verified only headless (screenshot in docs/); run it once on your screen.
- Graphviz is needed only to regenerate diagrams; the PNGs are included.
