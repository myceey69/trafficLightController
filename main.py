"""
main.py - Command-line interface.
Run:  python main.py            (assignment-literal table)
      python main.py --recovery (uses the Emergency_Cleared fix)
"""
import logging
import sys
from traffic_light.dfa import Symbol
from traffic_light.intersection import Intersection

HELP = """Commands: timer | sensor | emergency_trigger | emergency_clear
          status | help | quit"""


def show(box):
    l = box.lights()
    print(f"  State: {box.state.value:<18} NS: {l['NS']:<6} EW: {l['EW']:<6} "
          f"waiting: {box.waiting}  accepting: {box.dfa.accepts()}")


def main():
    logging.basicConfig(filename="traffic.log", level=logging.INFO,
                        format="%(asctime)s %(message)s")
    box = Intersection(use_recovery_state="--recovery" in sys.argv)
    print("Traffic Light Controller (DFA)\n" + HELP)
    show(box)
    while True:
        try:
            cmd = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if cmd in ("quit", "exit"):
            break
        if cmd == "help":
            print(HELP)
        elif cmd == "status":
            pass
        else:
            try:
                box.process(Symbol(cmd))
            except ValueError:
                print(f"  Invalid command: {cmd!r}. Type 'help'.")
                continue
        show(box)


if __name__ == "__main__":
    main()
