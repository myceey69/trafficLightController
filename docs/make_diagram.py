"""Generates the state diagram from the REAL transition table (so it can't drift)."""
import subprocess, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from collections import defaultdict
from traffic_light.dfa import build_delta, TrafficDFA, State

def make(recovery, out):
    delta = build_delta(recovery)
    edges = defaultdict(list)
    for (q, a), n in delta.items():
        edges[(q, n)].append(a.value)
    acc = TrafficDFA.ACCEPTING
    lines = ["digraph M {", "rankdir=LR;", 'node [shape=circle,fontsize=11];',
             'start [shape=point]; start -> Red;']
    for q in State:
        shape = "doublecircle" if q in acc else "circle"
        fill = "#ffd6d6" if "Emergency" in q.value else "#ffffff"
        lines.append(f'{q.value} [shape={shape},style=filled,fillcolor="{fill}"];')
    for (q, n), labels in edges.items():
        lines.append(f'{q.value} -> {n.value} [label="{", ".join(labels)}",fontsize=9];')
    lines.append("}")
    open(out + ".dot", "w").write("\n".join(lines))
    subprocess.run(["dot", "-Tpng", out + ".dot", "-o", out + ".png"], check=True)

here = os.path.dirname(__file__)
make(False, os.path.join(here, "dfa_literal"))
make(True, os.path.join(here, "dfa_recovery"))
