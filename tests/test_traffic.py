import unittest
from traffic_light.dfa import TrafficDFA, State, Symbol, build_delta
from traffic_light.intersection import Intersection, NS, EW, GREEN, RED, YELLOW

T, SE, ET, EC = Symbol.TIMER, Symbol.SENSOR, Symbol.EMERGENCY_TRIGGER, Symbol.EMERGENCY_CLEAR


class TestDFA(unittest.TestCase):
    def run_seq(self, dfa, *syms):
        for s in syms:
            dfa.step(s)
        return dfa.state

    def test_initial_state(self):
        self.assertEqual(TrafficDFA().state, State.RED)

    def test_normal_cycle(self):
        d = TrafficDFA()
        self.assertEqual(self.run_seq(d, T), State.GREEN)
        self.assertEqual(self.run_seq(d, T), State.YELLOW)
        self.assertEqual(self.run_seq(d, T), State.RED)

    def test_emergency_from_each_normal_state(self):
        for prefix in ([], [T], [T, T]):
            d = TrafficDFA()
            self.run_seq(d, *prefix)
            self.assertEqual(self.run_seq(d, ET), State.EMERGENCY_ACTIVE)

    def test_ignores_timer_and_sensor_during_emergency(self):
        d = TrafficDFA()
        self.run_seq(d, ET)
        self.assertEqual(self.run_seq(d, T, SE, T, SE), State.EMERGENCY_ACTIVE)

    def test_recovery_literal(self):
        d = TrafficDFA()
        self.assertEqual(self.run_seq(d, ET, EC), State.RED)

    def test_recovery_with_cleared_state(self):
        d = TrafficDFA(use_recovery_state=True)
        self.assertEqual(self.run_seq(d, ET, EC), State.EMERGENCY_CLEARED)
        self.assertEqual(self.run_seq(d, T), State.RED)

    def test_delta_is_total(self):
        for flag in (False, True):
            delta = build_delta(flag)
            for q in State:
                for a in Symbol:
                    self.assertIn((q, a), delta)

    def test_cleared_unreachable_in_literal_table(self):
        self.assertNotIn(State.EMERGENCY_CLEARED, TrafficDFA().reachable_states())
        self.assertIn(State.EMERGENCY_CLEARED, TrafficDFA(True).reachable_states())

    def test_invalid_symbol_rejected(self):
        with self.assertRaises(ValueError):
            TrafficDFA().step("banana")

    def test_stray_clear_is_ignored(self):
        d = TrafficDFA()
        self.assertEqual(self.run_seq(d, EC), State.RED)


class TestIntersection(unittest.TestCase):
    def test_axes_alternate_and_never_conflict(self):
        b = Intersection()
        b.process(T)
        self.assertEqual(b.lights(), {NS: GREEN, EW: RED})
        b.process(T)
        self.assertEqual(b.lights(), {NS: YELLOW, EW: RED})
        b.process(T)
        b.process(T)
        self.assertEqual(b.lights(), {EW: GREEN, NS: RED})

    def test_emergency_all_red(self):
        b = Intersection()
        b.process(T)
        b.process(ET)
        self.assertEqual(b.lights(), {NS: RED, EW: RED})

    def test_sensor_counts_waiting_but_not_in_emergency(self):
        b = Intersection()
        b.process(SE); b.process(SE)
        self.assertEqual(b.waiting, 2)
        b.process(ET); b.process(SE)
        self.assertEqual(b.waiting, 2)

    def test_random_walk_always_safe(self):
        import random
        random.seed(1)
        b = Intersection(True)
        for _ in range(2000):
            b.process(random.choice(list(Symbol)))  # SafetyError would fail the test


if __name__ == "__main__":
    unittest.main()
