"""Tests for the optional SimPy / Pygame parts. Skipped automatically if libs are missing."""
import os
import unittest

try:
    import simpy  # noqa
    HAS_SIMPY = True
except ImportError:
    HAS_SIMPY = False

try:
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame  # noqa
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False


@unittest.skipUnless(HAS_SIMPY, "simpy not installed")
class TestSimulation(unittest.TestCase):
    def test_emergency_is_immediate_and_system_recovers(self):
        from traffic_light.simulation import Simulation
        from traffic_light.dfa import State
        sim = Simulation(seed=5)
        result = sim.run(until=120)
        self.assertEqual(result["emergency_response"], [0, 0])     # recognised at once
        # during each emergency window only emergency_* inputs may appear in the log
        window = [e for e in sim.log if 40 <= e[0] < 50]
        self.assertTrue(all(e[2] == State.EMERGENCY_ACTIVE.value for e in window))
        self.assertGreater(result["passed"], 0)                    # traffic kept flowing
        self.assertNotEqual(sim.box.state, State.EMERGENCY_ACTIVE) # recovered by t=120

    def test_deterministic_with_same_seed(self):
        from traffic_light.simulation import Simulation
        self.assertEqual(Simulation(seed=9).run(), Simulation(seed=9).run())


@unittest.skipUnless(HAS_PYGAME, "pygame not installed")
class TestVisual(unittest.TestCase):
    def test_headless_run_with_ambulance(self):
        import visual
        from traffic_light.dfa import Symbol
        world = visual.run(frames=1200, scripted={300: "NS"}, seed=1)
        symbols = [h[1] for h in world.box.history]
        self.assertIn(Symbol.EMERGENCY_TRIGGER, symbols)
        self.assertIn(Symbol.EMERGENCY_CLEAR, symbols)


if __name__ == "__main__":
    unittest.main()
