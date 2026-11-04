"""
metrics.py - Caltrans & Highway Capacity Manual (HCM) Operational Performance Metrics.

Calculates:
- Highway Capacity Manual (HCM) Level of Service (LOS A - F) based on control delay.
- Equivalent vehicles-per-hour (VPH) throughput.
- Queue length and density telemetry.
- Emergency Vehicle Preemption (EVP / Opticom) response efficiency metrics.
- Estimated emissions / idle fuel reduction.
"""
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class LOSRating:
    grade: str            # 'A', 'B', 'C', 'D', 'E', 'F'
    label: str            # e.g. "Free Flow", "Acceptable", "At Capacity", "Gridlock"
    color_rgb: Tuple[int, int, int]
    description: str


def get_hcm_los(avg_delay_seconds: float) -> LOSRating:
    """Determine HCM signalized intersection Level of Service based on control delay."""
    if avg_delay_seconds <= 10.0:
        return LOSRating("A", "Free Flow", (40, 200, 70), "Insignificant delay, optimal progression.")
    elif avg_delay_seconds <= 20.0:
        return LOSRating("B", "Good Flow", (100, 210, 60), "Minimal delay, highly acceptable operations.")
    elif avg_delay_seconds <= 35.0:
        return LOSRating("C", "Fair Flow", (220, 200, 40), "Stable flow, noticeable queues during peaks.")
    elif avg_delay_seconds <= 55.0:
        return LOSRating("D", "Approaching Cap.", (235, 140, 40), "Noticeable congestion, queue clearance delays.")
    elif avg_delay_seconds <= 80.0:
        return LOSRating("E", "At Capacity", (220, 80, 50), "Unstable flow, significant queuing at stop lines.")
    else:
        return LOSRating("F", "Forced / Jam", (220, 40, 40), "Gridlock condition; demand exceeds intersection capacity.")


class TrafficTelemetry:
    """Maintains real-time traffic operational telemetry for Caltrans & Public Works."""

    def __init__(self):
        self.total_spawned: int = 0
        self.total_cleared: int = 0
        self.completed_delays: List[float] = []
        self.evp_events_count: int = 0
        self.evp_response_times: List[float] = []
        self.evp_speed_bonus: float = 2.1        # Ambulances travel ~2.1x normal speed
        self.elapsed_sim_time: float = 0.0

    def record_spawn(self, is_emergency: bool = False):
        self.total_spawned += 1
        if is_emergency:
            self.evp_events_count += 1

    def record_vehicle_exit(self, delay_seconds: float, is_emergency: bool = False):
        self.total_cleared += 1
        if not is_emergency:
            self.completed_delays.append(max(0.0, delay_seconds))
            # Keep rolling window of last 100 vehicles for smooth real-time response
            if len(self.completed_delays) > 100:
                self.completed_delays.pop(0)

    def record_evp_lock(self, response_time_sec: float):
        self.evp_response_times.append(response_time_sec)

    @property
    def average_delay(self) -> float:
        if not self.completed_delays:
            return 4.2  # Default baseline for empty intersection
        return sum(self.completed_delays) / len(self.completed_delays)

    @property
    def los(self) -> LOSRating:
        return get_hcm_los(self.average_delay)

    @property
    def equivalent_vph(self) -> int:
        """Equivalent vehicles per hour throughput."""
        if self.elapsed_sim_time < 5.0:
            return 850
        hours = max(0.001, self.elapsed_sim_time / 3600.0)
        return int(self.total_cleared / hours)

    @property
    def co2_saved_grams(self) -> float:
        """Estimated grams of CO2 saved via sensor-actuation and preemption efficiency."""
        # Baseline idle: ~1.5g CO2/minute idling avoided
        saved_mins = (self.total_cleared * 6.5) / 60.0
        return round(saved_mins * 1.5, 1)

    @property
    def evp_avg_response_time(self) -> float:
        if not self.evp_response_times:
            return 0.0
        return sum(self.evp_response_times) / len(self.evp_response_times)
