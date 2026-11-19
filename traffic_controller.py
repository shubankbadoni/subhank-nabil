"""
Intelligent Traffic Signal Optimization Agent
CS 502 - Fundamentals of Artificial Intelligence

Rule-based adaptive controller for a two-stage SUMO junction.
The controller uses:
    Priority = 0.55 * normalized_queue
             + 0.30 * normalized_longest_wait
             + 0.15 * normalized_time_without_service

IMPORTANT:
- This is a configurable starter implementation based on the project report.
- You MUST fill in the actual SUMO lane IDs and traffic-light state strings
  from your own .net.xml network before running it.
- The simulation has not been validated merely by running this script.
"""

import argparse
import csv
import json
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional

try:
    import traci
except ImportError:
    traci = None


@dataclass
class PhaseMetrics:
    queue: int
    longest_wait: float
    red_time: float
    priority: float = 0.0


class TrafficSignalAgent:
    def __init__(self, config: dict, log_path: str):
        self.cfg = config
        self.tls_id = config["traffic_light_id"]
        self.phases = config["phases"]
        self.current_phase = config.get("initial_phase", "NS")
        self.min_green = float(config.get("min_green_seconds", 10))
        self.max_green = float(config.get("max_green_seconds", 60))
        self.decision_interval = float(config.get("decision_interval_seconds", 1))
        self.switch_margin = float(config.get("switching_margin", 0.05))
        self.yellow_duration = float(config.get("yellow_seconds", 3))
        self.all_red_duration = float(config.get("all_red_seconds", 1))

        self.reference_queue = float(config.get("reference_queue", 50))
        self.reference_wait = float(config.get("reference_wait_seconds", 120))
        self.reference_red = float(config.get("reference_no_service_seconds", 90))

        self.red_time = {name: 0.0 for name in self.phases}
        self.green_elapsed = 0.0
        self.last_decision_time = -1e9
        self.transitioning = False

        self.log_file = open(log_path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.log_file,
            fieldnames=[
                "time", "current_phase", "queue_NS", "wait_NS", "red_NS",
                "score_NS", "queue_EW", "wait_EW", "red_EW", "score_EW",
                "decision", "signal_state"
            ],
        )
        self.writer.writeheader()

    @staticmethod
    def normalize(value: float, reference: float) -> float:
        if reference <= 0:
            return 0.0
        return min(max(value / reference, 0.0), 1.0)

    def collect_metrics(self, phase_name: str) -> PhaseMetrics:
        lane_ids = self.phases[phase_name]["lanes"]
        total_queue = 0
        longest_wait = 0.0

        for lane_id in lane_ids:
            # Number of vehicles stopped at or below SUMO's halting threshold.
            total_queue += traci.lane.getLastStepHaltingNumber(lane_id)

            # Examine vehicles currently on this lane and find the longest
            # current waiting time among stopped vehicles.
            for vehicle_id in traci.lane.getLastStepVehicleIDs(lane_id):
                speed = traci.vehicle.getSpeed(vehicle_id)
                if speed <= 0.1:
                    wait = traci.vehicle.getWaitingTime(vehicle_id)
                    longest_wait = max(longest_wait, wait)

        red = self.red_time[phase_name]
        q_score = self.normalize(total_queue, self.reference_queue)
        w_score = self.normalize(longest_wait, self.reference_wait)
        t_score = self.normalize(red, self.reference_red)

        priority = 0.55 * q_score + 0.30 * w_score + 0.15 * t_score
        return PhaseMetrics(total_queue, longest_wait, red, priority)

    def choose_phase(self, metrics: Dict[str, PhaseMetrics]) -> str:
        """Choose the highest-scoring phase; ties keep the current phase."""
        other = "EW" if self.current_phase == "NS" else "NS"
        current_score = metrics[self.current_phase].priority
        other_score = metrics[other].priority

        now = traci.simulation.getTime()
        if self.green_elapsed < self.min_green:
            return self.current_phase

        if self.green_elapsed >= self.max_green:
            return other

        if other_score > current_score + self.switch_margin:
            return other

        return self.current_phase

    def set_state(self, state: str):
        traci.trafficlight.setRedYellowGreenState(self.tls_id, state)

    def transition_to(self, target_phase: str):
        """Run yellow -> all-red -> target green; never jump directly green-to-green."""
        current_cfg = self.phases[self.current_phase]
        target_cfg = self.phases[target_phase]

        self.transitioning = True
        self.set_state(current_cfg["yellow_state"])
        self._advance_for(self.yellow_duration)

        self.set_state(self.cfg["all_red_state"])
        self._advance_for(self.all_red_duration)

        self.set_state(target_cfg["green_state"])
        self.red_time[target_phase] = 0.0
        self.current_phase = target_phase
        self.green_elapsed = 0.0
        self.transitioning = False

    def _advance_for(self, duration: float):
        end_time = traci.simulation.getTime() + duration
        step = float(self.cfg.get("simulation_step_seconds", 1.0))
        while traci.simulation.getTime() + 1e-9 < end_time:
            traci.simulationStep()
            self._update_clocks(step)

    def _update_clocks(self, step: float):
        self.green_elapsed += step
        for phase_name in self.phases:
            if phase_name != self.current_phase:
                self.red_time[phase_name] += step

    def run(self):
        # Start with the configured initial green state.
        self.set_state(self.phases[self.current_phase]["green_state"])
        step = float(self.cfg.get("simulation_step_seconds", 1.0))

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            self._update_clocks(step)
            now = traci.simulation.getTime()

            if now - self.last_decision_time < self.decision_interval:
                continue
            self.last_decision_time = now

            metrics = {
                name: self.collect_metrics(name)
                for name in self.phases
            }
            target = self.choose_phase(metrics)
            decision = "HOLD" if target == self.current_phase else f"SWITCH_TO_{target}"

            current_state = traci.trafficlight.getRedYellowGreenState(self.tls_id)
            self.writer.writerow({
                "time": now,
                "current_phase": self.current_phase,
                "queue_NS": metrics["NS"].queue,
                "wait_NS": round(metrics["NS"].longest_wait, 2),
                "red_NS": round(metrics["NS"].red_time, 2),
                "score_NS": round(metrics["NS"].priority, 4),
                "queue_EW": metrics["EW"].queue,
                "wait_EW": round(metrics["EW"].longest_wait, 2),
                "red_EW": round(metrics["EW"].red_time, 2),
                "score_EW": round(metrics["EW"].priority, 4),
                "decision": decision,
                "signal_state": current_state,
            })
            self.log_file.flush()

            if target != self.current_phase:
                self.transition_to(target)

        self.log_file.close()


def main():
    parser = argparse.ArgumentParser(
        description="Run the adaptive Intelligent Traffic Signal Agent in SUMO."
    )
    parser.add_argument("--config", default="config.json", help="Path to controller config JSON")
    parser.add_argument("--sumocfg", required=True, help="Path to SUMO .sumocfg file")
    parser.add_argument("--gui", action="store_true", help="Launch sumo-gui instead of sumo")
    parser.add_argument("--log", default="adaptive_run.csv", help="CSV output path")
    args = parser.parse_args()

    if traci is None:
        sys.exit(
            "TraCI was not found. Install SUMO and configure its tools folder "
            "on PYTHONPATH, or use the Python environment supplied with SUMO."
        )

    with open(args.config, "r", encoding="utf-8") as f:
        config = json.load(f)

    sumo_binary = "sumo-gui" if args.gui else "sumo"
    sumo_cmd = [
        sumo_binary,
        "-c", args.sumocfg,
        "--step-length", str(config.get("simulation_step_seconds", 1.0)),
        "--no-warnings", "true",
    ]

    traci.start(sumo_cmd)
    try:
        agent = TrafficSignalAgent(config, args.log)
        agent.run()
    finally:
        traci.close()


if __name__ == "__main__":
    main()
