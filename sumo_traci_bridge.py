"""
Minimal SUMO TraCI bridge for real-time traffic factor updates.
"""

from __future__ import annotations

import os
import sys
import logging
from typing import Optional

# Add SUMO tools to path if available.
if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

try:
    import traci  # type: ignore
    import sumolib  # type: ignore
    TRACI_AVAILABLE = True
except ImportError:
    traci = None
    sumolib = None
    TRACI_AVAILABLE = False

logger = logging.getLogger(__name__)


class SUMOTraCIBridge:
    """Bridge that exposes SUMO traffic state via TraCI."""

    def __init__(self, sumocfg_file: str, port: int = 8813, gui: bool = False):
        if not TRACI_AVAILABLE:
            raise ImportError(
                "TraCI not available. Install SUMO and set SUMO_HOME correctly."
            )
        if not os.path.exists(sumocfg_file):
            raise FileNotFoundError(f"SUMO config file not found: {sumocfg_file}")

        self.sumocfg_file = sumocfg_file
        self.port = port
        self.gui = gui
        self.connected = False
        self.simulation_time = 0.0

        sumo_binary = sumolib.checkBinary("sumo-gui" if gui else "sumo")
        sumo_cmd = [
            sumo_binary,
            "-c",
            sumocfg_file,
            "--start",
            "--step-length",
            "1.0",
            "--no-step-log",
            "--no-warnings",
        ]
        if not gui:
            sumo_cmd.append("--quit-on-end")

        traci.start(sumo_cmd, port=port)
        self.connected = True
        self.simulation_time = traci.simulation.getTime()

    def is_connected(self) -> bool:
        if not self.connected:
            return False
        try:
            traci.simulation.getTime()
            return True
        except Exception:
            self.connected = False
            return False

    def get_edge_speed(self, edge_id: str) -> Optional[float]:
        """Return current edge speed in m/s."""
        if not self.is_connected():
            return None
        try:
            speed_ms = traci.edge.getLastStepMeanSpeed(edge_id)
            if speed_ms is None or speed_ms <= 0:
                # Fallback to lane max speed when edge has no recent vehicles.
                lane_count = traci.edge.getLaneNumber(edge_id)
                if lane_count > 0:
                    speed_ms = traci.lane.getMaxSpeed(f"{edge_id}_0")
                else:
                    speed_ms = 13.89  # ~50 km/h fallback
            return float(speed_ms)
        except Exception:
            return None

    def calculate_traffic_factor(
        self, edge_id: str, base_speed_kmh: float = 50.0
    ) -> Optional[float]:
        """Convert current edge speed to a bounded traffic factor."""
        speed_ms = self.get_edge_speed(edge_id)
        if speed_ms is None or base_speed_kmh <= 0:
            return None
        speed_kmh = speed_ms * 3.6
        factor = speed_kmh / base_speed_kmh
        return max(0.7, min(1.6, factor))

    def advance_simulation(self, seconds: float) -> None:
        if not self.is_connected():
            return
        try:
            steps = max(0, int(seconds))
            for _ in range(steps):
                traci.simulationStep()
            self.simulation_time = traci.simulation.getTime()
        except Exception as exc:
            logger.warning("Failed to advance SUMO simulation: %s", exc)
            self.connected = False

    def close(self) -> None:
        if self.connected:
            try:
                traci.close()
            except Exception:
                pass
            finally:
                self.connected = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
