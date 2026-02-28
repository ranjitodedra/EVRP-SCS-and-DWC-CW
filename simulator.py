"""
Route decoder / simulator.

Given a customer-visit order, this module expands it into a full route
(including intermediate shortest-path nodes and charging-station detours)
and returns every metric required by the objective function and the
output report.

Charging logic
──────────────
* **Eq 4.2** — before every edge, check
      E_current ≤ γ·E_bat  OR  E_current ≤ E_edge
  If true → charge (or detour to nearest reachable CS first).
* **Policy B** — charge only what is needed (+ safety margin) to
  reach the next CS / depot.  Fallback to full charge (Policy A).
* **DWC** — while traversing an electric road, the battery gains
  E_DWC = P_chg · η · L / v  (Eq 8).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

from energy import (
    compute_actual_speed,
    compute_travel_time,
    compute_energy_consumption,
    compute_dwc_energy_gain,
    compute_charging_time,
)
from config import BATTERY_THRESHOLD, PENALTY_BATTERY_DEAD


# ── result containers ──────────────────────────────────────────

@dataclass
class ChargingStop:
    station_id: str
    time_charged: float    # hours
    energy_added: float    # kWh


@dataclass
class RouteResult:
    objective_value: float
    route_sequence: List[str]
    total_travel_time: float               # hours
    total_distance: float                  # km
    total_energy_consumed: float           # kWh
    total_energy_charged_static: float     # kWh
    total_energy_charged_dynamic: float    # kWh
    final_battery_level: float             # kWh
    final_battery_percent: float           # %
    packages_delivered: int
    charging_stops: List[ChargingStop]
    feasible: bool
    # trails
    soc_trail: List[Tuple[str, float]]                         # (node, soc%)
    edge_details: List[Tuple[str, str, float, float, float]]   # (from, to, energy, time_min, dist)
    # objective components
    sum_arrival_times: float
    sum_battery_levels: float


# ── simulator ──────────────────────────────────────────────────

class Simulator:
    """Stateful route decoder — one instance per graph."""

    def __init__(self, graph):
        self.graph = graph
        self.params = graph.params

    # ── public API ─────────────────────────────────────────────

    def simulate(self, customer_order) -> RouteResult:
        """Decode *customer_order* into a full route and return metrics."""
        self._init(customer_order)

        destinations = list(customer_order) + [self.graph.depot]
        for dest in destinations:
            self._navigate_to(dest)
            if not self._feasible:
                break
            # deliver
            if dest != self.graph.depot and dest in self.graph.customers:
                self._packages_delivered += 1
                self._load -= self.params["package_weight"]

        return self._build_result()

    # ── initialisation ─────────────────────────────────────────

    def _init(self, customer_order):
        E_bat = self.params["battery_capacity"]
        self._E_bat = E_bat
        self._E_current = self.params["initial_battery_percent"] / 100.0 * E_bat
        self._gamma = BATTERY_THRESHOLD

        self._vehicle_mass = self.params["vehicle_mass"]
        self._load = len(customer_order) * self.params["package_weight"]

        self._total_time = 0.0
        self._total_dist = 0.0
        self._total_energy_consumed = 0.0
        self._total_energy_charged_static = 0.0
        self._total_energy_charged_dynamic = 0.0

        depot = self.graph.depot
        self._current = depot
        self._route = [depot]
        self._soc_trail = [(depot, self._E_current / E_bat * 100)]
        self._edge_details: list = []
        self._charging_stops: list = []

        self._sum_arr = 0.0                 # sum of arrival times
        self._sum_bat = self._E_current     # sum of battery levels
        self._feasible = True
        self._packages_delivered = 0

    # ── navigation ─────────────────────────────────────────────

    def _navigate_to(self, dest: str):
        """Move from *self._current* to *dest*, handling charging."""
        MAX_STEPS = 10_000
        for _ in range(MAX_STEPS):
            if self._current == dest:
                return
            if not self._feasible:
                return

            _, path = self.graph.dijkstra(self._current, dest)
            if not path or len(path) < 2:
                self._feasible = False
                return

            next_node = path[1]
            edge = self.graph.get_edge(self._current, next_node)
            if edge is None:
                self._feasible = False
                return

            E_edge = self._edge_energy(edge)

            # Eq 4.2 — charging decision
            need = (self._E_current <= self._gamma * self._E_bat) or \
                   (self._E_current <= E_edge)

            if need:
                if self._try_charge(dest):
                    continue                       # re-evaluate path
                if self._E_current > E_edge:
                    pass                           # can still traverse
                else:
                    self._feasible = False
                    return

            self._traverse(self._current, next_node, edge)

        # safety: exceeded step limit
        if self._current != dest:
            self._feasible = False

    # ── edge traversal ─────────────────────────────────────────

    def _traverse(self, from_node: str, to_node: str, edge: dict):
        is_elec = edge["type"] == "electric"
        speed = compute_actual_speed(
            self.params["base_speed"], edge["traffic_factor"],
            is_elec, self.params["electric_road_speed"],
        )
        total_mass = self._vehicle_mass + self._load
        E_edge = compute_energy_consumption(
            edge["distance"], speed, total_mass, self.params,
        )
        t = compute_travel_time(edge["distance"], speed)

        # deplete battery
        self._E_current -= E_edge
        self._total_energy_consumed += E_edge
        self._total_time += t
        self._total_dist += edge["distance"]

        # DWC gain on electric roads
        if is_elec:
            gain = compute_dwc_energy_gain(
                edge["distance"], speed,
                self.params["dwc_power"], self.params["dwc_efficiency"],
            )
            self._E_current += gain
            self._E_current = min(self._E_current, self._E_bat)
            self._total_energy_charged_dynamic += gain

        if self._E_current < 0:
            self._feasible = False
            self._E_current = 0.0

        # book-keeping
        self._current = to_node
        self._route.append(to_node)
        self._sum_arr += self._total_time
        self._sum_bat += self._E_current
        self._edge_details.append(
            (from_node, to_node, E_edge, t * 60, edge["distance"])
        )
        self._soc_trail.append(
            (to_node, self._E_current / self._E_bat * 100)
        )

    # ── charging helpers ───────────────────────────────────────

    def _try_charge(self, dest: str) -> bool:
        """Attempt to charge; return True if charging occurred."""
        # already at a CS → charge here
        if self.graph.nodes[self._current]["type"] == "charging_station":
            self._do_charge(dest)
            return True

        # find the nearest reachable CS
        cs = self._nearest_reachable_cs()
        if cs is None:
            return False

        # navigate to that CS (straight traversal, no recursion)
        _, cs_path = self.graph.dijkstra(self._current, cs)
        if not cs_path or len(cs_path) < 2:
            return False

        for k in range(len(cs_path) - 1):
            e = self.graph.get_edge(cs_path[k], cs_path[k + 1])
            if e is None:
                self._feasible = False
                return False
            self._traverse(cs_path[k], cs_path[k + 1], e)
            if not self._feasible:
                return False

        if self._current == cs:
            self._do_charge(dest)
            return True
        return False

    def _do_charge(self, dest: str):
        """Charge at the current CS using Policy B / fallback A."""
        E_target = self._charge_target(dest)
        if E_target <= self._E_current:
            return

        t_c = compute_charging_time(
            E_target, self._E_current,
            self.params["charging_power"],
            self.params["charging_efficiency"],
        )
        added = E_target - self._E_current
        self._E_current = E_target
        self._total_time += t_c
        self._total_energy_charged_static += added
        self._charging_stops.append(
            ChargingStop(self._current, t_c, added)
        )
        # update SoC trail entry to post-charge value
        if self._soc_trail and self._soc_trail[-1][0] == self._current:
            self._soc_trail[-1] = (
                self._current,
                self._E_current / self._E_bat * 100,
            )

    def _charge_target(self, dest: str) -> float:
        """Policy B (minimum-required) with Policy A (full) fallback."""
        try:
            _, path = self.graph.dijkstra(self._current, dest)
            if not path:
                return self._E_bat

            total_energy = 0.0
            for k in range(len(path) - 1):
                e = self.graph.get_edge(path[k], path[k + 1])
                if e is None:
                    return self._E_bat

                is_elec = e["type"] == "electric"
                speed = compute_actual_speed(
                    self.params["base_speed"], e["traffic_factor"],
                    is_elec, self.params["electric_road_speed"],
                )
                mass = self._vehicle_mass + self._load
                E_e = compute_energy_consumption(
                    e["distance"], speed, mass, self.params,
                )
                if is_elec:
                    gain = compute_dwc_energy_gain(
                        e["distance"], speed,
                        self.params["dwc_power"],
                        self.params["dwc_efficiency"],
                    )
                    E_e = max(0.0, E_e - gain)
                total_energy += E_e

                # stop counting at the next CS along the path
                if (path[k + 1] in self.graph.charging_stations
                        and k + 1 < len(path) - 1):
                    break

            safety = self._gamma * self._E_bat
            target = min(self._E_bat, total_energy + safety)
            if target < self._E_current:
                target = min(self._E_bat, self._E_current + safety)
            return target

        except Exception:
            return self._E_bat                    # fallback: full charge

    def _nearest_reachable_cs(self):
        """Return the nearest CS reachable with current battery, or None."""
        best, best_dist = None, float("inf")
        for cs in self.graph.charging_stations:
            if cs == self._current:
                continue
            dist, path = self.graph.dijkstra(self._current, cs)
            if not path:
                continue
            cost = self._path_energy(path)
            if cost <= self._E_current and dist < best_dist:
                best_dist = dist
                best = cs
        return best

    # ── energy estimation helpers ──────────────────────────────

    def _edge_energy(self, edge: dict) -> float:
        is_elec = edge["type"] == "electric"
        speed = compute_actual_speed(
            self.params["base_speed"], edge["traffic_factor"],
            is_elec, self.params["electric_road_speed"],
        )
        mass = self._vehicle_mass + self._load
        return compute_energy_consumption(
            edge["distance"], speed, mass, self.params,
        )

    def _path_energy(self, path: list) -> float:
        """Net energy needed to traverse *path* (consumption − DWC gains)."""
        total = 0.0
        for k in range(len(path) - 1):
            e = self.graph.get_edge(path[k], path[k + 1])
            if e is None:
                return float("inf")
            is_elec = e["type"] == "electric"
            speed = compute_actual_speed(
                self.params["base_speed"], e["traffic_factor"],
                is_elec, self.params["electric_road_speed"],
            )
            mass = self._vehicle_mass + self._load
            E_e = compute_energy_consumption(
                e["distance"], speed, mass, self.params,
            )
            if is_elec:
                gain = compute_dwc_energy_gain(
                    e["distance"], speed,
                    self.params["dwc_power"],
                    self.params["dwc_efficiency"],
                )
                E_e -= gain
            total += E_e
        return total

    # ── result builder ─────────────────────────────────────────

    def _build_result(self) -> RouteResult:
        if self._feasible:
            obj = self._sum_arr - 0.01 * self._sum_bat
        else:
            obj = self._sum_arr - 0.01 * self._sum_bat + PENALTY_BATTERY_DEAD

        E_bat = self._E_bat
        return RouteResult(
            objective_value=obj,
            route_sequence=self._route,
            total_travel_time=self._total_time,
            total_distance=self._total_dist,
            total_energy_consumed=self._total_energy_consumed,
            total_energy_charged_static=self._total_energy_charged_static,
            total_energy_charged_dynamic=self._total_energy_charged_dynamic,
            final_battery_level=self._E_current,
            final_battery_percent=self._E_current / E_bat * 100 if E_bat else 0,
            packages_delivered=self._packages_delivered,
            charging_stops=self._charging_stops,
            feasible=self._feasible,
            soc_trail=self._soc_trail,
            edge_details=self._edge_details,
            sum_arrival_times=self._sum_arr,
            sum_battery_levels=self._sum_bat,
        )
