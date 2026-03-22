"""
Graph representation and Dijkstra shortest-path computation.

* Loads the road network and all parameters from a JSON instance file.
* Normal roads are stored as **bidirectional** edges.
* Electric roads are stored as **one-directional** (start → end only).
"""

import json
import heapq
from config import (
    BASE_SPEED, INITIAL_BATTERY_PERCENT, BATTERY_CAPACITY,
    VEHICLE_MASS, ROLLING_RESISTANCE, DRAG_COEFFICIENT,
    CROSS_SECTIONAL_AREA, MASS_FACTOR, PACKAGE_WEIGHT,
    CHARGING_POWER, CHARGING_EFFICIENCY,
    DWC_POWER, DWC_EFFICIENCY, ELECTRIC_ROAD_SPEED,
    AIR_DENSITY, ROAD_ANGLE_DEG,
)


class Graph:
    """Directed graph with typed nodes and weighted edges."""

    def __init__(self):
        self.nodes: dict = {}          # id → {type, …}
        self.adj: dict   = {}          # id → [(neighbour, edge_data), …]
        self.customers: list          = []
        self.depot: str | None        = None
        self.charging_stations: list  = []
        self.electric_road_starts: list = []
        self.electric_road_ends: list   = []
        self.params: dict = {}

    # ── loading ────────────────────────────────────────────────

    def load_from_json(self, filepath: str) -> None:
        with open(filepath, "r") as fh:
            data = json.load(fh)

        # — nodes —
        for node in data["nodes"]:
            nid = node["id"]
            self.nodes[nid] = node
            self.adj.setdefault(nid, [])

            ntype = node["type"]
            if ntype == "depot":
                self.depot = nid
            elif ntype == "customer":
                self.customers.append(nid)
            elif ntype == "charging_station":
                self.charging_stations.append(nid)
            elif ntype == "electric_road_start":
                self.electric_road_starts.append(nid)
            elif ntype == "electric_road_end":
                self.electric_road_ends.append(nid)

        # — edges —
        for edge in data["edges"]:
            src, dst = edge["from"], edge["to"]
            edge_data = {
                "distance":       edge["distance"],
                "traffic_factor": edge["traffic_factor"],
                "type":           edge["type"],
            }
            if "sumo_edge_id" in edge:
                edge_data["sumo_edge_id"] = edge["sumo_edge_id"]
            self.adj[src].append((dst, edge_data))

            # Normal roads → also add the reverse direction
            if edge["type"] == "normal":
                self.adj[dst].append((src, edge_data))

        # — parameters (JSON overrides compiled defaults) —
        self.params = {
            "base_speed":              data.get("base_speed",              BASE_SPEED),
            "initial_battery_percent": data.get("initial_battery_percent", INITIAL_BATTERY_PERCENT),
            "starting_node":           data.get("starting_node",           "D"),
            "battery_capacity":        data.get("battery_capacity",        BATTERY_CAPACITY),
            "vehicle_mass":            data.get("vehicle_mass",            VEHICLE_MASS),
            "rolling_resistance":      data.get("rolling_resistance",      ROLLING_RESISTANCE),
            "drag_coefficient":        data.get("drag_coefficient",        DRAG_COEFFICIENT),
            "cross_sectional_area":    data.get("cross_sectional_area",    CROSS_SECTIONAL_AREA),
            "mass_factor":             data.get("mass_factor",             MASS_FACTOR),
            "package_weight":          data.get("package_weight",          PACKAGE_WEIGHT),
            "charging_power":          data.get("charging_power",          CHARGING_POWER),
            "charging_efficiency":     data.get("charging_efficiency",     CHARGING_EFFICIENCY),
            "dwc_power":               data.get("dwc_power",              DWC_POWER),
            "dwc_efficiency":          data.get("dwc_efficiency",          DWC_EFFICIENCY),
            "electric_road_speed":     data.get("electric_road_speed",     ELECTRIC_ROAD_SPEED),
            "air_density":             data.get("air_density",             AIR_DENSITY),
            "angle":                   data.get("angle",                   ROAD_ANGLE_DEG),
        }

    def load_from_sumo_data(self, nodes_list: list, edges_list: list, params_dict: dict) -> None:
        """Populate graph from in-memory SUMO-converted structures."""
        self.nodes = {}
        self.adj = {}
        self.customers = []
        self.depot = None
        self.charging_stations = []
        self.electric_road_starts = []
        self.electric_road_ends = []
        self.params = {}

        for node in nodes_list:
            nid = node["id"]
            self.nodes[nid] = node
            self.adj.setdefault(nid, [])
            ntype = node["type"]
            if ntype == "depot":
                self.depot = nid
            elif ntype == "customer":
                self.customers.append(nid)
            elif ntype == "charging_station":
                self.charging_stations.append(nid)
            elif ntype == "electric_road_start":
                self.electric_road_starts.append(nid)
            elif ntype == "electric_road_end":
                self.electric_road_ends.append(nid)

        for edge in edges_list:
            src, dst = edge["from"], edge["to"]
            edge_data = {
                "distance": edge["distance"],
                "traffic_factor": edge["traffic_factor"],
                "type": edge["type"],
            }
            if "sumo_edge_id" in edge:
                edge_data["sumo_edge_id"] = edge["sumo_edge_id"]

            self.adj.setdefault(src, [])
            self.adj.setdefault(dst, [])
            self.adj[src].append((dst, edge_data))
            if edge["type"] == "normal":
                self.adj[dst].append((src, edge_data))

        self.params = {
            "base_speed":              params_dict.get("base_speed",              BASE_SPEED),
            "initial_battery_percent": params_dict.get("initial_battery_percent", INITIAL_BATTERY_PERCENT),
            "starting_node":           params_dict.get("starting_node",           self.depot or "D"),
            "battery_capacity":        params_dict.get("battery_capacity",        BATTERY_CAPACITY),
            "vehicle_mass":            params_dict.get("vehicle_mass",            VEHICLE_MASS),
            "rolling_resistance":      params_dict.get("rolling_resistance",      ROLLING_RESISTANCE),
            "drag_coefficient":        params_dict.get("drag_coefficient",        DRAG_COEFFICIENT),
            "cross_sectional_area":    params_dict.get("cross_sectional_area",    CROSS_SECTIONAL_AREA),
            "mass_factor":             params_dict.get("mass_factor",             MASS_FACTOR),
            "package_weight":          params_dict.get("package_weight",          PACKAGE_WEIGHT),
            "charging_power":          params_dict.get("charging_power",          CHARGING_POWER),
            "charging_efficiency":     params_dict.get("charging_efficiency",     CHARGING_EFFICIENCY),
            "dwc_power":               params_dict.get("dwc_power",              DWC_POWER),
            "dwc_efficiency":          params_dict.get("dwc_efficiency",          DWC_EFFICIENCY),
            "electric_road_speed":     params_dict.get("electric_road_speed",     ELECTRIC_ROAD_SPEED),
            "air_density":             params_dict.get("air_density",             AIR_DENSITY),
            "angle":                   params_dict.get("angle",                   ROAD_ANGLE_DEG),
        }

    def update_traffic_factors(self, traci_bridge, base_speed: float) -> None:
        """Refresh edge traffic factors from SUMO TraCI using sumo_edge_id metadata."""
        if not traci_bridge or not traci_bridge.is_connected():
            return
        for src in self.adj:
            updated = []
            for dst, edge_data in self.adj[src]:
                sumo_edge_id = edge_data.get("sumo_edge_id")
                if sumo_edge_id:
                    tf = traci_bridge.calculate_traffic_factor(sumo_edge_id, base_speed)
                    if tf is not None:
                        edge_data["traffic_factor"] = tf
                updated.append((dst, edge_data))
            self.adj[src] = updated

    # ── shortest path ─────────────────────────────────────────

    def dijkstra(self, source: str, target: str):
        """
        Return ``(distance, path)`` for the shortest path *source → target*.

        *distance* is total km; *path* is the ordered list of node-ids.
        Returns ``(inf, [])`` when unreachable.
        """
        dist = {source: 0}
        prev = {source: None}
        counter = 0
        pq = [(0, counter, source)]

        while pq:
            d, _, u = heapq.heappop(pq)
            if d > dist.get(u, float("inf")):
                continue
            if u == target:
                break
            for v, edata in self.adj.get(u, []):
                nd = d + edata["distance"]
                if nd < dist.get(v, float("inf")):
                    dist[v] = nd
                    prev[v] = u
                    counter += 1
                    heapq.heappush(pq, (nd, counter, v))

        if target not in dist:
            return float("inf"), []

        path, node = [], target
        while node is not None:
            path.append(node)
            node = prev[node]
        path.reverse()
        return dist[target], path

    def dijkstra_all(self, source: str) -> dict:
        """
        Return ``{node: distance}`` from *source* to every reachable node.
        """
        dist = {source: 0}
        counter = 0
        pq = [(0, counter, source)]

        while pq:
            d, _, u = heapq.heappop(pq)
            if d > dist.get(u, float("inf")):
                continue
            for v, edata in self.adj.get(u, []):
                nd = d + edata["distance"]
                if nd < dist.get(v, float("inf")):
                    dist[v] = nd
                    counter += 1
                    heapq.heappush(pq, (nd, counter, v))
        return dist

    # ── edge lookup ────────────────────────────────────────────

    def get_edge(self, from_node: str, to_node: str):
        """Return edge-data dict or ``None``."""
        for nbr, edata in self.adj.get(from_node, []):
            if nbr == to_node:
                return edata
        return None
