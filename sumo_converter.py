"""
SUMO network converter for EVRP-SCS-and-DWC-CW.

Supports:
- Reading SUMO networks (.sumocfg / .net.xml)
- Assigning depot, customers, charging stations, intersections
- Electric-road placement via edge betweenness centrality
- Exporting JSON instances compatible with Graph.load_from_json()
- Building Graph objects directly for SUMO mode in main.py
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import random
import sys
import xml.etree.ElementTree as ET
from urllib.parse import unquote
from typing import Any, Dict, List, Optional, Tuple

import networkx as nx
import numpy as np

try:
    from scipy.cluster.vq import kmeans2
except ImportError:  # pragma: no cover - optional dependency fallback
    kmeans2 = None

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

try:
    import sumolib  # type: ignore
except ImportError as exc:
    raise ImportError(
        "sumolib is required. Install SUMO and set SUMO_HOME."
    ) from exc

logger = logging.getLogger(__name__)


def extract_net_file_from_sumocfg(sumocfg_file: str) -> str:
    if not os.path.exists(sumocfg_file):
        raise FileNotFoundError(f"SUMO config file not found: {sumocfg_file}")

    tree = ET.parse(sumocfg_file)
    root = tree.getroot()
    input_elem = root.find("input")
    if input_elem is None:
        raise ValueError("No <input> section in SUMOCFG")
    net_elem = input_elem.find("net-file")
    if net_elem is None:
        raise ValueError("No <net-file> in SUMOCFG")

    net_file = net_elem.get("value")
    if not net_file:
        raise ValueError("Empty net-file value in SUMOCFG")
    net_file = unquote(net_file)

    cfg_dir = os.path.dirname(os.path.abspath(sumocfg_file))
    if not os.path.isabs(net_file):
        net_file = os.path.join(cfg_dir, net_file)
    net_file = os.path.normpath(net_file)
    return net_file


def parse_sumo_network_with_sumolib(
    net_file: str,
    traffic_min: float = 0.6,
    traffic_max: float = 1.0,
) -> Tuple[Dict[str, Dict[str, Any]], nx.Graph]:
    """Return nodes dict and undirected graph with SUMO edge metadata.

    Traffic factors remap each edge's lane speed to ``[traffic_min, traffic_max]``
    by normalizing speeds across the (largest connected) network.
    """
    if traffic_min > traffic_max:
        raise ValueError(f"traffic_min ({traffic_min}) must be <= traffic_max ({traffic_max})")

    if not os.path.exists(net_file):
        raise FileNotFoundError(f"SUMO network file not found: {net_file}")

    net = sumolib.net.readNet(net_file)
    graph = nx.Graph()
    nodes_dict: Dict[str, Dict[str, Any]] = {}

    for node in net.getNodes():
        node_id = node.getID()
        x_m, y_m = node.getCoord()
        nodes_dict[node_id] = {"id": node_id, "x": x_m / 1000.0, "y": y_m / 1000.0}
        graph.add_node(node_id, x=x_m / 1000.0, y=y_m / 1000.0)

    # Pass 1: edges with raw speed_kmh (traffic_factor filled after LCC + global min/max)
    for edge in net.getEdges():
        u = edge.getFromNode().getID()
        v = edge.getToNode().getID()
        edge_id = edge.getID()
        distance = edge.getLength() / 1000.0
        speed_kmh = edge.getSpeed() * 3.6

        if graph.has_edge(u, v):
            graph[u][v].setdefault("sumo_edge_ids", []).append(edge_id)
        else:
            graph.add_edge(
                u,
                v,
                distance=distance,
                traffic_factor=traffic_min,  # placeholder; set in pass 2
                type="normal",
                speed_kmh=speed_kmh,
                sumo_edge_ids=[edge_id],
            )

    if graph.number_of_nodes() == 0 or graph.number_of_edges() == 0:
        raise ValueError("Parsed empty SUMO graph")

    if not nx.is_connected(graph):
        largest = max(nx.connected_components(graph), key=len)
        graph = graph.subgraph(largest).copy()
        nodes_dict = {nid: data for nid, data in nodes_dict.items() if nid in largest}

    # Pass 2: remap speeds to [traffic_min, traffic_max]
    speeds = [float(data["speed_kmh"]) for _, _, data in graph.edges(data=True)]
    min_s = min(speeds)
    max_s = max(speeds)
    span = traffic_max - traffic_min

    for u, v, data in graph.edges(data=True):
        s = float(data["speed_kmh"])
        if max_s > min_s:
            norm = (s - min_s) / (max_s - min_s)
            tf = traffic_min + norm * span
        else:
            tf = traffic_min
        data["traffic_factor"] = round(float(tf), 2)

    return nodes_dict, graph


def _assign_depot(nodes: Dict[str, Dict[str, Any]]) -> str:
    xs = [n["x"] for n in nodes.values()]
    ys = [n["y"] for n in nodes.values()]
    cx, cy = float(np.mean(xs)), float(np.mean(ys))
    ranked = sorted(
        nodes.keys(),
        key=lambda nid: (nodes[nid]["x"] - cx) ** 2 + (nodes[nid]["y"] - cy) ** 2,
    )
    return ranked[0]


def _assign_customers(
    nodes: Dict[str, Dict[str, Any]], depot_id: str, n_customers: int, seed: Optional[int]
) -> List[str]:
    if seed is not None:
        np.random.seed(seed)
    if n_customers <= 0:
        return []

    candidates = [nid for nid in nodes if nid != depot_id]
    xs = [nodes[n]["x"] for n in candidates]
    ys = [nodes[n]["y"] for n in candidates]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    mid_x, mid_y = (min_x + max_x) / 2.0, (min_y + max_y) / 2.0

    quads = {"ll": [], "lr": [], "ul": [], "ur": []}
    for nid in candidates:
        x, y = nodes[nid]["x"], nodes[nid]["y"]
        if x <= mid_x and y <= mid_y:
            quads["ll"].append(nid)
        elif x > mid_x and y <= mid_y:
            quads["lr"].append(nid)
        elif x <= mid_x and y > mid_y:
            quads["ul"].append(nid)
        else:
            quads["ur"].append(nid)

    selected: List[str] = []
    base = n_customers // 4
    for key in ["ll", "lr", "ul", "ur"]:
        q = quads[key]
        if q:
            take = min(base, len(q))
            if take > 0:
                selected.extend(np.random.choice(q, size=take, replace=False).tolist())

    needed = n_customers - len(selected)
    if needed > 0:
        remaining = [n for n in candidates if n not in set(selected)]
        if remaining:
            selected.extend(
                np.random.choice(remaining, size=min(needed, len(remaining)), replace=False).tolist()
            )

    return selected


def _assign_charging_stations(
    nodes: Dict[str, Dict[str, Any]],
    depot_id: str,
    customers: List[str],
    n_cs: int,
    seed: Optional[int],
) -> List[str]:
    if seed is not None:
        np.random.seed(seed)
    excluded = {depot_id, *customers}
    candidates = [nid for nid in nodes if nid not in excluded]
    if n_cs <= 0 or not candidates:
        return []

    if kmeans2 is None or len(candidates) <= n_cs:
        return random.sample(candidates, k=min(n_cs, len(candidates)))

    points = np.array([[nodes[n]["x"], nodes[n]["y"]] for n in candidates])
    try:
        centroids, labels = kmeans2(points, n_cs, minit="random", seed=42 if seed is None else seed)
    except Exception:
        return random.sample(candidates, k=min(n_cs, len(candidates)))

    chosen: List[str] = []
    used = set()
    for cluster_id in range(n_cs):
        idxs = [i for i, lab in enumerate(labels) if int(lab) == cluster_id]
        if not idxs:
            continue
        cx, cy = centroids[cluster_id]
        best = min(
            idxs,
            key=lambda i: (points[i][0] - cx) ** 2 + (points[i][1] - cy) ** 2,
        )
        nid = candidates[best]
        if nid not in used:
            chosen.append(nid)
            used.add(nid)

    if len(chosen) < n_cs:
        rem = [n for n in candidates if n not in used]
        if rem:
            chosen.extend(random.sample(rem, k=min(n_cs - len(chosen), len(rem))))

    return chosen


def _apply_electric_roads_by_betweenness(graph: nx.Graph, er_fraction: float) -> set[Tuple[str, str]]:
    """Mark top-betweenness undirected edges as electric and return chosen edge set."""
    if er_fraction <= 0:
        return set()
    er_fraction = min(er_fraction, 1.0)
    n_edges = graph.number_of_edges()
    if n_edges == 0:
        return set()
    k = max(1, int(round(er_fraction * n_edges)))

    centrality = nx.edge_betweenness_centrality(graph, normalized=True)
    ranked = sorted(centrality.items(), key=lambda kv: kv[1], reverse=True)
    chosen = {edge for edge, _ in ranked[:k]}

    for u, v in chosen:
        if graph.has_edge(u, v):
            graph[u][v]["type"] = "electric"
            graph[u][v]["traffic_factor"] = 1.0
    return chosen


def _build_instance_dict(
    nodes_dict: Dict[str, Dict[str, Any]],
    graph_nx: nx.Graph,
    depot_id: str,
    customers: List[str],
    charging_stations: List[str],
    params: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    assigned = {depot_id, *customers, *charging_stations}
    nodes_out: List[Dict[str, Any]] = []

    for nid in sorted(nodes_dict.keys()):
        if nid == depot_id:
            ntype = "depot"
        elif nid in customers:
            ntype = "customer"
        elif nid in charging_stations:
            ntype = "charging_station"
        else:
            ntype = "intersection"
        nodes_out.append({"id": nid, "type": ntype})

    edges_out: List[Dict[str, Any]] = []
    for u, v, data in graph_nx.edges(data=True):
        edge_type = data.get("type", "normal")
        distance = float(data["distance"])
        traffic = float(data.get("traffic_factor", 1.0))
        sumo_edge_ids = data.get("sumo_edge_ids", [])
        sumo_edge_id = sumo_edge_ids[0] if sumo_edge_ids else None

        # Electric roads: keep endpoints on original SUMO nodes (matches hand-crafted
        # JSON instances). Do not replace with isolated ER_start/ER_end segments.
        # Unlike `Graph.load_from_json`, electric edges are not auto-reversed; emit
        # both directions so connectivity matches normal bidirectional roads.
        if edge_type == "electric":
            elec = {
                "distance": distance,
                "traffic_factor": 1.0,
                "type": "electric",
                "sumo_edge_id": sumo_edge_id,
            }
            edges_out.append({"from": u, "to": v, **elec})
            edges_out.append({"from": v, "to": u, **elec})
        else:
            edges_out.append(
                {
                    "from": u,
                    "to": v,
                    "distance": distance,
                    "traffic_factor": traffic,
                    "type": "normal",
                    "sumo_edge_id": sumo_edge_id,
                }
            )

    base_params = {
        "base_speed": 50,
        "initial_battery_percent": 100,
        "starting_node": depot_id,
        "battery_capacity": 100,
    }
    if params:
        base_params.update(params)

    return {"nodes": nodes_out, "edges": edges_out, **base_params}


def create_instance_from_sumo(
    net_file_or_cfg: str,
    n_customers: int = 20,
    n_cs: int = 10,
    er_fraction: float = 0.10,
    seed: Optional[int] = None,
    traffic_min: float = 0.6,
    traffic_max: float = 1.0,
    cs_ratio: Optional[float] = None,
) -> Dict[str, Any]:
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    net_file = (
        extract_net_file_from_sumocfg(net_file_or_cfg)
        if net_file_or_cfg.endswith(".sumocfg")
        else net_file_or_cfg
    )
    nodes_dict, graph_nx = parse_sumo_network_with_sumolib(
        net_file, traffic_min=traffic_min, traffic_max=traffic_max
    )

    if cs_ratio is not None:
        n_cs = max(1, round(cs_ratio * len(nodes_dict)))

    depot_id = _assign_depot(nodes_dict)
    customers = _assign_customers(nodes_dict, depot_id, n_customers, seed)
    charging_stations = _assign_charging_stations(nodes_dict, depot_id, customers, n_cs, seed)
    _apply_electric_roads_by_betweenness(graph_nx, er_fraction)

    return _build_instance_dict(
        nodes_dict=nodes_dict,
        graph_nx=graph_nx,
        depot_id=depot_id,
        customers=customers,
        charging_stations=charging_stations,
    )


def export_to_json(instance_dict: Dict[str, Any], output_path: str) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as fh:
        json.dump(instance_dict, fh, indent=2)


def build_graph_from_sumo(
    net_file_or_cfg: str,
    n_customers: int,
    n_cs: int,
    er_fraction: float,
    seed: Optional[int],
    graph_cls,
    traffic_min: float = 0.6,
    traffic_max: float = 1.0,
    cs_ratio: Optional[float] = None,
):
    """Build and return a populated Graph instance using graph_cls()."""
    instance_dict = create_instance_from_sumo(
        net_file_or_cfg=net_file_or_cfg,
        n_customers=n_customers,
        n_cs=n_cs,
        er_fraction=er_fraction,
        seed=seed,
        traffic_min=traffic_min,
        traffic_max=traffic_max,
        cs_ratio=cs_ratio,
    )
    graph = graph_cls()
    graph.load_from_sumo_data(
        nodes_list=instance_dict["nodes"],
        edges_list=instance_dict["edges"],
        params_dict={k: v for k, v in instance_dict.items() if k not in ("nodes", "edges")},
    )
    return graph, instance_dict


def _parse_args():
    parser = argparse.ArgumentParser(description="Convert SUMO network to EVRP-SCS-DWC instance JSON")
    parser.add_argument("net_file_or_cfg", type=str, help="Path to .sumocfg or .net.xml")
    parser.add_argument("--customers", type=int, default=20)
    cs_group = parser.add_mutually_exclusive_group()
    cs_group.add_argument("--cs", type=int, default=10, help="Number of charging stations")
    cs_group.add_argument(
        "--cs-ratio",
        type=float,
        default=None,
        help="Charging stations as fraction of total network nodes (overrides --cs)",
    )
    parser.add_argument("--electric-road-fraction", type=float, default=0.10)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--traffic-min", type=float, default=0.6, help="Min traffic factor after remap")
    parser.add_argument("--traffic-max", type=float, default=1.0, help="Max traffic factor after remap")
    parser.add_argument("--export", type=str, required=True, help="Output JSON instance path")
    return parser.parse_args()


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    args = _parse_args()
    instance_dict = create_instance_from_sumo(
        net_file_or_cfg=args.net_file_or_cfg,
        n_customers=args.customers,
        n_cs=args.cs,
        er_fraction=args.electric_road_fraction,
        seed=args.seed,
        traffic_min=args.traffic_min,
        traffic_max=args.traffic_max,
        cs_ratio=args.cs_ratio,
    )
    export_to_json(instance_dict, args.export)
    print(f"Exported SUMO instance to: {args.export}")


if __name__ == "__main__":
    main()
