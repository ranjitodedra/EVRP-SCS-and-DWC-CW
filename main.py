"""
Entry point — Clarke & Wright Savings Algorithm for the
Electric-Vehicle Routing Problem with Static Charging Stations
and Dynamic Wireless Charging.

Usage
─────
  python main.py --instance instances/instance_1.json
  python main.py --instance instances/instance_3.json --local-search both
  python main.py --instance instances/instance_3.json --workers 4
"""

from __future__ import annotations

import argparse
import contextlib
import os
import sys
import time
import random

from graph import Graph
from simulator import Simulator
from savings import clarke_wright, compute_savings_matrix
from local_search import two_opt, or_opt
from logger import print_network_statistics, print_summary, print_top_savings
from config import CW_DEFAULTS, PARALLEL_WORKERS


class _TeeStream:
    """Mirror writes to multiple text streams."""

    def __init__(self, *streams):
        self._streams = streams

    def write(self, data):
        for stream in self._streams:
            stream.write(data)
        return len(data)

    def flush(self):
        for stream in self._streams:
            stream.flush()


def parse_args():
    p = argparse.ArgumentParser(
        description="Clarke & Wright Savings Algorithm for EVRP-SCS-DWC",
    )
    p.add_argument("--instance", type=str, default=None,
                   help="Path to JSON instance file")
    p.add_argument("--sumo-cfg", type=str, default=None,
                   help="Path to SUMO .sumocfg or .net.xml file")
    p.add_argument("--customers", type=int, default=20,
                   help="Number of customers to assign in SUMO mode")
    cs_group = p.add_mutually_exclusive_group()
    cs_group.add_argument("--cs", type=int, default=10,
                          help="Number of charging stations to assign in SUMO mode")
    cs_group.add_argument("--cs-ratio", type=float, default=None,
                          help="Charging stations as fraction of total SUMO network nodes (overrides --cs)")
    p.add_argument("--electric-road-fraction", type=float, default=0.10,
                   help="Fraction of edges to mark as electric roads in SUMO mode")
    p.add_argument("--traffic-min", type=float, default=0.6,
                   help="Min traffic factor after SUMO speed remap (static mode)")
    p.add_argument("--traffic-max", type=float, default=1.0,
                   help="Max traffic factor after SUMO speed remap (static mode)")
    p.add_argument("--realtime-traffic", action="store_true",
                   help="Enable real-time traffic factors from SUMO TraCI")
    p.add_argument("--gui", action="store_true",
                   help="Start SUMO GUI (only valid with --realtime-traffic)")
    p.add_argument("--sumo-seed", type=int, default=None,
                   help="Random seed for SUMO node role assignment")
    p.add_argument("--savings-metric", type=str,
                   default=CW_DEFAULTS["savings_metric"],
                   choices=["distance", "time", "objective"],
                   help="Metric for savings computation")
    p.add_argument("--local-search", type=str,
                   default=CW_DEFAULTS["local_search"],
                   choices=["2opt", "oropt", "both", "none"],
                   help="Post-optimisation strategy")
    p.add_argument("--max-ls-iterations", type=int,
                   default=CW_DEFAULTS["max_ls_iterations"])
    p.add_argument("--ls-threshold", type=float,
                   default=CW_DEFAULTS["ls_improvement_threshold"])
    p.add_argument("--seed", type=int,
                   default=CW_DEFAULTS["random_seed"])
    p.add_argument("--show-savings", action="store_true",
                   help="Print top-10 savings pairs")
    p.add_argument("--workers", type=int, default=PARALLEL_WORKERS,
                   help="Number of parallel workers (default: auto-detect CPU count)")
    p.add_argument("--output-txt", type=str, default=None,
                   help="Save console output to a text file while still printing to the terminal")
    args = p.parse_args()
    if bool(args.instance) == bool(args.sumo_cfg):
        p.error("Provide exactly one of --instance or --sumo-cfg.")
    if args.gui and not args.realtime_traffic:
        p.error("--gui requires --realtime-traffic.")
    if args.traffic_min > args.traffic_max:
        p.error("--traffic-min must be <= --traffic-max.")
    return args


def _run(args):
    random.seed(args.seed)

    t_start = time.perf_counter()

    workers = args.workers          # None → auto-detect inside each module

    # ── load graph (JSON or SUMO) ─────────────────────────────
    traci_bridge = None
    try:
        if args.sumo_cfg:
            from sumo_converter import build_graph_from_sumo

            graph, _ = build_graph_from_sumo(
                net_file_or_cfg=args.sumo_cfg,
                n_customers=args.customers,
                n_cs=args.cs,
                er_fraction=args.electric_road_fraction,
                seed=args.sumo_seed,
                graph_cls=Graph,
                traffic_min=args.traffic_min,
                traffic_max=args.traffic_max,
                cs_ratio=args.cs_ratio,
            )
            if args.realtime_traffic:
                from sumo_traci_bridge import SUMOTraCIBridge
                traci_bridge = SUMOTraCIBridge(args.sumo_cfg, gui=args.gui)
                graph.update_traffic_factors(traci_bridge, graph.params["base_speed"])
        else:
            graph = Graph()
            graph.load_from_json(args.instance)

        sim = Simulator(graph)

        print(f"Instance       : {args.instance if args.instance else args.sumo_cfg}")
        print(f"Depot          : {graph.depot}")
        print(f"Customers ({len(graph.customers):>2}) : {graph.customers}")
        print(f"Charging Stn   : {graph.charging_stations}")
        print(f"E-road starts  : {graph.electric_road_starts}")
        print(f"E-road ends    : {graph.electric_road_ends}")
        print()
        print_network_statistics(graph)

        # ── savings computation ────────────────────────────────────
        t_sav_start = time.perf_counter()
        savings_list = compute_savings_matrix(graph, args.savings_metric,
                                             workers=workers)
        customer_order, cw_result = clarke_wright(
            graph, sim, args.savings_metric, workers=workers,
        )
        t_sav = time.perf_counter() - t_sav_start

        if args.show_savings:
            print_top_savings(savings_list)

        print(f"\nC&W customer order: {customer_order}")
        print(f"C&W objective     : {cw_result.objective_value:.6f}")

        # ── local search ───────────────────────────────────────────
        t_ls_start = time.perf_counter()
        best_order, best_result = list(customer_order), cw_result

        ls_mode = args.local_search
        max_it  = args.max_ls_iterations
        thresh  = args.ls_threshold

        if ls_mode in ("2opt", "both"):
            best_order, best_result = two_opt(
                best_order, sim, max_it, thresh, workers=workers,
            )
        if ls_mode in ("oropt", "both"):
            best_order, best_result = or_opt(
                best_order, sim, max_it, thresh, workers=workers,
            )

        t_ls = time.perf_counter() - t_ls_start
        t_total = time.perf_counter() - t_start

        if ls_mode != "none":
            print(f"LS customer order : {best_order}")
            print(f"LS objective      : {best_result.objective_value:.6f}")

        # ── final report ───────────────────────────────────────────
        print_summary(best_result, t_sav, t_ls, t_total)
    finally:
        if traci_bridge:
            traci_bridge.close()


def main():
    args = parse_args()
    if not args.output_txt:
        _run(args)
        return

    output_path = os.path.abspath(args.output_txt)
    output_dir = os.path.dirname(output_path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    with open(output_path, "w", encoding="utf-8", buffering=1) as output_file:
        tee = _TeeStream(sys.stdout, output_file)
        err_tee = _TeeStream(sys.stderr, output_file)
        with contextlib.redirect_stdout(tee), contextlib.redirect_stderr(err_tee):
            print(f"Saving console output to: {output_path}")
            _run(args)


if __name__ == "__main__":
    main()
