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
import time
import random

from graph import Graph
from simulator import Simulator
from savings import clarke_wright, compute_savings_matrix
from local_search import two_opt, or_opt
from logger import print_summary, print_top_savings
from config import CW_DEFAULTS, PARALLEL_WORKERS


def parse_args():
    p = argparse.ArgumentParser(
        description="Clarke & Wright Savings Algorithm for EVRP-SCS-DWC",
    )
    p.add_argument("--instance", type=str, required=True,
                   help="Path to JSON instance file")
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
    return p.parse_args()


def main():
    args = parse_args()
    random.seed(args.seed)

    t_start = time.perf_counter()

    workers = args.workers          # None → auto-detect inside each module

    # ── load instance ──────────────────────────────────────────
    graph = Graph()
    graph.load_from_json(args.instance)
    sim = Simulator(graph)

    print(f"Instance       : {args.instance}")
    print(f"Depot          : {graph.depot}")
    print(f"Customers ({len(graph.customers):>2}) : {graph.customers}")
    print(f"Charging Stn   : {graph.charging_stations}")
    print(f"E-road starts  : {graph.electric_road_starts}")
    print(f"E-road ends    : {graph.electric_road_ends}")

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


if __name__ == "__main__":
    main()
