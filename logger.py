"""
Formatted console output and optional CSV / savings-matrix printing.
"""

from __future__ import annotations

from simulator import RouteResult


# ── main summary ───────────────────────────────────────────────

def print_summary(result: RouteResult,
                  savings_time: float = 0.0,
                  ls_time: float = 0.0,
                  total_runtime: float = 0.0) -> None:

    sep = "=" * 60

    print(f"\n{sep}")
    print("  CLARKE & WRIGHT SAVINGS ALGORITHM -- SOLUTION SUMMARY")
    print(sep)

    print(f"  Objective Value (legacy):         {result.objective_value:.6f}")
    print(f"  Total Travel Time:          "
          f"{result.total_travel_time:.4f} hours "
          f"({result.total_travel_time * 60:.2f} min)")
    print(f"  Total Distance:             {result.total_distance:.4f} km")
    print(f"  Total Energy Depletion:     {result.total_energy_consumed:.4f} kWh")
    print(f"  Total Energy Charged (SC):  {result.total_energy_charged_static:.4f} kWh")
    print(f"  Total Energy Charged (DWC): {result.total_energy_charged_dynamic:.4f} kWh")
    print(f"  Final Battery Level:        "
          f"{result.final_battery_level:.4f} kWh "
          f"({result.final_battery_percent:.1f}%)")
    print(f"  Boxes Delivered:            {result.packages_delivered}")

    # charging stops
    if result.charging_stops:
        print("  Charging Stops:")
        for cs in result.charging_stops:
            print(f"    - {cs.station_id}: "
                  f"{cs.time_charged * 60:.2f} min, "
                  f"{cs.energy_added:.4f} kWh")
    else:
        print("  Charging Stops:             None")

    print()

    # ── Route sequence ─────────────────────────────────────────
    print(f"Route Sequence: {' -> '.join(result.route_sequence)}")
    print()

    # ── SoC trail ──────────────────────────────────────────────
    soc_parts = [f"{n}({s:.1f}%)" for n, s in result.soc_trail]
    print(f"SoC Trail: {' -> '.join(soc_parts)}")
    print()

    # ── Energy consumption trail ───────────────────────────────
    if result.edge_details:
        parts = [f"{frm} - {e:.1f} kWh"
                 for frm, _to, e, _t, _d in result.edge_details]
        last = result.route_sequence[-1]
        print(f"Energy Consumption Trail: {' > '.join(parts)} > {last}")
        print()

    # ── Travel time trail ──────────────────────────────────────
    if result.edge_details:
        parts = [f"{frm} - {t:.2f} min"
                 for frm, _to, _e, t, _d in result.edge_details]
        last = result.route_sequence[-1]
        print(f"Travel Time Trail: {' > '.join(parts)} > {last}")
        print()

    # ── timing ─────────────────────────────────────────────────
    print(f"  Savings computation time:    {savings_time:.2f} seconds")
    print(f"  Local search time:           {ls_time:.2f} seconds")
    print(f"  Total Runtime:               {total_runtime:.2f} seconds")
    print(sep)


# ── optional: top-N savings pairs ──────────────────────────────

def print_top_savings(savings_list, n: int = 10) -> None:
    """Print the top-*n* savings pairs for debugging."""
    print(f"\nTop-{n} savings pairs:")
    print(f"  {'Rank':<6}{'Pair':<20}{'Saving':>10}")
    print("  " + "-" * 36)
    for idx, (s, ci, cj) in enumerate(savings_list[:n]):
        print(f"  {idx + 1:<6}{ci + ' -> ' + cj:<20}{s:>10.4f}")
    print()
