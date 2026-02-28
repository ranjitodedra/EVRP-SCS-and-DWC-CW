"""
Clarke & Wright Savings Algorithm.

1. Compute shortest-path distances between depot ↔ customers and
   between all customer pairs (Dijkstra).
2. Build a savings matrix:  s(Li, Lj) = d(D,Li) + d(Lj,D) − d(Li,Lj).
3. Sort savings in descending order.
4. Iteratively merge routes while feasibility holds.
"""

from __future__ import annotations

from typing import List, Tuple

from simulator import Simulator, RouteResult


# ── savings computation ────────────────────────────────────────

def compute_savings_matrix(graph, metric: str = "distance"):
    """
    Return a sorted list of ``(saving, ci, cj)`` tuples (descending).

    *metric* may be ``"distance"`` (default), ``"time"``, or
    ``"objective"``.  Only ``"distance"`` is fully implemented here;
    the other two are placeholders that fall back to distance.
    """
    depot = graph.depot
    customers = list(graph.customers)

    # one Dijkstra per relevant source → {target: dist}
    dist_from: dict = {}
    for node in [depot] + customers:
        dist_from[node] = graph.dijkstra_all(node)

    savings: List[Tuple[float, str, str]] = []
    for ci in customers:
        for cj in customers:
            if ci == cj:
                continue

            d_D_ci  = dist_from[depot].get(ci, float("inf"))
            d_cj_D  = dist_from[cj].get(depot, float("inf"))
            d_ci_cj = dist_from[ci].get(cj, float("inf"))

            if float("inf") in (d_D_ci, d_cj_D, d_ci_cj):
                continue

            if metric == "time":
                # approximate: time ≈ dist / base_speed
                bs = graph.params.get("base_speed", 50)
                s = d_D_ci / bs + d_cj_D / bs - d_ci_cj / bs
            else:                                   # "distance" or fallback
                s = d_D_ci + d_cj_D - d_ci_cj

            savings.append((s, ci, cj))

    savings.sort(key=lambda x: -x[0])
    return savings


# ── route merging ──────────────────────────────────────────────

def clarke_wright(graph, simulator: Simulator,
                  metric: str = "distance") -> Tuple[list, RouteResult]:
    """
    Run the Clarke & Wright Savings heuristic.

    Returns ``(customer_order, route_result)``.
    """
    depot = graph.depot
    customers = list(graph.customers)
    n = len(customers)

    # trivial cases
    if n == 0:
        return [], simulator.simulate([])
    if n == 1:
        result = simulator.simulate(customers)
        return list(customers), result

    # Step 1 — savings
    savings = compute_savings_matrix(graph, metric)

    # Step 2 — one shuttle route per customer:  D → Li → D
    route_id = 0
    routes: dict = {}             # rid → [customer list]
    cust_route: dict = {}         # customer → rid

    for c in customers:
        routes[route_id] = [c]
        cust_route[c] = route_id
        route_id += 1

    # Step 3 — merge in savings order
    for s_val, ci, cj in savings:
        ri = cust_route[ci]
        rj = cust_route[cj]

        if ri == rj:
            continue                               # same route already

        if routes[ri][-1] != ci:
            continue                               # ci not at END of its route

        if routes[rj][0] != cj:
            continue                               # cj not at START of its route

        # feasibility check — simulate the merged order
        merged = routes[ri] + routes[rj]
        result = simulator.simulate(merged)
        if not result.feasible:
            continue

        # merge
        routes[ri] = merged
        for c in routes[rj]:
            cust_route[c] = ri
        del routes[rj]

    # Step 4 — collect remaining (possibly >1) routes
    remaining = [r for r in routes.values() if r]

    if len(remaining) == 1:
        final_order = remaining[0]
    else:
        # greedy nearest-insertion for any leftover routes
        dist_from_depot = graph.dijkstra_all(depot)
        dist_maps = {depot: dist_from_depot}
        for c in customers:
            dist_maps[c] = graph.dijkstra_all(c)

        final_order = list(remaining[0])
        used = {id(remaining[0])}
        for r in remaining[1:]:
            # find best insertion point
            best_pos, best_cost = 0, float("inf")
            for pos in range(len(final_order) + 1):
                prev = depot if pos == 0 else final_order[pos - 1]
                nxt  = depot if pos == len(final_order) else final_order[pos]
                cost_in  = dist_maps.get(prev, {}).get(r[0], float("inf"))
                cost_out = dist_maps.get(r[-1], {}).get(nxt, float("inf"))
                cost = cost_in + cost_out
                if cost < best_cost:
                    best_cost = cost
                    best_pos = pos
            for idx, c in enumerate(r):
                final_order.insert(best_pos + idx, c)

    best_result = simulator.simulate(final_order)
    return final_order, best_result
