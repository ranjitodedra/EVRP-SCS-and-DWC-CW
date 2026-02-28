"""
Post-optimisation local-search improvements.

* **2-opt** — reverse a sub-segment of the customer visit order.
* **Or-opt** — relocate 1, 2, or 3 consecutive customers to another
  position in the tour.

Both use a *first-improvement* strategy and stop after
*max_iterations* passes without improvement or when the gain drops
below *threshold*.
"""

from __future__ import annotations

from typing import Tuple, List

from simulator import Simulator, RouteResult


# ── 2-opt ──────────────────────────────────────────────────────

def two_opt(customer_order: list,
            simulator: Simulator,
            max_iterations: int = 1000,
            threshold: float = 1e-6) -> Tuple[list, RouteResult]:
    """
    Improve *customer_order* by 2-opt reversal moves.

    Returns ``(improved_order, route_result)``.
    """
    best = list(customer_order)
    best_res = simulator.simulate(best)
    best_cost = best_res.objective_value
    n = len(best)

    if n < 2:
        return best, best_res

    for _ in range(max_iterations):
        improved = False
        for i in range(n - 1):
            for j in range(i + 2, n):
                candidate = best[:i] + list(reversed(best[i:j + 1])) + best[j + 1:]
                res = simulator.simulate(candidate)
                if res.feasible and res.objective_value < best_cost - threshold:
                    best = candidate
                    best_res = res
                    best_cost = res.objective_value
                    improved = True
                    break                          # first-improvement
            if improved:
                break
        if not improved:
            break

    return best, best_res


# ── Or-opt ─────────────────────────────────────────────────────

def or_opt(customer_order: list,
           simulator: Simulator,
           max_iterations: int = 1000,
           threshold: float = 1e-6) -> Tuple[list, RouteResult]:
    """
    Improve *customer_order* by relocating chains of 1–3 customers.

    Returns ``(improved_order, route_result)``.
    """
    best = list(customer_order)
    best_res = simulator.simulate(best)
    best_cost = best_res.objective_value
    n = len(best)

    if n < 2:
        return best, best_res

    for _ in range(max_iterations):
        improved = False
        for seg_len in (1, 2, 3):
            if improved:
                break
            for i in range(n - seg_len + 1):
                if improved:
                    break
                segment   = best[i:i + seg_len]
                remaining = best[:i] + best[i + seg_len:]
                for j in range(len(remaining) + 1):
                    candidate = remaining[:j] + segment + remaining[j:]
                    if candidate == best:
                        continue
                    res = simulator.simulate(candidate)
                    if res.feasible and res.objective_value < best_cost - threshold:
                        best = candidate
                        best_res = res
                        best_cost = res.objective_value
                        n = len(best)
                        improved = True
                        break
        if not improved:
            break

    return best, best_res
