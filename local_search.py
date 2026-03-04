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

import os
import warnings
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Tuple, List

from simulator import Simulator, RouteResult


# ── thread-safe simulation helper ──────────────────────────────

def _simulate_candidate(graph, candidate: list) -> RouteResult:
    """Create a fresh Simulator and evaluate *candidate*."""
    sim = Simulator(graph)
    return sim.simulate(candidate)


# ── 2-opt ──────────────────────────────────────────────────────

def two_opt(customer_order: list,
            simulator: Simulator,
            max_iterations: int = 1000,
            threshold: float = 1e-6, *,
            workers: int | None = None) -> Tuple[list, RouteResult]:
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

    n_workers = workers if workers is not None else os.cpu_count()
    use_parallel = (n_workers is not None and n_workers > 1 and n >= 4)

    for _ in range(max_iterations):
        improved = False

        if use_parallel:
            try:
                improved = _two_opt_parallel(
                    best, best_cost, threshold, simulator.graph,
                    n_workers, best, best_res,
                )
                if isinstance(improved, tuple):
                    best, best_res, best_cost = improved
                    improved = True
                else:
                    improved = False
            except Exception as exc:
                warnings.warn(f"Parallel 2-opt failed ({exc}); "
                              f"falling back to sequential for this iteration.")
                use_parallel = False          # disable for remaining iterations
                improved = _two_opt_sequential(
                    best, best_cost, threshold, simulator,
                    best, best_res,
                )
                if isinstance(improved, tuple):
                    best, best_res, best_cost = improved
                    improved = True
                else:
                    improved = False
        else:
            improved = _two_opt_sequential(
                best, best_cost, threshold, simulator,
                best, best_res,
            )
            if isinstance(improved, tuple):
                best, best_res, best_cost = improved
                improved = True
            else:
                improved = False

        if not improved:
            break

    return best, best_res


def _two_opt_sequential(best, best_cost, threshold, simulator,
                        _best_ref, _best_res_ref):
    """Original sequential first-improvement 2-opt pass."""
    n = len(best)
    for i in range(n - 1):
        for j in range(i + 2, n):
            candidate = best[:i] + list(reversed(best[i:j + 1])) + best[j + 1:]
            res = simulator.simulate(candidate)
            if res.feasible and res.objective_value < best_cost - threshold:
                return (candidate, res, res.objective_value)
    return False


def _two_opt_parallel(best, best_cost, threshold, graph, n_workers,
                      _best_ref, _best_res_ref):
    """
    Parallel first-improvement 2-opt pass.

    For each outer index *i*, evaluates all *j* candidates in parallel.
    Returns the first improvement (smallest *j*) to preserve the
    canonical first-improvement order.
    """
    n = len(best)
    with ThreadPoolExecutor(max_workers=n_workers) as pool:
        for i in range(n - 1):
            # submit all j candidates for this i
            futures = {}
            for j in range(i + 2, n):
                candidate = best[:i] + list(reversed(best[i:j + 1])) + best[j + 1:]
                fut = pool.submit(_simulate_candidate, graph, candidate)
                futures[fut] = (j, candidate)

            # collect results, find first improvement by j-order
            results = []
            for fut in futures:
                res = fut.result()
                j, cand = futures[fut]
                if res.feasible and res.objective_value < best_cost - threshold:
                    results.append((j, cand, res))

            if results:
                # pick the smallest j (preserves first-improvement order)
                results.sort(key=lambda x: x[0])
                _, cand, res = results[0]
                return (cand, res, res.objective_value)

    return False


# ── Or-opt ─────────────────────────────────────────────────────

def or_opt(customer_order: list,
           simulator: Simulator,
           max_iterations: int = 1000,
           threshold: float = 1e-6, *,
           workers: int | None = None) -> Tuple[list, RouteResult]:
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

    n_workers = workers if workers is not None else os.cpu_count()
    use_parallel = (n_workers is not None and n_workers > 1 and n >= 4)

    for _ in range(max_iterations):
        improved = False

        if use_parallel:
            try:
                result = _or_opt_parallel(
                    best, best_cost, threshold, simulator.graph, n_workers,
                )
                if isinstance(result, tuple):
                    best, best_res, best_cost = result
                    n = len(best)
                    improved = True
                else:
                    improved = False
            except Exception as exc:
                warnings.warn(f"Parallel or-opt failed ({exc}); "
                              f"falling back to sequential for this iteration.")
                use_parallel = False
                result = _or_opt_sequential(
                    best, best_cost, threshold, simulator,
                )
                if isinstance(result, tuple):
                    best, best_res, best_cost = result
                    n = len(best)
                    improved = True
                else:
                    improved = False
        else:
            result = _or_opt_sequential(
                best, best_cost, threshold, simulator,
            )
            if isinstance(result, tuple):
                best, best_res, best_cost = result
                n = len(best)
                improved = True
            else:
                improved = False

        if not improved:
            break

    return best, best_res


def _or_opt_sequential(best, best_cost, threshold, simulator):
    """Original sequential first-improvement or-opt pass."""
    n = len(best)
    for seg_len in (1, 2, 3):
        for i in range(n - seg_len + 1):
            segment   = best[i:i + seg_len]
            remaining = best[:i] + best[i + seg_len:]
            for j in range(len(remaining) + 1):
                candidate = remaining[:j] + segment + remaining[j:]
                if candidate == best:
                    continue
                res = simulator.simulate(candidate)
                if res.feasible and res.objective_value < best_cost - threshold:
                    return (candidate, res, res.objective_value)
    return False


def _or_opt_parallel(best, best_cost, threshold, graph, n_workers):
    """
    Parallel first-improvement or-opt pass.

    For each ``(seg_len, i)`` pair, evaluates all insertion positions *j*
    in parallel.  Returns the first improvement (smallest *j*) to
    preserve the canonical first-improvement order.
    """
    n = len(best)
    with ThreadPoolExecutor(max_workers=n_workers) as pool:
        for seg_len in (1, 2, 3):
            for i in range(n - seg_len + 1):
                segment   = best[i:i + seg_len]
                remaining = best[:i] + best[i + seg_len:]

                futures = {}
                for j in range(len(remaining) + 1):
                    candidate = remaining[:j] + segment + remaining[j:]
                    if candidate == best:
                        continue
                    fut = pool.submit(_simulate_candidate, graph, candidate)
                    futures[fut] = (j, candidate)

                # no candidates submitted (all equal to best)
                if not futures:
                    continue

                results = []
                for fut in futures:
                    res = fut.result()
                    j, cand = futures[fut]
                    if res.feasible and res.objective_value < best_cost - threshold:
                        results.append((j, cand, res))

                if results:
                    results.sort(key=lambda x: x[0])
                    _, cand, res = results[0]
                    return (cand, res, res.objective_value)

    return False
