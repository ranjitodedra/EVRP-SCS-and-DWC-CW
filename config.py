"""
Default configuration and constants for the EV Routing Problem
with Static Charging Stations and Dynamic Wireless Charging.
"""

import math

# ── Physics constants ──────────────────────────────────────────
GRAVITY = 9.8          # m/s²
AIR_DENSITY = 1.205    # kg/m³
ROAD_ANGLE_DEG = 0.86  # degrees
ROAD_ANGLE_RAD = math.radians(ROAD_ANGLE_DEG)

# ── Vehicle defaults (overridden by JSON) ──────────────────────
VEHICLE_MASS = 1800           # kg (kerb, no payload)
ROLLING_RESISTANCE = 0.01
DRAG_COEFFICIENT = 0.6
CROSS_SECTIONAL_AREA = 3.5   # m²
MASS_FACTOR = 1.1             # rolling-inertia mass (kg)
PACKAGE_WEIGHT = 5            # kg per package
BASE_SPEED = 50               # km/h

# ── Battery defaults ───────────────────────────────────────────
BATTERY_CAPACITY = 100        # kWh
INITIAL_BATTERY_PERCENT = 100 # %
BATTERY_THRESHOLD = 0.20      # γ  (mileage-anxiety coefficient)

# ── Static Charging Station defaults ───────────────────────────
CHARGING_POWER = 100          # kW
CHARGING_EFFICIENCY = 0.95    # ε

# ── Dynamic Wireless Charging (DWC) defaults ───────────────────
DWC_POWER = 20                # kW
DWC_EFFICIENCY = 0.85         # η_chg
ELECTRIC_ROAD_SPEED = 50      # km/h  (constant, traffic-free)

# ── Clarke & Wright algorithm defaults ─────────────────────────
CW_DEFAULTS = {
    "savings_metric": "distance",       # "distance", "time", or "objective"
    "local_search": "2opt",             # "2opt", "oropt", "both", or "none"
    "max_ls_iterations": 1000,
    "ls_improvement_threshold": 1e-6,
    "random_seed": 42,
}

# ── Penalty constants ──────────────────────────────────────────
PENALTY_BATTERY_DEAD = 99999
PENALTY_UNREACHABLE = 99999

# ── Parallelisation ───────────────────────────────────────────
PARALLEL_WORKERS = None          # None → auto-detect via os.cpu_count()
