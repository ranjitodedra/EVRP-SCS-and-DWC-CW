"""
Energy consumption, travel-time, and charging calculations.

Key equations
─────────────
Energy (Eq 7-style):
  E = (1/3600) * [M·g·(f·cos α + sin α) + 0.0386·ρ·Cx·A·v² + (M+m)·dv/dt] * d
  with d in km → result in kWh

DWC gain (Eq 8):
  E_DWC = P_chg · η_chg · L_DWC / v

Static charging time (Eq 4.4):
  t_c = (E_ex − E_re) / (P · ε)
"""

import math


# ── helpers ────────────────────────────────────────────────────

def compute_actual_speed(base_speed: float,
                         traffic_factor: float,
                         is_electric: bool,
                         electric_road_speed: float) -> float:
    """Return actual speed (km/h) on an edge."""
    if is_electric:
        return electric_road_speed          # constant, traffic-free
    return base_speed * traffic_factor


def compute_dv_dt(v0_kmh: float) -> float:
    """Average acceleration (m/s²) based on speed *in km/h*."""
    if 50 <= v0_kmh <= 80:
        return 0.3
    elif 81 <= v0_kmh <= 120:
        return 2.0
    return 0.0


def compute_travel_time(distance_km: float,
                        actual_speed_kmh: float) -> float:
    """Travel time (hours) for one edge."""
    if actual_speed_kmh <= 0:
        return float("inf")
    return distance_km / actual_speed_kmh


# ── energy consumption ─────────────────────────────────────────

def compute_energy_consumption(distance_km: float,
                               actual_speed_kmh: float,
                               total_mass_kg: float,
                               params: dict) -> float:
    """
    Energy consumed traversing one edge (kWh).

    E = (1/3600) · F · d   (d in km → kWh)
    F = M·g·(f·cos α + sin α) + 0.0386·ρ·Cx·A·v² + (M+m)·(dv/dt)
    v must be converted to m/s for the aero term.
    """
    g   = 9.8
    f   = params["rolling_resistance"]
    rho = params["air_density"]
    Cx  = params["drag_coefficient"]
    A   = params["cross_sectional_area"]
    m   = params["mass_factor"]
    alpha = math.radians(params["angle"])

    M      = total_mass_kg
    v_ms   = actual_speed_kmh / 3.6            # m/s
    dv_dt  = compute_dv_dt(actual_speed_kmh)
    d      = distance_km                       # km

    rolling_grade = M * g * (f * math.cos(alpha) + math.sin(alpha))
    aero_drag     = 0.0386 * rho * Cx * A * (v_ms ** 2)
    inertia       = (M + m) * dv_dt

    energy = (1.0 / 3600.0) * (rolling_grade + aero_drag + inertia) * d
    return max(energy, 0.0)


# ── DWC energy gain ────────────────────────────────────────────

def compute_dwc_energy_gain(distance_km: float,
                            speed_kmh: float,
                            dwc_power: float,
                            dwc_efficiency: float) -> float:
    """
    Energy gained from Dynamic Wireless Charging (kWh).

    E_DWC = P_chg · η_chg · (L / v)
    L in km, v in km/h → t in hours → E in kWh.
    """
    if speed_kmh <= 0:
        return 0.0
    t_on = distance_km / speed_kmh             # hours
    return dwc_power * dwc_efficiency * t_on   # kWh


# ── static charging ────────────────────────────────────────────

def compute_charging_time(E_target: float,
                          E_current: float,
                          charging_power: float,
                          charging_efficiency: float) -> float:
    """
    Charging time (hours) at a static station.

    t_c = (E_target − E_current) / (P · ε)
    """
    if E_target <= E_current:
        return 0.0
    R_r = charging_power * charging_efficiency   # effective kW
    return (E_target - E_current) / R_r
