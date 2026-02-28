This project introduces Single-Electric-Vehicle Routing Problem. The vehicle navigates a dynamic urban network, deciding routes and charging stops based on real-time traffic and the vehicle's changing payload weight. Energy consumption depends on how loaded the truck is, and it must choose between charging at dedicated static charging stations or using dynamic wireless electric roads. In short, you're fine-tuning the route, timing, and energy decisions for one vehicle to minimize delivery time.

- The EV has limited battery capacity
- Fixed locations where the vehicle can recharge
- Special road segments that provide dynamic (while-driving) charging
- The vehicle must visit specific customer locations to drop boxes
- Can only move to nodes that are adjacent to the current node

Optimization Objective: Minimize: Total travel time - (Sum of all battery levels / 100)

Primary goal: Minimize total travel time
Secondary goal: Maximize remaining battery (battery levels are slightly favored)
The division by 100 makes battery a minor factor compared to time

Environment is made of roads(edges) and locations(nodes). There are 4 types of node: Depot, Intersections, Charging Station, Customers. Now, we introduce 'electric road boundary' nodes—special markers at the start and end of electric road segments. These act as key checkpoints: the vehicle can gain in-motion energy efficiency between them. Thus, when a route is planned, crossing these boundary nodes signifies entering a zone with different energy rules. Instruct the system that these boundary nodes should be treated as distinct decision points—if the vehicle enters them, it changes energy modeling logic for that segment. In short, you're giving them a special category that modifies energy rules but still fits into the existing node framework. There are 2 types of edges: normal roads and electric roads(that allow vehicle to charge while its driving).

- Program will always have one depot, represented by label D.
- Program can also have 1 to n number of customers, labeled like L1, L2, L3,...Ln.
- Program can also have 1 to n number of Charging stations, labeled as CS1, CS2,....CSn.
- Program can also have Intersections, labeled like 1, 2, 3,...n.
- Electric road boundary nodes: Ns1, Ne1, Ns2, Ne2, …
  Where:
    - Nsx = start of electric road segment x
    - Nex = end of electric road segment x

- Every time EV starts from depot the starting charge is 100% (fully charged at depot)

For each edge we have:
- Distance of edge
- Traffic factor of an edge
- Type: i. normal or ii. electric

We will have a variable called base_speed. That will be considered as base speed at each edge.

5. Summary inputs to consider
    5.1. nodes (Depot, Intersections, Charging Station, Customers, special nodes that shows the starting and ending of electric road like Ns1 to Ne1)
    5.2. edges (including distance and traffic factor for each edge, and type of road: normal or electric)
    5.3. base speed (km)
    5.4. percentage battery level at which we start our journey from.
    5.5. starting node (From which node are we starting our journey, By default its depot)
    5.6. Capacity of battery
    M   = total vehicle mass (kg)
    f   = rolling resistance coefficient (unitless)
    Cx  = aerodynamic drag coefficient (unitless)
    A   = vehicle cross-sectional area (m²)
    m = mass_factor

- Program should take input from a json file. So all program input will be given in a json file. For each example instance we will need a different json file.

note that Charging Station = Static Charging Station = Parked / Plug-in / Static Linear Charging

6. Program
- Overall goal of program is to start journey from current node to deliver packages to all the customers and return to depot, while minimizing arrival time.

Objective: The objective gives higher priority to reducing arrival time and a smaller priority to keeping more battery.

```
cost(R) = sum_arrival_times(R) - 0.01 * total_remaining_battery(R)
```

Where:
- sum_arrival_times(R) : sum of arrival times at each node in the decoded route.
- total_remaining_battery(R) : sum of battery levels measured at arrival to each node.

- Speed of vehicle on each electric road is constant and same for all electric roads. Additionally, traffic will have no impact on them as well.
- In this program each customer will have exactly 1 package.
- Weight of each package is 5kg. This is the package that we will deliver to customer; know that each package will have same weight.

- Do not create two directed edges for normal roads.
- Electric roads are directional (start→end only).

- Calculate the Total_load that vehicle is carrying, by number_of_package * Weight_of_packages.
- Now calculate the actual_speed and Travel_time as mentioned below.
- actual_speed (v0) km/h = base_speed * traffic_factor(for that edge)
- Travel_time(of an edge) = distance (of that edge in km) / actual_speed (at an edge in km/h)
- compute it for every edge on the path and sum the results to get the total energy for the whole path
- Total_travel_time = sum(Travel_time_of_edge)

7. Now we calculate the energy, based on this equation
    -  energy_consumption = E = (1/3600) * [ M * g * ( f * cos(alpha) + sin(alpha) ) + 0.0386 * ( rho * Cx * A * v^2 ) + (M + m) * (dv/dt) ] * d

    - meaning of symbols

        M   = total vehicle mass including payload (kg)
        g   = gravity (9.81 m/s²)
        f   = rolling resistance coefficient (unitless)
        rho   = air density (kg/m³)
        Cx  = aerodynamic drag coefficient (unitless)
        A   = vehicle cross-sectional area (m²)
        v   = vehicle speed (m/s)
        m = mass_factor
        alpha = angle
        d = distance

    if 50 <= v0 <= 80:
        dv_dt = 0.3
    elif 81 <= v0 <= 120:
        dv_dt = 2
    else:
        dv_dt = 0

- note that all value like 50, 80, 81, 120 are in km/h

- the above mentioned equation can help to find the energy consumption between two nodes meaning an edge.
- to find the total energy consumption on a route we need to compute it for every edge on the path and sum the results to get the total energy for the whole path.
    - it should consider the change in speed on each edge except the electric road (based on traffic).
    - it should consider change in weight after delivery.
    Total_energy = sum(Travel_energy)
    - speed of vehicle on each electric road is constant and same for all electric roads. Additionally, traffic will have no impact on them as well.

- the route must start AND end at depot D
- time cost for dropping off a package is 0
- When traversing an electric road time cost for dropping off a package
- Initial node "By default its depot"

• Vehicle speed: vij = 50 km/h
• Total vehicle mass: mij = 1800 kg
• Mass of rolling inertia(aka mass factor): mf = 1.1 kg
• Gravitational acceleration: g = 9.8 m/s2
• Rolling resistance coefficient: f = 0.01
• Air density: ρ = 1.205 kg/m3
• Vehicle cross-sectional area: A = 3.5 m2
• Drag coefficient: Cx = 0.6
• Average road angle: θ = 0.86° degrees
• EV battery capacity: 100 kWh
• Charging station speed: 100 kW
• Average acceleration: dv/dt = 0.3 m/s2
• Battery threshold: 20% of total capacity

# EV Static Charging Model for Static Charging Station

## 1. Variable Definitions

All energy values must use consistent units (recommended: **kWh**).
Charging power should be in **kW**.

* `E_i(t)` → Remaining battery energy at time `t` (kWh)
* `E_bat` → Full battery capacity (kWh)
* `γ` → Mileage anxiety coefficient (0 < γ < 1)
* `E_edge(i→j)` → Energy required to traverse edge (kWh)
* `Ere_i_k` → Remaining energy upon arrival at charging station `CS_k` (kWh)
* `E_ex_i_k` → Energy after charging at `CS_k` (kWh)
* `P` → Charging power (kW)
* `ε` → Charging efficiency (0 < ε ≤ 1)
* `t_c` → Charging time (hours)

note that here Mileage anxiety coefficient = Battery threshold = 20%

## 2. Charging Decision Rule (Equation 4.2)

Before traveling from node `i` to node `j`, charge if:

```
E_i(t) ≤ γ * E_bat
OR
E_i(t) ≤ E_edge(i→j)
```

This means:

* Charge if battery is below safety threshold
* OR if battery cannot reach next segment

## 3. Remaining Energy at Charging Station (Equation 4.3)

When arriving at charging station `CS_k`:

```
Ere_i_k = E_i(tr) - E_edge(i→CS_k)
```

Where:

* `E_i(tr)` is energy before traveling to `CS_k`
* `E_edge(i→CS_k)` is energy required to reach the station

---

## 4. Charging Time Formula (Equation 4.4)

```
t_c = (E_ex_i_k - Ere_i_k) / (P * ε)
```

Conditions:

```
0 ≤ Ere_i_k ≤ E_ex_i_k ≤ E_bat
```

After charging:

```
E_current = E_ex_i_k
total_time += t_c
```


## 5. Charging Policies (Using Static Charging Station)

### Option A — Full Charge Policy

```
E_ex_i_k = E_bat
t_c = (E_bat - Ere_i_k) / (P * ε)
```

Simple and stable.

---

### Option B — Minimum Required Charge

Compute required energy to safely reach next charging station or depot:

```
E_needed_ahead = sum of E_edge until next CS or depot
safety = γ * E_bat
E_ex_i_k = min(E_bat, E_needed_ahead + safety)
t_c = (E_ex_i_k - Ere_i_k) / (P * ε)
```

More efficient, avoids unnecessary full charging.

---

### Charging Policy — Which one to use?

**Recommendation: Implement Option B (Minimum Required Charge) as the primary policy, with Option A (Full Charge) as fallback.**

**Implementation logic:**
- **Primary:** Use Option B to compute `E_needed_ahead` and charge only what's needed plus safety margin
- **Fallback:** If Option B calculation fails (e.g., cannot determine next CS/depot, or computed `E_ex_i_k` is invalid), use Option A (full charge to `E_bat`)
- **Rationale:** Option B is smarter and better for optimization (minimizes charging time), while Option A provides a safe, simple fallback

## 6. Battery Update During Travel

After traversing edge `(i → j)`:

```
E_current = E_current - E_edge(i→j)
total_time += travel_time(i→j)
```

Feasibility condition:

```
If E_current < 0 → Infeasible solution (apply heavy penalty)
```

## 8. Important Implementation Notes

* Use consistent energy units (kWh recommended)
* Clamp battery values within `[0, E_bat]`
* Charging time directly affects total travel time
* Keep charging policy deterministic during route evaluation
* Apply strong penalties for infeasible solutions

* Equation (4.2) → Charging decision
* Equation (4.3) → Remaining energy calculation
* Equation (4.4) → Charging time calculation

Below is the Markdown file content you can paste into a `.md` file. It adds the PIC and DWC (electric road system) charging models and shows exactly how to integrate them into your route simulator alongside the static charging model you already have.

# EV Charging Models — Static + Electric Road System (MD Version)

## 1. Variable / Unit Conventions (use consistently)

* Energies: **kWh**
* Power: **kW**
* Time: **hours**
* Distance / length: **km** (or meters — be consistent)
* Speed: **km/h**
* `E_current`, `E_bat`, `Ere`, `E_ex` → kWh
* `P_chg` → charging power (kW)
* `η_chg` → charging efficiency (0 < η ≤ 1)
* `t_charging` → charging time (hours)
* `L_DWC` → length of dynamic wireless charging section (km)
* `v` → average vehicle speed while on DWC section (km/h)

## 2. PIC (Parked / Plug-in / Static Linear Charging) — Linear Static Model

The common linear static charging model (PIC) updates battery state as a linear function of charging time:

**Equation (7)**
[
\lambda_{R_{k,w}} = \lambda_{i_{k,w}} + R_r \cdot t_{charging}
]

* `\lambda_{i_{k,w}}` — battery state (kWh or SOC in same units) before charging.
* `\lambda_{R_{k,w}}` — battery state after charging.
* `R_r` — effective charging rate (kW). **Clarification: R_r = P * ε**, where:
  - `P` = Charging power (kW) — same as in the static charging model
  - `ε` = Charging efficiency (0 < ε ≤ 1) — same as in the static charging model
  - This makes PIC consistent with the static charging model (Equation 4.4).
* `t_charging` — charging duration (hours).

**Implementation notes**

* Energy added: `E_added = R_r * t_charging` (kWh).
* After charging: `E_current := min(E_bat, E_current + E_added)`.
* Charging time for given `E_target`: `t_charging = (E_target - E_current) / R_r` (hours).
* Clamp `E_target ≤ E_bat` and `E_current ≥ 0`.

## 3. DWC (Dynamic Wireless Charging — on-road charging)

DWC links charging energy to the distance traveled on the charging lane and the time spent on it. Two equivalent perspectives:

**Instantaneous rate view**

* Effective charging power while on ERS: `R_r = P_chg * η_chg` (kW).
* Time spent on ERS section: `t_on = L_DWC / v` (hours).
* Energy gained crossing the section: `E_gain = R_r * t_on`.

**Direct energy formula (paper form)**

**Equation (8)**
[
E_{DWC} = P_{chg} \cdot \eta_{chg} \cdot \frac{L_{DWC}}{v}
]

* `P_chg` — charging power capacity of the DWC infrastructure (kW).
* `η_chg` — charging efficiency.
* `L_DWC` — length of ERS section (km).
* `v` — speed on that section (km/h).
* `E_DWC` (kWh) is the energy added while traversing the charging section.

**Implementation notes**

* When a route edge (or subsegment) is an ERS-enabled segment, compute:

  ```
  t_on = L_DWC / v
  E_gain = P_chg * η_chg * t_on   # same as Eq (8)
  E_current := min(E_bat, E_current + E_gain)
  total_time += t_on   # only if the time spent is not already counted in travel_time
  ```
* Often `t_on` is already included in `travel_time` for that edge; do not double-count travel time. Only add `t_on` separately if your travel_time model excluded ERS dwell time.
* DWC can produce *net positive* energy (reduces net consumption) or simply reduce net drain on that segment.

---

## 4. Integration: Static Charging (Eq. 4.2–4.4) + PIC + DWC

### Core rules summary (use these exactly in simulator)

* **Static station charging** (use Eq. 4.3 & 4.4 style):

  * `Ere = E_before - E_edge_to_CS`
  * Choose `E_ex` (target after charge) per policy (full / minimal / hybrid)
  * `t_charging = (E_ex - Ere) / (P * ε)` or using PIC `t_charging = (E_ex - Ere) / R_r` (same if `R_r = P*ε`)
  * `E_current := E_ex`
* **PIC**: follow Eq (7) directly:

  * `E_after = E_before + R_r * t_charging` (clamped to `E_bat`)
* **DWC**: while traversing ERS section:

  * `E_gain = P_chg * η_chg * (L_DWC / v)` (Eq 8)
  * `E_current := min(E_bat, E_current + E_gain)`

### Conflicts / precedence

* If traversing DWC and also planning a static stop at a charging node located inside the ERS section, decide ordering:

  * Typical: add `E_gain` from DWC during traversal, then compute `Ere` at arrival and evaluate static charging decision (Eq. 4.2).
* Always clamp battery to `[0, E_bat]`.

## 6. Practical Points and Gotchas

* **Units:** be obsessive. Convert Wh ↔ kWh correctly.
* **Double counting time:** travel_time for an ERS edge usually already accounts for time spent on the ERS; only add extra charging time for static charging. DWC energy is obtained during travel, not as extra stop time.
* **Nonlinear charging / power limits:** PIC linear model assumes constant effective power. If you later include tapering curves, replace linear `R_r * t` with an integral over `P(t)`.
* **Look-ahead:** when using minimal-charge policies, compute `E_needed_ahead` until next CS or depot; if next CS is only reachable via ERS that will add energy, include expected `E_gain` from DWC when computing feasibility.

## 7. Quick Reference Equations

* PIC (Eq 7): (\lambda_{R} = \lambda_{i} + R_r \cdot t_{charging})
* DWC (Eq 8): (E_{DWC} = P_{chg} \cdot \eta_{chg} \cdot \dfrac{L_{DWC}}{v})
* Static charging time (Eq 4.4 style): (t_c = \dfrac{E_{ex} - E_{re}}{P \cdot \varepsilon})

8.  Now we have map with nodes and edges, goal which is make delivery to each customer from current node and return to depot, and ability to compute total_travel_time and total_energy for any customer furthermore.

9. Now write code
- consider input as we discussed above
- program will find best path to make deliveries to all the customers and to return to depot with least travel time.

Assumptions and constraints
- Queuing time at Charging Station is 0
- Vehicle payload decreases after each delivery; this reduction in mass is accounted for in the energy calculations.

Values to remember
- battery_threshold = 20%

Clarifications

- Unit of energy is kWh everywhere, only where we need to explain the user like State of charge of total battery, printed it in percentages.

- air_density and angle for each road are following.

    air_density = 1.205
    angle = 0.86


Key Metrics to Track (print in output)
- Total Travel Time: Time to reach destination
- Total Distance: Sum of distances traveled
- Total Energy Depletion: Sum of energy consumed on all edges
- Total Energy Charged at Stations: Sum of all stationary charging
- Total Energy Charged on E-Roads: Sum of all dynamic charging
- Route Sequence: Exact order of nodes visited
- Final Battery Level: Battery percentage when reaching destination
- Boxes Delivered: Confirmation that all boxes were dropped
- Charging Stops: Which stations were used and for how long
- Runtime of program

Also print the following trails for detailed route analysis:

```
SoC Trail: D(100.0%) -> 2(99.3%) -> 4(98.6%) -> L8(98.1%) -> ...
Energy Consumption Trail: D - 0.7 kWh > 2 - 0.6 kWh > 4 - 0.6 kWh > ...
Travel Time Trail: D - 9.80 min > 2 - 4.21 min > 4 - 3.95 min > ...
```

# Clarke and Wright Savings Algorithm — Design Specifications

## Overview

The Clarke and Wright (C&W) Savings Algorithm is a classical constructive heuristic for vehicle routing problems. Instead of using a metaheuristic (like GA), it builds a high-quality route deterministically by computing **savings** from merging individual customer trips.

**Core Idea:** Start with n "shuttle" routes (one per customer: D → Li → D), then iteratively merge them based on the largest savings value until a single feasible tour remains.

## CW.1 Savings Computation

### Step 1: Compute Shortest Paths

Using Dijkstra (same as the GA project), compute shortest-path distances between:
- Depot D to every customer Li
- Every customer Li to every customer Lj (for all pairs i ≠ j)

Let `d(i, j)` = shortest path distance from node i to node j.

### Step 2: Compute Savings Matrix

For every pair of customers (Li, Lj) where i ≠ j, compute the savings:

```
s(Li, Lj) = d(D, Li) + d(Lj, D) - d(Li, Lj)
```

**Interpretation:** The savings `s(Li, Lj)` represents how much distance (and approximately time) is saved by visiting customer Li and then Lj consecutively in one route, instead of making two separate round trips (D→Li→D and D→Lj→D).

### Step 3: Sort Savings

Sort all savings values in **descending order** (largest savings first). This creates a priority list of the most beneficial customer-pair merges.

## CW.2 Route Construction (Sequential Merge)

Starting state: n individual routes, each: D → Li → D

Repeat until all customers are in a single tour:

```
for each savings s(Li, Lj) in descending order:
    if Li and Lj are in different routes:
        if Li is at the END of its route (just before returning to D):
            if Lj is at the START of its route (just after leaving D):
                if merging is FEASIBLE (battery, energy constraints):
                    MERGE: connect Li → Lj (remove Li→D and D→Lj)
```

**Merge feasibility check** — Before merging, simulate the combined route through the decoder/simulator to verify:
1. Battery never drops below 0 at any point
2. All nodes on the merged path are reachable via the graph
3. Charging station detours (if needed) are feasible

## CW.3 Savings Metric Options

### Option A — Distance-Based Savings (Default)
```
s(Li, Lj) = d(D, Li) + d(Lj, D) - d(Li, Lj)
```
Uses shortest-path distances. Simple and fast to compute.

### Option B — Time-Based Savings
```
s(Li, Lj) = t(D, Li) + t(Lj, D) - t(Li, Lj)
```
Where `t(i, j)` is travel time along the shortest path from i to j. More aligned with the time-minimization objective.

### Option C — Objective-Based Savings
```
s(Li, Lj) = cost(D→Li→D) + cost(D→Lj→D) - cost(D→Li→Lj→D)
```
Where `cost()` uses the full objective function (sum_arrival_times - 0.01 * sum_battery_levels). Most accurate but most expensive to compute.

**Recommendation: Implement Option A (Distance-Based) as default. Optionally allow Option B or C as configuration parameters.**

## CW.4 Decoder / Simulator Logic

The route evaluation simulator is **identical** to the GA project. Once C&W determines the customer visit order, the decoder:

```
def decode(customer_order, graph, params):
    route = [Depot]
    E_current = initial_battery (kWh)
    total_time = 0
    load = num_customers * package_weight

    for each customer in customer_order:
        # Find shortest path from current_node to customer
        # For each edge on that path:
        #   1. Check charging decision (Eq 4.2) BEFORE traversing
        #   2. If charge needed and CS reachable, detour to nearest CS
        #   3. Traverse edge: update time, energy, handle electric roads
        #   4. If customer reached, deliver package (load -= package_weight)

    # Return to depot from last customer
    # Final: compute objective = sum_arrival_times - 0.01 * sum_battery_levels
```

This is exactly the same decoder that the GA project uses. All energy consumption equations, charging logic (Eq 4.2–4.4), DWC calculations (Eq 8), and feasibility checks remain unchanged.

## CW.5 Post-Optimization (Local Search Improvement)

After the C&W algorithm constructs an initial solution, apply local search to improve it:

### 2-opt Improvement
Repeatedly try reversing sub-segments of the customer visit order:
```
for i in range(n_customers):
    for j in range(i+2, n_customers):
        new_order = customer_order[:i] + reversed(customer_order[i:j+1]) + customer_order[j+1:]
        if cost(new_order) < cost(current_order):
            current_order = new_order
            improved = True
```

### Or-opt Improvement (optional)
Try relocating a single customer (or a chain of 2–3 consecutive customers) to a different position in the tour:
```
for each customer Li in tour:
    for each position p in tour:
        new_order = move Li from current position to position p
        if cost(new_order) < cost(current_order):
            current_order = new_order
```

**Recommendation: Always run 2-opt after C&W construction. Or-opt is optional but recommended for better quality.**

### Local Search Parameters
```
max_iterations: 1000          # maximum local search iterations
improvement_threshold: 1e-6   # stop if improvement is less than this
```

## CW.6 Infeasibility Handling

During the merge step and final evaluation:
- If battery < 0 at any point during simulation → INFEASIBLE merge (skip this merge)
- If vehicle cannot reach next node or depot from current position → INFEASIBLE merge
- Penalty for final solution (if somehow infeasible): fitness += PENALTY_INFEASIBLE (99999)

Unlike GA, C&W typically produces feasible solutions because infeasible merges are rejected upfront. However, the penalty mechanism remains as a safety net.

## CW.7 Penalty Constants

```
PENALTY_BATTERY_DEAD = 99999      # battery drops below 0 during route
PENALTY_UNREACHABLE = 99999       # cannot reach next node or depot
```

## CW.8 Algorithm Parameters

```
savings_metric: "distance"        # "distance", "time", or "objective"
local_search: "2opt"              # "2opt", "oropt", "both", or "none"
max_ls_iterations: 1000           # max iterations for local search
ls_improvement_threshold: 1e-6    # convergence threshold for local search
random_seed: 42                   # for tie-breaking reproducibility
```

## CW.9 Logging & Outputs

### Per-run output (final summary):
  - objective_value (raw objective, same formula as GA: sum_arrival_times - 0.01 * sum_battery_levels)
  - best_route (full decoded route with all intermediate nodes)
  - total_travel_time (hours)
  - total_distance (km)
  - total_energy_consumed (kWh)
  - total_energy_charged_static (kWh)
  - total_energy_charged_dynamic (kWh)
  - final_battery_level (kWh and %)
  - packages_delivered (count)
  - charging_stops (list of: station_id, time_charged, energy_added)
  - route_sequence (ordered list of all nodes visited)

### Console output format:

```
============================================================
  CLARKE & WRIGHT SAVINGS ALGORITHM — SOLUTION SUMMARY
============================================================
  Objective Value (legacy):         3.930482
  Total Travel Time:          2.2733 hours (136.40 min)
  Total Distance:             87.9400 km
  Total Energy Depletion:     15.6954 kWh
  Total Energy Charged (SC):  0.0000 kWh
  Total Energy Charged (DWC): 1.8326 kWh
  Final Battery Level:        86.1372 kWh (86.1%)
  Boxes Delivered:            10
  Charging Stops:             None

Route Sequence: D -> 2 -> 4 -> L8 -> ... -> D

SoC Trail: D(100.0%) -> 2(99.3%) -> 4(98.6%) -> ...

Energy Consumption Trail: D - 0.7 kWh > 2 - 0.6 kWh > ...

Travel Time Trail: D - 9.80 min > 2 - 4.21 min > ...

  Savings computation time:    0.05 seconds
  Local search time:           1.23 seconds
  Total Runtime:               1.28 seconds
```

### Savings matrix output (optional, for debugging):
  - Print or save the savings matrix to CSV
  - Show top-10 savings pairs

## CW.10 Comparison with GA

To facilitate comparison between C&W and GA solutions on the same instance:
- Both use the **exact same** objective function: `sum_arrival_times - 0.01 * sum_battery_levels`
- Both use the **exact same** decoder/simulator (energy, charging, DWC calculations)
- Both use the **exact same** JSON input format
- Both produce the **exact same** output metrics
- C&W is deterministic (single run, no randomness except tie-breaking)
- C&W is typically much faster than GA (seconds vs. minutes)
- GA may find better solutions due to its stochastic search, while C&W provides a quick baseline

# Test Instances

Use the same JSON instance files as the GA project. The C&W algorithm should be tested on the same instances to allow direct comparison.

### Instance 1: Trivial (no charging needed)
```
Nodes: D, L1
Edges: D—L1 (distance=5km, traffic=1.0, type=normal)
Customers: 1
Expected: D → L1 → D, no charging needed
Purpose: Verify basic routing, energy calc, objective calc
```

### Instance 2: Simple with charging
```
Nodes: D, L1, L2, CS1, 1
Edges:
  D—1 (10km, traffic=1.0, normal)
  1—L1 (8km, traffic=0.8, normal)
  1—CS1 (5km, traffic=1.0, normal)
  CS1—L2 (7km, traffic=0.9, normal)
  L2—D (12km, traffic=1.0, normal)
Customers: 2
Purpose: Verify charging station detour logic, weight reduction after delivery
```

### Instance 3: With electric road
```
Nodes: D, L1, L2, CS1, Ns1, Ne1, 1, 2
Edges:
  D—1 (10km, traffic=1.0, normal)
  1—Ns1 (3km, traffic=1.0, normal)
  Ns1—Ne1 (8km, traffic=1.0, electric)
  Ne1—L1 (4km, traffic=0.9, normal)
  1—2 (6km, traffic=0.7, normal)
  2—CS1 (5km, traffic=1.0, normal)
  CS1—L2 (7km, traffic=0.85, normal)
  L2—D (9km, traffic=1.0, normal)
  Ne1—2 (3km, traffic=1.0, normal)
Customers: 2
Purpose: Verify DWC energy gain, electric road speed handling,
         compare route via electric road vs normal road
```

### Instance 4: Stress test (larger)
```
Nodes: D, L1-L5, CS1-CS2, Ns1, Ne1, Ns2, Ne2, 1-5
[Define 15-20 edges with varied traffic factors]
Customers: 5
Purpose: Verify savings computation, merge logic, local search improvement
```

# JSON Input Format

The exact same JSON format is used for input. Example:

```json
{
    "nodes": [
        {"id": "D",  "type": "depot"},
        {"id": "1",  "type": "intersection"},
        {"id": "L1", "type": "customer"},
        {"id": "L2", "type": "customer"},
        {"id": "CS1","type": "charging_station"},
        {"id": "Ns1","type": "electric_road_start", "segment": 1},
        {"id": "Ne1","type": "electric_road_end",   "segment": 1}
    ],
    "edges": [
        {"from": "D", "to": "1", "distance": 10, "traffic_factor": 1.0, "type": "normal"},
        {"from": "Ns1", "to": "Ne1", "distance": 8, "traffic_factor": 1.0, "type": "electric"}
    ],
    "base_speed": 50,
    "initial_battery_percent": 100,
    "starting_node": "D",
    "battery_capacity": 100,
    "vehicle_mass": 1800,
    "rolling_resistance": 0.01,
    "drag_coefficient": 0.6,
    "cross_sectional_area": 3.5,
    "mass_factor": 1.1,
    "package_weight": 5,
    "charging_power": 100,
    "charging_efficiency": 0.95,
    "dwc_power": 20,
    "dwc_efficiency": 0.85,
    "electric_road_speed": 50,
    "air_density": 1.205,
    "angle": 0.86
}
```

# Project Structure

```
ev-routing-cw/
├── main.py                  # Entry point: runs C&W, handles CLI args
├── config.py                # All constants, default parameters (same as GA project)
├── graph.py                 # Graph representation, shortest path Dijkstra (same as GA project)
├── energy.py                # Energy consumption & charging calculations (same as GA project)
├── simulator.py             # Route decoder: customer_order → full route with metrics (same as GA project)
├── savings.py               # Clarke & Wright savings computation and route merging
├── local_search.py          # Post-optimization: 2-opt and Or-opt improvement
├── logger.py                # Output logging, summary writing
├── instances/               # Test instance definitions (same JSON files as GA project)
│   ├── instance_1.json
│   ├── instance_2.json
│   ├── instance_3.json
│   └── instance_4.json
├── results/                 # Output directory for logs
└── README.md
```

### Module Responsibilities

- **`config.py`** — Identical to GA project. All physics constants, vehicle parameters, battery parameters, charging parameters. C&W-specific parameters (savings_metric, local_search type) added here.
- **`graph.py`** — Identical to GA project. Graph loading from JSON, Dijkstra shortest path, node queries.
- **`energy.py`** — Identical to GA project. Energy consumption formula (Eq 7), travel time, charging decision (Eq 4.2), static charging time (Eq 4.4), DWC energy gain (Eq 8).
- **`simulator.py`** — Identical to GA project. Takes a customer visit order (instead of "chromosome") and decodes it into a full route with all metrics. Same RouteResult dataclass.
- **`savings.py`** — **NEW module.** Computes savings matrix, performs iterative route merging following the C&W algorithm.
- **`local_search.py`** — **NEW module.** Implements 2-opt and Or-opt improvement heuristics applied after C&W construction.
- **`main.py`** — Entry point. Loads instance, runs C&W + local search, prints summary in same format as GA project.
- **`logger.py`** — Simplified from GA project. No per-generation CSV needed. Writes final solution summary.
