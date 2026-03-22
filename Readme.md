
How to run
```
python main.py --instance instances/instance_4.json --local-search both --show-savings
```

# Project Structure
EVRP-SCS-and-DWC-CW/
├── main.py              # Entry point with CLI args
├── config.py            # Constants, defaults, penalties
├── graph.py             # Graph loading + Dijkstra shortest path
├── energy.py            # Energy consumption (Eq 7), DWC (Eq 8), charging time (Eq 4.4)
├── simulator.py         # Route decoder — full simulation with charging logic
├── savings.py           # Clarke & Wright savings computation + route merging
├── local_search.py      # 2-opt and Or-opt post-optimization
├── logger.py            # Formatted console output & trail printing
└── instances/
    ├── instance_1.json  # Trivial (1 customer, no charging)
    ├── instance_2.json  # 2 customers + CS, weight reduction
    ├── instance_3.json  # 2 customers + electric road (DWC)
    └── instance_4.json  # Stress: 5 customers, 2 CSs, 2 e-roads

# Auto-detect workers (default)
python main.py --instance instances/40c_20bss_100total.json --local-search both

# Explicit worker count
python main.py --instance instances/40c_20bss_100total.json --local-search both --workers 8

# Sequential (baseline)
python main.py --instance instances/40c_20bss_100total.json --local-search both --workers 1

# Save SUMO run output to a text file
python main.py --sumo-cfg "C:\Users\ranji\Documents\Projects\sumo\sumo.sumocfg" --customers 200 --cs-ratio 0.15 --electric-road-fraction 0.20 --traffic-min 0.6 --traffic-max 1.0 --sumo-seed 42 --local-search none --output-txt outputs\sumo_run_200.txt
