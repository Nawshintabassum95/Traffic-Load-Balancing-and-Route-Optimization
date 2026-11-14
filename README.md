# Traffic Load Balancing and Route Optimization

A C++ implementation of a traffic load balancing and route optimization system using **Yen's K-Shortest Paths Algorithm** and a greedy load-aware path selection strategy.

## Overview

This project simulates a connected road network and distributes transportation requests among alternative routes.

The main objective is to balance traffic loads across different roads while minimizing the total travel distance of vehicles.

The system compares three routing strategies to evaluate their performance in terms of road utilization, traffic distribution, total travel distance, and execution time.

## Key Features

- Random connected weighted graph generation.
- Random transportation request generation.
- Multiple time-step traffic simulation.
- Yen's K-Shortest Loopless Paths Algorithm.
- Greedy load-aware route selection.
- Adaptive vehicle-to-path assignment.
- Cumulative road load updates.
- Comparison of three routing strategies.
- Performance evaluation using multiple metrics.
- Configurable simulation parameters.

## Algorithms Used

### 1. Shortest Path Routing

- Uses Dijkstra's Algorithm to find the shortest-distance path.
- All vehicles belonging to a request follow the same selected path.
- Focuses only on minimizing travel distance.

### 2. Greedy Load Balancing

- Uses Yen's K-Shortest Paths Algorithm to generate alternative routes.
- Selects the route with the minimum current average road load.
- Assigns vehicles one by one and updates road loads after each assignment.

### 3. Proposed Algorithm

**Yen's K-Shortest Paths Based Greedy Adaptive Load-Balancing Heuristic**

The proposed approach considers both travel distance and current road load during route selection.

It evaluates the candidate paths using a combined cost function that balances:

- Total travel distance.
- Average current road load.

A configurable lambda parameter controls the importance of traffic load compared to distance.

The path with the minimum combined cost is selected for each vehicle.

After every assignment, the load of all roads in the selected path is updated. This allows subsequent vehicles to make routing decisions based on the updated traffic distribution.

## Experimental Configuration

Default simulation parameters:

| Parameter | Value |
|---|---:|
| Number of vertices | 20 |
| Additional random roads | 30 |
| Time steps | 3 |
| Requests per time step | 2 |
| Vehicles per request | 5–15 |
| Candidate paths (K) | 3 |
| Lambda | 0.5 |
| Random seed | 42 |

These parameters can be modified in the configuration section of `main.cpp`.

## Performance Evaluation

The system compares the three algorithms using the following metrics:

- **Maximum Road Load:** Highest number of assigned vehicles on any single road.
- **Average Road Load:** Average cumulative vehicle load across all roads.
- **Load Variance:** Measures how evenly traffic is distributed throughout the network.
- **Total Travel Distance:** Total distance traveled by all assigned vehicles.
- **Running Time:** Execution time of each routing strategy in milliseconds.

## Project Workflow

1. Generate a random connected weighted road network.
2. Generate transportation requests for multiple time steps.
3. Calculate the shortest path or generate alternative paths using Yen's algorithm.
4. Evaluate candidate routes based on the selected strategy.
5. Assign vehicles to routes one by one.
6. Update road loads after each vehicle assignment.
7. Calculate performance metrics.
8. Compare the results of all three algorithms.

## Technologies Used

- **Language:** C++
- **Standard:** C++17
- **Graph Representation:** Adjacency List
- **Priority Queue:** STL
- **Algorithms:** Dijkstra's Algorithm and Yen's K-Shortest Paths
- **Random Generation:** `rand()`
- **Execution Time Measurement:** C++ Chrono Library

## Installation and Execution

### Requirements

- C++ compiler supporting C++17.
- GCC, MinGW, or Code::Blocks.

### Compile

```bash
g++ -std=c++17 -O2 main.cpp -o traffic
```

### Run on Windows

```bash
traffic.exe
```

### Run on Linux or macOS

```bash
./traffic
```

## Output

The program displays:

- Generated road network information.
- Transportation requests for each time step.
- Selected routes and vehicle distributions.
- Updated cumulative road loads.
- Final performance comparison.

## Assumptions and Limitations

- The road network is connected, undirected, and weighted.
- All roads and vehicles are treated equally.
- Road distances remain fixed throughout the simulation.
- Each vehicle follows one selected route.
- Each vehicle contributes one unit of load to every road it traverses.
- Road loads accumulate across time steps.
- Road capacity constraints are not considered.
- Actual vehicle speed and travel time are not simulated.
- Only a limited number of alternative paths are considered.
- The proposed algorithm is a heuristic and does not guarantee a globally optimal solution.
- 
## Report Link
https://drive.google.com/file/d/1JEuvLL62M2BEJx71JoyTxseKu-eK6EPC/view?usp=sharing
