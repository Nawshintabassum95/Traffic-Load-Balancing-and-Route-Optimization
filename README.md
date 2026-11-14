# Traffic Load Balancing and Route Optimization

A C++ implementation of a traffic load balancing and route optimization system using Yen's K-Shortest Paths Algorithm and a greedy load-aware path selection strategy.

## Overview

This project simulates a connected, weighted road network and distributes transportation requests among alternative routes.

The main objective is to balance traffic loads across roads while minimizing the total travel distance of vehicles.

The proposed approach combines:
- Yen's K-Shortest Loopless Paths Algorithm
- Greedy adaptive path selection
- Distance-aware and load-aware cost evaluation

## Problem Description

The road network is represented by a connected, undirected, weighted graph:

\[
G=(V,E,w)
\]

Where:
- `V` represents road intersections.
- `E` represents bidirectional roads.
- `w(e)` represents the distance of a road.

Each transportation request is represented as:

\[
r_i=(n_i,s_i,d_i)
\]

Where:
- `n_i` = Number of vehicles
- `s_i` = Source intersection
- `d_i` = Destination intersection

Vehicles are assigned to one or more valid paths while considering road loads and travel distances.

## Features

- Random connected weighted graph generation.
- Random transportation request generation.
- Multiple discrete time-step simulation.
- Yen's K-Shortest Paths implementation.
- Adaptive vehicle-to-path assignment.
- Cumulative road load updates.
- Comparison of three routing strategies.
- Performance evaluation using multiple metrics.
- Configurable graph size, number of requests, K, and lambda.

## Algorithms Used

### 1. Shortest Path Routing

Uses Dijkstra's algorithm to find the minimum-distance path.

All vehicles belonging to a request follow the same shortest path.

### 2. Greedy Load Balancing

Uses Yen's K-Shortest Paths Algorithm to generate candidate paths.

Among the candidates, it selects the path with the minimum average current road load.

### 3. Proposed Algorithm

**Yen's K-Shortest Paths Based Greedy Adaptive Load-Balancing Heuristic**

The proposed approach considers both distance and current road load.

Its path selection cost function is:

\[
Cost(P)=Distance(P)+\lambda \times LoadCost(P)
\]

Where:

\[
Distance(P)=\sum_{e\in P}w(e)
\]

\[
LoadCost(P)=\frac{\sum_{e\in P}L(e)}{|P|}
\]

Here:
- `Distance(P)` = Total path distance.
- `LoadCost(P)` = Average current load of the roads in the path.
- `L(e)` = Current cumulative vehicle load on road e.
- `lambda` = Parameter controlling the trade-off between distance and load balancing.

The path with the minimum combined cost is selected for each vehicle.

After assigning a vehicle, the load of every road in its selected path is incremented by one.

## Experimental Parameters

The default simulation configuration is:

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

These values can be modified in the configuration section of `main.cpp`.

## Performance Evaluation

The algorithms are evaluated using five metrics.

### Maximum Road Load

\[
MaxLoad=\max_{e\in E}L(e)
\]

Measures the maximum number of assigned vehicles using any road.

### Average Road Load

\[
AverageLoad=\frac{\sum_{e\in E}L(e)}{|E|}
\]

Measures the average cumulative vehicle load per road.

### Load Variance

\[
Variance=\frac{1}{|E|}\sum_{e\in E}(L(e)-AverageLoad)^2
\]

Measures how unevenly vehicles are distributed across roads.

### Total Travel Distance

\[
TotalDistance=\sum_{v}Distance(P_v)
\]

Measures the total distance traveled by all assigned vehicles.

### Running Time

Measures the execution time of each routing algorithm in milliseconds.

## Project Workflow

1. Generate a random connected weighted graph.
2. Generate transportation requests over multiple time steps.
3. Calculate the shortest path or generate K candidate paths using Yen's algorithm.
4. Evaluate candidate paths using the selected routing strategy.
5. Assign vehicles one by one.
6. Update cumulative road loads after each assignment.
7. Evaluate performance metrics.
8. Compare the three algorithms.

## Technologies Used

- Programming Language: C++
- Standard: C++17
- Graph Representation: Adjacency List
- Priority Queue: STL
- Algorithms: Dijkstra's Algorithm and Yen's K-Shortest Paths
- Random Generation: `rand()`
- Performance Measurement: C++ Chrono Library

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
- Assigned paths and vehicle distributions.
- Cumulative road loads after each time step.
- Final performance comparison.

The comparison table includes:

| Algorithm | Maximum Load | Average Load | Variance | Total Distance | Running Time |
|---|---|---|---|---|---|
| Shortest Path | ✓ | ✓ | ✓ | ✓ | ✓ |
| Greedy | ✓ | ✓ | ✓ | ✓ | ✓ |
| Proposed | ✓ | ✓ | ✓ | ✓ | ✓ |

## Complexity Analysis

Let:
- `V` = Number of vertices.
- `E` = Number of roads.
- `K` = Number of candidate paths.
- `N` = Total number of vehicles.

Dijkstra's algorithm with a priority queue:

\[
O((V+E)\log V)
\]

Yen's K-Shortest Paths algorithm:

\[
O(KV(E+V)\log V)
\]

For the proposed method, Yen's candidate generation is performed once per request. Vehicle assignment then evaluates the K candidate paths using the current road loads.

A simplified overall upper-bound estimate is:

\[
O\left(RKV(E+V)\log V+NKP\right)
\]

Where `R` is the number of requests and `P` is the maximum number of edges in a candidate path.

The actual running time depends on graph structure, K, request count, and path lengths.

## Assumptions and Limitations

- The road network is connected, undirected, and weighted.
- All roads and vehicles are treated as identical.
- Road distances are fixed.
- Each vehicle follows one selected path.
- Each vehicle contributes one unit to every road it traverses.
- Road loads are cumulative and are not reset between time steps.
- Road capacity constraints are not considered.
- Actual travel time, vehicle speed, and departure/arrival dynamics are not simulated.
- Yen's algorithm generates only K candidate paths, so the globally best load distribution may not be included.
- The proposed method is a heuristic and does not guarantee a globally optimal solution.



## License

This project is available for educational and research purposes.
