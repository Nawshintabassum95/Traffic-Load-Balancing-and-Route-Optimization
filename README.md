//Traffic-Load-Balancing-and-Route-Optimization

#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <set>
#include <map>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <climits>
#include <string>

using namespace std;

// ==========================================
// DATA STRUCTURES
// ==========================================

struct Edge {
    int to;
    int weight;
    int id;
};

struct Road {
    int u, v, weight;
};

struct Request {
    int vehicles;
    int source;
    int destination;
};

struct Candidate {
    vector<int> path;
    int distance;
};

struct CompareCandidate {
    bool operator()(const Candidate &a,
                    const Candidate &b) const {

        if (a.distance != b.distance)
            return a.distance > b.distance;

        return a.path > b.path;
    }
};

struct Result {
    vector<int> roadLoad;

    vector<vector<map<vector<int>, int>>> assignments;

    vector<vector<int>> loadHistory;

    double totalDistance = 0;
    double runningTime = 0;
};

struct Metrics {
    int maxLoad = 0;
    double averageLoad = 0;
    double variance = 0;
    double totalDistance = 0;
    double runningTime = 0;
};

// ==========================================
// CONFIGURATION
// ==========================================

const int V = 20;
const int EXTRA_EDGES = 30;

const int TIME_STEPS = 3;
const int REQUESTS_PER_STEP = 2;

const int K = 3;

const double LAMBDA = 0.5;

// Fixed seed for reproducible experiments.
// Change the seed to generate another graph.

const int RANDOM_SEED = 42;

// ==========================================
// RANDOM NUMBER GENERATION
// ==========================================

int randomNumber(int low, int high) {
    return low + rand() % (high - low + 1);
}

// ==========================================
// FIND ROAD ID
// ==========================================

int findEdgeId(
    const vector<vector<Edge>> &graph,
    int u,
    int v
) {
    for (int i = 0; i < (int)graph[u].size(); i++) {

        if (graph[u][i].to == v)
            return graph[u][i].id;
    }

    return -1;
}

// ==========================================
// STEP 1: GENERATE CONNECTED GRAPH
// ==========================================

void generateGraph(
    int vertices,
    int extraEdges,
    vector<vector<Edge>> &graph,
    vector<Road> &roads
) {
    graph.assign(vertices, vector<Edge>());
    roads.clear();

    set<pair<int, int>> used;

    // Generate a spanning tree first.
    // This guarantees connectivity.

    for (int i = 1; i < vertices; i++) {

        int parent = randomNumber(0, i - 1);
        int weight = randomNumber(1, 20);

        int id = roads.size();

        roads.push_back({i, parent, weight});

        graph[i].push_back({parent, weight, id});
        graph[parent].push_back({i, weight, id});

        used.insert({min(i, parent), max(i, parent)});
    }

    // Add extra random roads.

    int maximumEdges = vertices * (vertices - 1) / 2;

    int target = min(
        vertices - 1 + extraEdges,
        maximumEdges
    );

    while ((int)roads.size() < target) {

        int u = randomNumber(0, vertices - 1);
        int v = randomNumber(0, vertices - 1);

        if (u == v)
            continue;

        int a = min(u, v);
        int b = max(u, v);

        if (used.count({a, b}))
            continue;

        int weight = randomNumber(1, 20);

        int id = roads.size();

        roads.push_back({u, v, weight});

        graph[u].push_back({v, weight, id});
        graph[v].push_back({u, weight, id});

        used.insert({a, b});
    }
}

// ==========================================
// STEP 2: GENERATE REQUESTS
// ==========================================

void generateRequests(
    int vertices,
    vector<vector<Request>> &requests
) {
    requests.assign(TIME_STEPS, vector<Request>());

    for (int t = 0; t < TIME_STEPS; t++) {

        for (int r = 0; r < REQUESTS_PER_STEP; r++) {

            int source = randomNumber(0, vertices - 1);
            int destination = randomNumber(0, vertices - 1);

            while (source == destination) {
                destination = randomNumber(0, vertices - 1);
            }

            int vehicles = randomNumber(5, 15);

            requests[t].push_back({
                vehicles,
                source,
                destination
            });
        }
    }
}

// ==========================================
// DIJKSTRA'S SHORTEST PATH
// Supports banned vertices and edges.
// Used inside Yen's algorithm too.
// ==========================================

vector<int> dijkstra(
    const vector<vector<Edge>> &graph,
    int source,
    int destination,
    const vector<bool> &bannedNodes,
    const vector<bool> &bannedEdges
) {
    int vertices = graph.size();

    vector<int> dist(vertices, INT_MAX);
    vector<int> parent(vertices, -1);

    priority_queue<
        pair<int, int>,
        vector<pair<int, int>>,
        greater<pair<int, int>>
    > pq;

    dist[source] = 0;

    pq.push({0, source});

    while (!pq.empty()) {

        int currentDistance = pq.top().first;
        int u = pq.top().second;

        pq.pop();

        if (currentDistance != dist[u])
            continue;

        if (u == destination)
            break;

        for (int i = 0; i < (int)graph[u].size(); i++) {

            Edge e = graph[u][i];

            int v = e.to;

            if (bannedNodes[v] || bannedEdges[e.id])
                continue;

            if (dist[u] + e.weight < dist[v]) {

                dist[v] = dist[u] + e.weight;
                parent[v] = u;

                pq.push({dist[v], v});
            }
        }
    }

    vector<int> path;

    if (dist[destination] == INT_MAX)
        return path;

    int current = destination;

    while (current != -1) {

        path.push_back(current);

        if (current == source)
            break;

        current = parent[current];
    }

    reverse(path.begin(), path.end());

    if (path.empty() || path[0] != source)
        return {};

    return path;
}

// ==========================================
// CALCULATE PATH DISTANCE
// ==========================================

int pathDistance(
    const vector<vector<Edge>> &graph,
    const vector<int> &path
) {
    int total = 0;

    for (int i = 0; i + 1 < (int)path.size(); i++) {

        int id = findEdgeId(graph, path[i], path[i + 1]);

        if (id != -1) {

            for (int j = 0; j < (int)graph[path[i]].size(); j++) {

                if (graph[path[i]][j].id == id) {

                    total += graph[path[i]][j].weight;
                    break;
                }
            }
        }
    }

    return total;
}

// ==========================================
// STEP 3: YEN'S K-SHORTEST LOOPLESS PATHS
// ==========================================

vector<vector<int>> yenKShortestPaths(
    const vector<vector<Edge>> &graph,
    int source,
    int destination,
    int K
) {
    int vertices = graph.size();

    int E = 0;

    for (int i = 0; i < vertices; i++)
        E += graph[i].size();

    E /= 2;

    vector<vector<int>> acceptedPaths;

    vector<bool> noBannedNodes(vertices, false);
    vector<bool> noBannedEdges(E, false);

    // First shortest path

    vector<int> firstPath = dijkstra(
        graph,
        source,
        destination,
        noBannedNodes,
        noBannedEdges
    );

    if (firstPath.empty())
        return acceptedPaths;

    acceptedPaths.push_back(firstPath);

    set<vector<int>> generated;

    generated.insert(firstPath);

    priority_queue<
        Candidate,
        vector<Candidate>,
        CompareCandidate
    > candidates;

    // Generate remaining K-1 paths

    for (int k = 1; k < K; k++) {

        vector<int> previousPath = acceptedPaths[k - 1];

        // Consider each vertex as a spur node

        for (int i = 0; i + 1 < (int)previousPath.size(); i++) {

            vector<int> rootPath(
                previousPath.begin(),
                previousPath.begin() + i + 1
            );

            vector<bool> bannedNodes(vertices, false);
            vector<bool> bannedEdges(E, false);

            // Ban root path vertices before spur node

            for (int j = 0; j < i; j++)
                bannedNodes[rootPath[j]] = true;

            // Ban edges of already accepted paths
            // that share the same root path

            for (int j = 0; j < (int)acceptedPaths.size(); j++) {

                vector<int> p = acceptedPaths[j];

                if ((int)p.size() <= i + 1)
                    continue;

                bool sameRoot = true;

                for (int x = 0; x <= i; x++) {

                    if (p[x] != rootPath[x]) {

                        sameRoot = false;
                        break;
                    }
                }

                if (sameRoot) {

                    int id = findEdgeId(
                        graph,
                        p[i],
                        p[i + 1]
                    );

                    if (id != -1)
                        bannedEdges[id] = true;
                }
            }

            int spurNode = rootPath.back();

            // Find shortest spur path

            vector<int> spurPath = dijkstra(
                graph,
                spurNode,
                destination,
                bannedNodes,
                bannedEdges
            );

            if (spurPath.empty())
                continue;

            // Combine root path and spur path

            vector<int> totalPath = rootPath;

            for (int j = 1; j < (int)spurPath.size(); j++)
                totalPath.push_back(spurPath[j]);

            // Avoid duplicate paths

            if (!generated.count(totalPath)) {

                int distance = pathDistance(graph, totalPath);

                candidates.push({
                    totalPath,
                    distance
                });

                generated.insert(totalPath);
            }
        }

        // No more candidate paths

        if (candidates.empty())
            break;

        Candidate best = candidates.top();

        candidates.pop();

        acceptedPaths.push_back(best.path);
    }

    return acceptedPaths;
}

// ==========================================
// CALCULATE AVERAGE ROAD LOAD OF A PATH
// ==========================================

double calculateLoadCost(
    const vector<vector<Edge>> &graph,
    const vector<int> &path,
    const vector<int> &load
) {
    double totalLoad = 0;

    int count = 0;

    for (int i = 0; i + 1 < (int)path.size(); i++) {

        int id = findEdgeId(graph, path[i], path[i + 1]);

        if (id != -1) {

            totalLoad += load[id];
            count++;
        }
    }

    if (count == 0)
        return 0;

    return totalLoad / count;
}

// ==========================================
// STEP 4: SELECT BEST CANDIDATE PATH
//
// mode 0 = Shortest
// mode 1 = Greedy
// mode 2 = Proposed
// ==========================================

vector<int> selectPath(
    const vector<vector<int>> &paths,
    const vector<vector<Edge>> &graph,
    const vector<int> &load,
    int mode,
    double lambda
) {
    vector<int> bestPath;

    double minimumCost = 1e18;

    for (int i = 0; i < (int)paths.size(); i++) {

        vector<int> p = paths[i];

        double distance = pathDistance(graph, p);

        double loadCost = calculateLoadCost(
            graph,
            p,
            load
        );

        double cost;

        if (mode == 0) {

            // Shortest path
            cost = distance;
        }
        else if (mode == 1) {

            // Greedy load balancing
            cost = loadCost;
        }
        else {

            // Proposed combined cost
            cost = distance + lambda * loadCost;
        }

        if (cost < minimumCost) {

            minimumCost = cost;

            bestPath = p;
        }
    }

    return bestPath;
}

// ==========================================
// STEP 5: SIMULATION
// ==========================================

Result simulate(
    const vector<vector<Edge>> &graph,
    const vector<vector<Request>> &requests,
    int K,
    double lambda,
    int mode
) {
    int V = graph.size();

    int E = 0;

    for (int i = 0; i < V; i++)
        E += graph[i].size();

    E /= 2;

    Result result;

    result.roadLoad.assign(E, 0);

    result.assignments.resize(requests.size());

    result.loadHistory.resize(requests.size());

    auto start = chrono::high_resolution_clock::now();

    // Process time steps

    for (int t = 0; t < (int)requests.size(); t++) {

        result.assignments[t].resize(requests[t].size());

        // Process requests

        for (int r = 0; r < (int)requests[t].size(); r++) {

            Request req = requests[t][r];

            vector<vector<int>> candidatePaths;

            if (mode == 0) {

                // Shortest path baseline

                vector<bool> bannedNodes(V, false);
                vector<bool> bannedEdges(E, false);

                vector<int> p = dijkstra(
                    graph,
                    req.source,
                    req.destination,
                    bannedNodes,
                    bannedEdges
                );

                if (!p.empty())
                    candidatePaths.push_back(p);
            }
            else {

                // Generate K candidate paths using Yen

                candidatePaths = yenKShortestPaths(
                    graph,
                    req.source,
                    req.destination,
                    K
                );
            }

            // Assign vehicles one by one

            for (int vehicle = 0; vehicle < req.vehicles; vehicle++) {

                if (candidatePaths.empty())
                    continue;

                vector<int> selected = selectPath(
                    candidatePaths,
                    graph,
                    result.roadLoad,
                    mode,
                    lambda
                );

                // Store path and vehicle count

                result.assignments[t][r][selected]++;

                // Update road loads

                for (int i = 0; i + 1 < (int)selected.size(); i++) {

                    int id = findEdgeId(
                        graph,
                        selected[i],
                        selected[i + 1]
                    );

                    if (id != -1)
                        result.roadLoad[id]++;
                }

                // Update total travel distance

                result.totalDistance += pathDistance(
                    graph,
                    selected
                );
            }
        }

        // Save cumulative road loads after each time step

        result.loadHistory[t] = result.roadLoad;
    }

    auto finish = chrono::high_resolution_clock::now();

    result.runningTime =
        chrono::duration<double, milli>(finish - start).count();

    return result;
}

// ==========================================
// DISPLAY PATH
// ==========================================

void printPath(const vector<int> &path) {

    for (int i = 0; i < (int)path.size(); i++) {

        cout << path[i];

        if (i + 1 < (int)path.size())
            cout << " -> ";
    }
}

// ==========================================
// DISPLAY ASSIGNMENTS
// ==========================================

void displayAssignments(
    const Result &result,
    const vector<vector<Request>> &requests
) {
    for (int t = 0; t < (int)requests.size(); t++) {

        cout << "\nTIME STEP " << t + 1 << "\n";

        for (int r = 0; r < (int)requests[t].size(); r++) {

            cout << "\nRequest " << r + 1
                 << " (" << requests[t][r].vehicles
                 << " vehicles, "
                 << requests[t][r].source
                 << " -> "
                 << requests[t][r].destination
                 << ")\n";

            for (auto it = result.assignments[t][r].begin();
                 it != result.assignments[t][r].end();
                 it++) {

                cout << "  Path: ";

                printPath(it->first);

                cout << " | Vehicles: " << it->second << "\n";
            }
        }

        // Display road loads after this time step

        cout << "\nCumulative Road Loads:\n";

        for (int e = 0; e < (int)result.loadHistory[t].size(); e++) {

            cout << "Road " << e << ": "
                 << result.loadHistory[t][e] << " vehicles";

            if (e + 1 < (int)result.loadHistory[t].size())
                cout << " | ";

            if ((e + 1) % 4 == 0)
                cout << "\n";
        }

        cout << "\n";
    }
}

// ==========================================
// STEP 6: PERFORMANCE EVALUATION
// ==========================================

Metrics evaluate(const Result &result) {

    Metrics m;

    int E = result.roadLoad.size();

    if (E == 0)
        return m;

    int totalLoad = 0;

    for (int i = 0; i < E; i++) {

        totalLoad += result.roadLoad[i];

        m.maxLoad = max(
            m.maxLoad,
            result.roadLoad[i]
        );
    }

    m.averageLoad = (double)totalLoad / E;

    double varianceSum = 0;

    for (int i = 0; i < E; i++) {

        double difference =
            result.roadLoad[i] - m.averageLoad;

        varianceSum += difference * difference;
    }

    m.variance = varianceSum / E;

    m.totalDistance = result.totalDistance;

    m.runningTime = result.runningTime;

    return m;
}

// ==========================================
// DISPLAY COMPARISON
// ==========================================

void displayComparison(
    const Metrics &shortest,
    const Metrics &greedy,
    const Metrics &proposed
) {
    cout << "\n========== PERFORMANCE COMPARISON ==========\n";

    cout << left
         << setw(18) << "Algorithm"
         << setw(12) << "Max Load"
         << setw(13) << "Avg Load"
         << setw(13) << "Variance"
         << setw(16) << "Distance"
         << setw(13) << "Time(ms)"
         << "\n";

    cout << string(85, '-') << "\n";

    cout << fixed << setprecision(2);

    cout << left << setw(18) << "Shortest Path"
         << setw(12) << shortest.maxLoad
         << setw(13) << shortest.averageLoad
         << setw(13) << shortest.variance
         << setw(16) << shortest.totalDistance
         << setw(13) << shortest.runningTime
         << "\n";

    cout << left << setw(18) << "Greedy"
         << setw(12) << greedy.maxLoad
         << setw(13) << greedy.averageLoad
         << setw(13) << greedy.variance
         << setw(16) << greedy.totalDistance
         << setw(13) << greedy.runningTime
         << "\n";

    cout << left << setw(18) << "Proposed"
         << setw(12) << proposed.maxLoad
         << setw(13) << proposed.averageLoad
         << setw(13) << proposed.variance
         << setw(16) << proposed.totalDistance
         << setw(13) << proposed.runningTime
         << "\n";
}

// ==========================================
// MAIN FUNCTION
// ==========================================

int main() {

    srand(RANDOM_SEED);

    vector<vector<Edge>> graph;

    vector<Road> roads;

    vector<vector<Request>> requests;

    // STEP 1

    cout << "\n========================================\n";
    cout << "       TRAFFIC LOAD BALANCING SYSTEM\n";
    cout << "========================================\n";

    cout << "\n[STEP 1] RANDOM ROAD NETWORK\n";

    generateGraph(
        V,
        EXTRA_EDGES,
        graph,
        roads
    );

    cout << "Vertices: " << V << "\n";
    cout << "Roads: " << roads.size() << "\n";
    cout << "Status: Connected\n";

    // STEP 2

    cout << "\n[STEP 2] TRANSPORTATION REQUESTS\n";

    generateRequests(V, requests);

    for (int t = 0; t < TIME_STEPS; t++) {

        cout << "\nTime Step " << t + 1 << ":\n";

        for (int r = 0; r < REQUESTS_PER_STEP; r++) {

            Request req = requests[t][r];

            cout << "Request " << r + 1
                 << ": " << req.vehicles
                 << " vehicles ("
                 << req.source << " -> "
                 << req.destination << ")\n";
        }
    }

    // STEP 3

    cout << "\n[STEP 3] SHORTEST PATH ROUTING\n";

    Result shortestResult = simulate(
        graph, requests, K, LAMBDA, 0
    );

    displayAssignments(shortestResult, requests);

    Metrics shortestMetrics = evaluate(shortestResult);

    // STEP 4

    cout << "\n[STEP 4] GREEDY LOAD BALANCING\n";

    Result greedyResult = simulate(
        graph, requests, K, LAMBDA, 1
    );

    displayAssignments(greedyResult, requests);

    Metrics greedyMetrics = evaluate(greedyResult);

    // STEP 5

    cout << "\n[STEP 5] PROPOSED ALGORITHM\n";

    cout << "K = " << K << "\n";
    cout << "Lambda = " << LAMBDA << "\n";

    Result proposedResult = simulate(
        graph, requests, K, LAMBDA, 2
    );

    displayAssignments(proposedResult, requests);

    Metrics proposedMetrics = evaluate(proposedResult);

    // STEP 6

    cout << "\n[STEP 6] FINAL PERFORMANCE COMPARISON\n";

    displayComparison(
        shortestMetrics,
        greedyMetrics,
        proposedMetrics
    );

    cout << "\n========================================\n";
    cout << "          SIMULATION COMPLETED\n";
    cout << "========================================\n";

    return 0;
}
