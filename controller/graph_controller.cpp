#include "graph_controller.h"
#include <algorithm>

AlgorithmResult graph_bfs(Graph* graph, int start) {
    AlgorithmResult result;
    if (!graph_has_vertex(graph, start)) return result;
    
    std::unordered_map<int, bool> visited;
    std::queue<int> queue;
    
    queue.push(start);
    visited[start] = true;
    
    while (!queue.empty()) {
        int current = queue.front();
        queue.pop();
        result.traversal.push_back(current);
        
        for (int neighbor : graph_get_neighbors(graph, current)) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                queue.push(neighbor);
            }
        }
    }
    
    return result;
}

AlgorithmResult graph_dfs(Graph* graph, int start) {
    AlgorithmResult result;
    if (!graph_has_vertex(graph, start)) return result;
    
    std::unordered_map<int, bool> visited;
    std::stack<int> stack;
    
    stack.push(start);
    
    while (!stack.empty()) {
        int current = stack.top();
        stack.pop();
        
        if (!visited[current]) {
            visited[current] = true;
            result.traversal.push_back(current);
            
            for (int neighbor : graph_get_neighbors(graph, current)) {
                if (!visited[neighbor]) {
                    stack.push(neighbor);
                }
            }
        }
    }
    
    return result;
}

AlgorithmResult graph_dijkstra(Graph* graph, int start) {
    AlgorithmResult result;
    if (!graph_has_vertex(graph, start)) return result;
    
    auto compare = [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
        return a.first > b.first;
    };
    std::priority_queue<
        std::pair<double, int>,
        std::vector<std::pair<double, int>>,
        decltype(compare)> pq(compare);
    
    // Inicializar distancias
    for (int vertex : graph_get_vertices(graph)) {
        result.distances[vertex] = std::numeric_limits<double>::infinity();
        result.predecessors[vertex] = -1;
    }
    
    result.distances[start] = 0;
    pq.push({0, start});
    
    while (!pq.empty()) {
        auto [currentDist, current] = pq.top();
        pq.pop();
        
        if (currentDist > result.distances[current]) continue;
        
        for (const auto& edge : graph->adjacencyList[current]) {
            int neighbor = edge.destination;
            double newDist = currentDist + edge.weight;
            
            if (newDist < result.distances[neighbor]) {
                result.distances[neighbor] = newDist;
                result.predecessors[neighbor] = current;
                pq.push({newDist, neighbor});
            }
        }
    }
    
    return result;
}

AlgorithmResult graph_prim(Graph* graph) {
    AlgorithmResult result;
    if (graph->type != UNDIRECTED || graph_vertex_count(graph) == 0) return result;
    
    std::unordered_map<int, bool> inMST;
    auto compare = [](const Edge& a, const Edge& b) {
        return a.weight > b.weight;
    };
    std::priority_queue<Edge, std::vector<Edge>, decltype(compare)> pq(compare);
    
    // Comenzar con el primer vértice
    int start = graph_get_vertices(graph)[0];
    inMST[start] = true;
    
    // Agregar aristas del vértice inicial
    for (const auto& edge : graph->adjacencyList[start]) {
        pq.push(edge);
    }
    
    while (!pq.empty() && result.mstEdges.size() < graph_vertex_count(graph) - 1) {
        Edge edge = pq.top();
        pq.pop();
        
        if (inMST[edge.destination]) continue;
        
        result.mstEdges.push_back(edge);
        inMST[edge.destination] = true;
        
        // Agregar nuevas aristas
        for (const auto& newEdge : graph->adjacencyList[edge.destination]) {
            if (!inMST[newEdge.destination]) {
                pq.push(newEdge);
            }
        }
    }
    
    return result;
}

AlgorithmResult graph_kruskal(Graph* graph) {
    AlgorithmResult result;
    if (graph->type != UNDIRECTED) return result;
    
    auto edges = graph_get_edges(graph);
    std::sort(edges.begin(), edges.end(),
        [](const Edge& a, const Edge& b) {
            return a.weight < b.weight;
        });
    
    std::unordered_map<int, int> parent;
    std::unordered_map<int, int> rank;
    
    // Función para encontrar el representante
    std::function<int(int)> find = [&](int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    };
    
    // Función para unir dos conjuntos
    auto unite = [&](int x, int y) {
        int rootX = find(x);
        int rootY = find(y);
        
        if (rootX != rootY) {
            if (rank[rootX] < rank[rootY]) {
                parent[rootX] = rootY;
            } else if (rank[rootX] > rank[rootY]) {
                parent[rootY] = rootX;
            } else {
                parent[rootY] = rootX;
                rank[rootX]++;
            }
            return true;
        }
        return false;
    };
    
    // Inicializar estructura union-find
    for (int vertex : graph_get_vertices(graph)) {
        parent[vertex] = vertex;
        rank[vertex] = 0;
    }
    
    // Construir MST
    for (const auto& edge : edges) {
        if (unite(edge.source, edge.destination)) {
            result.mstEdges.push_back(edge);
        }
    }
    
    return result;
}

bool graph_is_connected(Graph* graph) {
    if (graph_vertex_count(graph) == 0) return true;
    
    auto vertices = graph_get_vertices(graph);
    AlgorithmResult bfsResult = graph_bfs(graph, vertices[0]);
    return bfsResult.traversal.size() == graph_vertex_count(graph);
}

std::vector<std::vector<int>> graph_connected_components(Graph* graph) {
    std::vector<std::vector<int>> components;
    std::unordered_map<int, bool> visited;
    
    for (int vertex : graph_get_vertices(graph)) {
        if (!visited[vertex]) {
            AlgorithmResult bfsResult = graph_bfs(graph, vertex);
            components.push_back(bfsResult.traversal);
            
            for (int v : bfsResult.traversal) {
                visited[v] = true;
            }
        }
    }
    
    return components;
}