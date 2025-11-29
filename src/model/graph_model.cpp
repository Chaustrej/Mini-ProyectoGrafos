#include "graph_model.h"
#include <algorithm>

Graph graph_create(GraphType type) {
    Graph graph;
    graph.type = type;
    graph.nextVertexId = 0;
    return graph;
}

void graph_add_vertex(Graph* graph, int vertex) {
    if (graph->adjacencyList.find(vertex) == graph->adjacencyList.end()) {
        graph->adjacencyList[vertex] = std::vector<Edge>();
    }
}

void graph_remove_vertex(Graph* graph, int vertex) {
    // Remover el vértice
    graph->adjacencyList.erase(vertex);
    
    // Remover aristas que apuntan al vértice
    for (auto& pair : graph->adjacencyList) {
        auto& edges = pair.second;
        edges.erase(
            std::remove_if(edges.begin(), edges.end(),
                [vertex](const Edge& edge) {
                    return edge.destination == vertex;
                }),
            edges.end()
        );
    }
}

void graph_add_edge(Graph* graph, int source, int dest, double weight) {
    graph_add_vertex(graph, source);
    graph_add_vertex(graph, dest);
    
    graph->adjacencyList[source].push_back({source, dest, weight});
    
    if (graph->type == UNDIRECTED) {
        graph->adjacencyList[dest].push_back({dest, source, weight});
    }
}

void graph_remove_edge(Graph* graph, int source, int dest) {
    if (graph->adjacencyList.find(source) != graph->adjacencyList.end()) {
        auto& edges = graph->adjacencyList[source];
        edges.erase(
            std::remove_if(edges.begin(), edges.end(),
                [dest](const Edge& edge) {
                    return edge.destination == dest;
                }),
            edges.end()
        );
    }
    
    if (graph->type == UNDIRECTED && graph->adjacencyList.find(dest) != graph->adjacencyList.end()) {
        auto& edges = graph->adjacencyList[dest];
        edges.erase(
            std::remove_if(edges.begin(), edges.end(),
                [source](const Edge& edge) {
                    return edge.destination == source;
                }),
            edges.end()
        );
    }
}

bool graph_has_vertex(Graph* graph, int vertex) {
    return graph->adjacencyList.find(vertex) != graph->adjacencyList.end();
}

bool graph_has_edge(Graph* graph, int source, int dest) {
    if (!graph_has_vertex(graph, source)) return false;
    
    const auto& edges = graph->adjacencyList[source];
    return std::any_of(edges.begin(), edges.end(),
        [dest](const Edge& edge) {
            return edge.destination == dest;
        });
}

std::vector<int> graph_get_vertices(Graph* graph) {
    std::vector<int> vertices;
    for (const auto& pair : graph->adjacencyList) {
        vertices.push_back(pair.first);
    }
    return vertices;
}

std::vector<Edge> graph_get_edges(Graph* graph) {
    std::vector<Edge> edges;
    for (const auto& pair : graph->adjacencyList) {
        edges.insert(edges.end(), pair.second.begin(), pair.second.end());
    }
    return edges;
}

std::vector<int> graph_get_neighbors(Graph* graph, int vertex) {
    std::vector<int> neighbors;
    if (graph_has_vertex(graph, vertex)) {
        for (const auto& edge : graph->adjacencyList[vertex]) {
            neighbors.push_back(edge.destination);
        }
    }
    return neighbors;
}

double graph_get_weight(Graph* graph, int source, int dest) {
    if (!graph_has_edge(graph, source, dest)) return -1;
    
    const auto& edges = graph->adjacencyList[source];
    auto it = std::find_if(edges.begin(), edges.end(),
        [dest](const Edge& edge) {
            return edge.destination == dest;
        });
    
    return it->weight;
}

int graph_vertex_count(Graph* graph) {
    return graph->adjacencyList.size();
}

int graph_edge_count(Graph* graph) {
    int count = 0;
    for (const auto& pair : graph->adjacencyList) {
        count += pair.second.size();
    }
    return graph->type == UNDIRECTED ? count / 2 : count;
}