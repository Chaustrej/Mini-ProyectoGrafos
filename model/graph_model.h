#ifndef GRAPH_MODEL_H
#define GRAPH_MODEL_H

#include <vector>
#include <unordered_map>
#include <string>

// Tipos de grafo
typedef enum {
    UNDIRECTED,
    DIRECTED
} GraphType;

// Estructura para representar una arista
typedef struct {
    int source;
    int destination;
    double weight;
} Edge;

// Estructura principal del grafo
typedef struct {
    GraphType type;
    std::unordered_map<int, std::vector<Edge>> adjacencyList;
    int nextVertexId;
} Graph;

// Resultados de algoritmos
typedef struct {
    std::vector<int> traversal;
    std::unordered_map<int, double> distances;
    std::unordered_map<int, int> predecessors;
    std::vector<Edge> mstEdges;
} AlgorithmResult;

// Prototipos de funciones del modelo
Graph graph_create(GraphType type);
void graph_add_vertex(Graph* graph, int vertex);
void graph_remove_vertex(Graph* graph, int vertex);
void graph_add_edge(Graph* graph, int source, int dest, double weight);
void graph_remove_edge(Graph* graph, int source, int dest);
bool graph_has_vertex(Graph* graph, int vertex);
bool graph_has_edge(Graph* graph, int source, int dest);
std::vector<int> graph_get_vertices(Graph* graph);
std::vector<Edge> graph_get_edges(Graph* graph);
std::vector<int> graph_get_neighbors(Graph* graph, int vertex);
double graph_get_weight(Graph* graph, int source, int dest);
int graph_vertex_count(Graph* graph);
int graph_edge_count(Graph* graph);

#endif