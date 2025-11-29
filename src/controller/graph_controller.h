#ifndef GRAPH_CONTROLLER_H
#define GRAPH_CONTROLLER_H

#include "../model/graph_model.h"
#include <queue>
#include <stack>
#include <limits>
#include <functional>
    
// Prototipos de algoritmos
AlgorithmResult graph_bfs(Graph* graph, int start);
AlgorithmResult graph_dfs(Graph* graph, int start);
AlgorithmResult graph_dijkstra(Graph* graph, int start);
AlgorithmResult graph_prim(Graph* graph);
AlgorithmResult graph_kruskal(Graph* graph);

// Utilidades
bool graph_is_connected(Graph* graph);
bool graph_has_cycle(Graph* graph);
std::vector<std::vector<int>> graph_connected_components(Graph* graph);

#endif