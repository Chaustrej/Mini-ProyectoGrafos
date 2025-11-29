#ifndef GRAPH_VIEW_H
#define GRAPH_VIEW_H

#include "../controller/graph_controller.h"

// Prototipos de funciones de visualización
void graph_print(Graph* graph);
void graph_print_traversal(AlgorithmResult result);
void graph_print_distances(AlgorithmResult result);
void graph_print_mst(AlgorithmResult result);
void graph_print_components(std::vector<std::vector<int>> components);
void graph_print_menu();

#endif