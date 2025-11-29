#include "graph_view.h"
#include <iostream>

using namespace std;

void graph_print(Graph* graph) {
    cout << "=== GRAFO ===" << endl;
    cout << "Tipo: " << (graph->type == UNDIRECTED ? "No Dirigido" : "Dirigido") << endl;
    cout << "Vertices: " << graph_vertex_count(graph) << endl;
    cout << "Aristas: " << graph_edge_count(graph) << endl;
    cout << "Lista de Adyacencia:" << endl;
    
    for (const auto& pair : graph->adjacencyList) {
        cout << pair.first << " -> ";
        for (const auto& edge : pair.second) {
            cout << edge.destination << "(" << edge.weight << ") ";
        }
        cout << endl;
    }
    cout << endl;
}

void graph_print_traversal(AlgorithmResult result) {
    cout << "Recorrido: ";
    for (int vertex : result.traversal) {
        cout << vertex << " ";
    }
    cout << endl;
}

void graph_print_distances(AlgorithmResult result) {
    cout << "Distancias desde el origen:" << endl;
    for (const auto& pair : result.distances) {
        cout << "  Hasta " << pair.first << ": " << pair.second << endl;
    }
}

void graph_print_mst(AlgorithmResult result) {
    cout << "Arbol de Expansion Minima:" << endl;
    double totalWeight = 0;
    for (const auto& edge : result.mstEdges) {
        cout << "  " << edge.source << " - " << edge.destination 
                  << " (peso: " << edge.weight << ")" << endl;
        totalWeight += edge.weight;
    }
    cout << "Peso total: " << totalWeight << endl;
}

void graph_print_components(vector<vector<int>> components) {
    cout << "Componentes Conexas: " << components.size() << endl;
    for (size_t i = 0; i < components.size(); i++) {
        cout << "Componente " << i + 1 << ": ";
        for (int vertex : components[i]) {
            cout << vertex << " ";
        }
        cout << endl;
    }
}

void graph_print_menu() {
    cout << "\n=== MENU GRAFOS ===" << endl;
    cout << "1. Agregar vertice" << endl;
    cout << "2. Agregar arista" << endl;
    cout << "3. Mostrar grafo" << endl;
    cout << "4. BFS" << endl;
    cout << "5. DFS" << endl;
    cout << "6. Dijkstra" << endl;
    cout << "7. Prim (MST)" << endl;
    cout << "8. Kruskal (MST)" << endl;
    cout << "9. Componentes conexas" << endl;
    cout << "10. Verificar conexion" << endl;
    cout << "0. Salir" << endl;
    cout << "Seleccione una opcion: ";
}