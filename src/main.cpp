#include <iostream>
#include "view/graph_view.h"

int main() {
    Graph graph = graph_create(UNDIRECTED);
    int opcion, vertex, source, dest;
    double weight;
    
    // Grafo de ejemplo
    graph_add_edge(&graph, 1, 2, 5.0);
    graph_add_edge(&graph, 1, 3, 3.0);
    graph_add_edge(&graph, 2, 4, 2.0);
    graph_add_edge(&graph, 3, 4, 1.0);
    graph_add_edge(&graph, 4, 5, 4.0);
    
    do {
        graph_print_menu();
        std::cin >> opcion;
        
        switch (opcion) {
            case 1:
                std::cout << "Ingrese vertice: ";
                std::cin >> vertex;
                graph_add_vertex(&graph, vertex);
                break;
                
            case 2:
                std::cout << "Ingrese origen, destino y peso: ";
                std::cin >> source >> dest >> weight;
                graph_add_edge(&graph, source, dest, weight);
                break;
                
            case 3:
                graph_print(&graph);
                break;
                
            case 4: {
                std::cout << "Ingrese vertice inicial: ";
                std::cin >> vertex;
                AlgorithmResult result = graph_bfs(&graph, vertex);
                graph_print_traversal(result);
                break;
            }
                
            case 5: {
                std::cout << "Ingrese vertice inicial: ";
                std::cin >> vertex;
                AlgorithmResult result = graph_dfs(&graph, vertex);
                graph_print_traversal(result);
                break;
            }
                
            case 6: {
                std::cout << "Ingrese vertice inicial: ";
                std::cin >> vertex;
                AlgorithmResult result = graph_dijkstra(&graph, vertex);
                graph_print_distances(result);
                break;
            }
                
            case 7: {
                AlgorithmResult result = graph_prim(&graph);
                graph_print_mst(result);
                break;
            }
                
            case 8: {
                AlgorithmResult result = graph_kruskal(&graph);
                graph_print_mst(result);
                break;
            }
                
            case 9: {
                auto components = graph_connected_components(&graph);
                graph_print_components(components);
                break;
            }
                
            case 10:
                std::cout << "El grafo " << (graph_is_connected(&graph) ? "esta" : "no esta") 
                          << " conectado" << std::endl;
                break;
                
            case 0:
                std::cout << "Hasta luego!" << std::endl;
                break;
                
            default:
                std::cout << "Opcion invalida" << std::endl;
        }
        
    } while (opcion != 0);
    
    return 0;
}