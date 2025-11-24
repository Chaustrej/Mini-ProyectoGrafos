# Proyecto de Grafos (EDD)

Este proyecto implementa una **librería de grafos** en C++ organizada con el patrón **Modelo–Controlador–Vista (MVC)** y una pequeña **aplicación de consola** que permite crear un grafo, añadir vértices y aristas, y ejecutar varios algoritmos clásicos.

- `model/`: representación interna del grafo y operaciones básicas (CRUD de vértices/aristas).
- `controller/`: algoritmos sobre el grafo (BFS, DFS, Dijkstra, Prim, Kruskal, componentes conexas, etc.).
- `view/`: toda la salida por consola (impresión de grafos, recorridos, distancias, menús).
- `main.cpp`: punto de entrada; conecta la vista con el modelo y el controlador.

A continuación se explica **qué hace cada archivo, cada estructura y cada función**, así como el funcionamiento de los algoritmos.

---

## 1. `main.cpp` – Punto de entrada

```cpp
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
        ...
    } while (opcion != 0);

    return 0;
}
```

### Flujo general de `main`

1. **Crear grafo**: `Graph graph = graph_create(UNDIRECTED);`
   - Crea un grafo vacío no dirigido.
2. **Datos de ejemplo**: se añaden aristas entre vértices `1..5` para tener un grafo inicial.
3. **Bucle de menú**:
   - Llama a `graph_print_menu()` (vista) para mostrar las opciones.
   - Lee la opción del usuario (`opcion`).
   - Según la opción, llama a funciones del **modelo** (manipulación de grafo) y del **controlador** (algoritmos), y finalmente a la **vista** para mostrar los resultados.

### Opciones del menú

- **1. Agregar vértice**
  - Pide el número del vértice.
  - Llama a `graph_add_vertex(&graph, vertex);` (modelo).

- **2. Agregar arista**
  - Pide `origen`, `destino` y `peso`.
  - Llama a `graph_add_edge(&graph, source, dest, weight);`.

- **3. Mostrar grafo**
  - Llama a `graph_print(&graph);` (vista), que muestra tipo, número de vértices, número de aristas y la lista de adyacencia.

- **4. BFS** (búsqueda en anchura)
  - Pide vértice inicial.
  - Llama a `graph_bfs(&graph, vertex);` (controlador) y luego a `graph_print_traversal(result);` (vista).

- **5. DFS** (búsqueda en profundidad)
  - Igual que BFS pero con `graph_dfs`.

- **6. Dijkstra**
  - Pide vértice origen.
  - `graph_dijkstra(&graph, vertex);` calcula las distancias mínimas.
  - `graph_print_distances(result);` muestra las distancias.

- **7. Prim (MST)**
  - `graph_prim(&graph);` calcula un **árbol de expansión mínima**.
  - `graph_print_mst(result);` muestra las aristas del MST y el peso total.

- **8. Kruskal (MST)**
  - Igual que Prim pero con el algoritmo de Kruskal.

- **9. Componentes conexas**
  - `graph_connected_components(&graph);` obtiene todas las componentes.
  - `graph_print_components(components);` las muestra.

- **10. Verificar conexión**
  - `graph_is_connected(&graph);` devuelve `true` si el grafo es conexo (una sola componente).

---

## 2. `model/graph_model.*` – Representación del grafo

### 2.1 Estructuras principales

```cpp
typedef enum {
    UNDIRECTED,
    DIRECTED
} GraphType;
```
- Define si el grafo es **no dirigido** o **dirigido**.

```cpp
typedef struct {
    int source;
    int destination;
    double weight;
} Edge;
```
- Representa una arista:
  - `source`: vértice origen.
  - `destination`: vértice destino.
  - `weight`: peso (coste, distancia, etc.).

```cpp
typedef struct {
    GraphType type;
    std::unordered_map<int, std::vector<Edge>> adjacencyList;
    int nextVertexId;
} Graph;
```
- Estructura principal del grafo:
  - `type`: tipo (dirigido/no dirigido).
  - `adjacencyList`: mapa `vértice -> lista de aristas salientes`.
  - `nextVertexId`: contador no usado en este código, pero preparado para generación automática de IDs.

```cpp
typedef struct {
    std::vector<int> traversal;
    std::unordered_map<int, double> distances;
    std::unordered_map<int, int> predecessors;
    std::vector<Edge> mstEdges;
} AlgorithmResult;
```
- Estructura genérica para devolver resultados de algoritmos:
  - `traversal`: orden de recorrido (para BFS, DFS, componentes).
  - `distances`: distancia mínima desde un origen (Dijkstra, Bellman-Ford).
  - `predecessors`: predecesor inmediato en la ruta más corta.
  - `mstEdges`: aristas que forman el árbol de expansión mínima.

### 2.2 Funciones del modelo

#### `Graph graph_create(GraphType type);`
Crea un grafo vacío del tipo indicado:
- Asigna el tipo.
- Inicializa `nextVertexId` a 0.
- La `adjacencyList` empieza vacía.

#### `void graph_add_vertex(Graph* graph, int vertex);`
- Si el vértice no existe en `adjacencyList`, crea una entrada con una lista de aristas vacía.
- No añade nada si ya existe (idempotente).

#### `void graph_remove_vertex(Graph* graph, int vertex);`
1. Elimina la entrada del vértice en `adjacencyList`.
2. Recorre todas las listas de aristas y elimina cualquier arista cuya `destination` sea ese vértice.

#### `void graph_add_edge(Graph* graph, int source, int dest, double weight);`
1. Asegura que `source` y `dest` existan llamando a `graph_add_vertex`.
2. Añade a `adjacencyList[source]` una arista `{source, dest, weight}`.
3. Si el grafo es `UNDIRECTED`, añade también la arista inversa `{dest, source, weight}`.

#### `void graph_remove_edge(Graph* graph, int source, int dest);`
- Elimina la arista `source → dest` de la lista de `source`.
- Si el grafo es `UNDIRECTED`, también elimina `dest → source`.

#### `bool graph_has_vertex(Graph* graph, int vertex);`
- Devuelve `true` si `vertex` está en `adjacencyList`.

#### `bool graph_has_edge(Graph* graph, int source, int dest);`
- Comprueba si desde `source` hay alguna arista con `destination == dest`.

#### `std::vector<int> graph_get_vertices(Graph* graph);`
- Recorre el `unordered_map` y devuelve un vector con todos los vértices existentes.

#### `std::vector<Edge> graph_get_edges(Graph* graph);`
- Recorre todo el mapa `adjacencyList` y concatena todas las aristas en un solo vector.
- En grafos no dirigidos, cada arista aparece **dos veces** (una por cada sentido).

#### `std::vector<int> graph_get_neighbors(Graph* graph, int vertex);`
- Devuelve todos los destinos de las aristas que salen de `vertex`.
- Se usa en BFS, DFS, etc.

#### `double graph_get_weight(Graph* graph, int source, int dest);`
- Si existe arista `source → dest`, devuelve su peso.
- Si no existe, devuelve `-1`.

#### `int graph_vertex_count(Graph* graph);`
- Devuelve el número de claves en `adjacencyList`, es decir, el número de vértices.

#### `int graph_edge_count(Graph* graph);`
- Suma el tamaño de todas las listas de aristas.
- Si es `UNDIRECTED`, divide entre 2 (porque se guardan dos aristas por conexión).

---

## 3. `controller/graph_controller.*` – Algoritmos sobre grafos

El controlador no almacena datos del grafo: **solo implementa algoritmos** que operan sobre `Graph` del modelo.

### 3.1 `AlgorithmResult graph_bfs(Graph* graph, int start);`

**BFS (Breadth-First Search)** – Búsqueda en anchura.

- Verifica que el vértice inicial `start` exista (`graph_has_vertex`).
- Usa:
  - `std::unordered_map<int, bool> visited` para marcar visitados.
  - `std::queue<int> queue` como cola FIFO de vértices por procesar.
- Pasos:
  1. Marca `start` como visitado y lo encola.
  2. Mientras la cola no esté vacía:
     - Desencola el vértice `current`.
     - Lo añade a `result.traversal`.
     - Para cada vecino de `current` (`graph_get_neighbors`):
       - Si no está visitado, lo marca visitado y lo encola.
- Resultado: `result.traversal` contiene el orden de visita por niveles desde `start`.

### 3.2 `AlgorithmResult graph_dfs(Graph* graph, int start);`

**DFS (Depth-First Search)** – Búsqueda en profundidad.

- Similar a BFS pero usa una **pila** `std::stack<int>` en lugar de una cola.
- Pasos:
  1. Mete `start` en la pila.
  2. Mientras la pila tenga elementos:
     - Saca el tope `current`.
     - Si `current` **no** estaba visitado:
       - Lo marca visitado.
       - Lo añade a `result.traversal`.
       - Mete todos sus vecinos en la pila (se procesarán más tarde, profundizando en una rama).
- Resultado: `result.traversal` da el orden de recorrido en profundidad.

### 3.3 `AlgorithmResult graph_dijkstra(Graph* graph, int start);`

Implementa el algoritmo de **Dijkstra** para caminos mínimos desde un origen.

- Comprueba que `start` exista.
- Prepara una **cola de prioridad mínima**:
  ```cpp
  auto compare = [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
      return a.first > b.first;
  };
  std::priority_queue<
      std::pair<double, int>,
      std::vector<std::pair<double, int>>,
      decltype(compare)> pq(compare);
  ```
- Inicialización:
  - Para cada vértice, pone `distances[vertex] = +inf` y `predecessors[vertex] = -1`.
  - Para `start`, `distances[start] = 0` y lo inserta en la cola.
- Bucle principal:
  1. Extrae el par `(currentDist, current)` con menor distancia de la cola.
  2. Si `currentDist` es mayor que `result.distances[current]`, lo ignora (es una entrada vieja).
  3. Para cada arista saliente `current → neighbor` con peso `edge.weight`:
     - Calcula `newDist = currentDist + edge.weight`.
     - Si `newDist` mejora la distancia almacenada:
       - Actualiza `result.distances[neighbor] = newDist`.
       - Actualiza `result.predecessors[neighbor] = current`.
       - Inserta el nuevo par `(newDist, neighbor)` en la cola.
- Resultado:
  - `distances[v]` contiene la distancia mínima desde `start` hasta `v`.
  - `predecessors[v]` permite reconstruir el camino.

> Nota: El código usa directamente `graph->adjacencyList[current]` en lugar de `graph_get_neighbors`, para poder acceder al peso de cada arista.

### 3.4 `AlgorithmResult graph_prim(Graph* graph);`

Implementa el algoritmo de **Prim** para el **árbol de expansión mínima (MST)**. Solo funciona en grafos **no dirigidos** (`graph->type == UNDIRECTED`).

- Si el grafo es dirigido o no tiene vértices, devuelve un `AlgorithmResult` vacío.
- Usa:
  - `std::unordered_map<int, bool> inMST` para marcar qué vértices ya están en el MST.
  - Cola de prioridad mínima de aristas según su `weight`.
- Pasos:
  1. Elige un vértice inicial: `int start = graph_get_vertices(graph)[0];`.
  2. Marca `start` como incluido en el MST.
  3. Inserta todas las aristas que salen de `start` en la cola.
  4. Mientras la cola no esté vacía y el MST tenga menos de `vertexCount - 1` aristas:
     - Toma la arista de menor peso `edge`.
     - Si el vértice destino de `edge` **ya está** en el MST, la salta.
     - En caso contrario:
       - Añade `edge` a `result.mstEdges`.
       - Marca `edge.destination` como parte del MST.
       - Inserta todas las aristas que salen de ese nuevo vértice y van a vértices que aún no están en el MST.
- Resultado: `result.mstEdges` contiene las aristas del MST.

### 3.5 `AlgorithmResult graph_kruskal(Graph* graph);`

Implementa el algoritmo de **Kruskal** para el MST usando una estructura **Union-Find (Disjoint Set)**.

- Solo se ejecuta si el grafo es `UNDIRECTED`.
- Pasos:
  1. Obtiene todas las aristas con `graph_get_edges(graph);`.
  2. Ordena el vector de aristas por peso de menor a mayor.
  3. Inicializa dos mapas:
     - `parent[v] = v` (padre de cada conjunto).
     - `rank[v] = 0` (profundidad aproximada del árbol).
  4. Define `find(x)`: busca el representante del conjunto de `x` con compresión de caminos.
  5. Define `unite(x, y)`: une los conjuntos de `x` y `y` usando `rank` para mantener el árbol balanceado. Devuelve `true` si se han unido, `false` si ya estaban conectados.
  6. Recorre las aristas ya ordenadas:
     - Si `unite(edge.source, edge.destination)` devuelve `true`, esa arista **no forma ciclo** y se añade al MST (`result.mstEdges`).
- Resultado: `result.mstEdges` es el MST.

### 3.6 Funciones de utilidad

#### `bool graph_is_connected(Graph* graph);`

- Si no hay vértices, considera el grafo como conexo (`true`).
- Si hay vértices:
  1. Obtiene la lista de vértices.
  2. Llama a `graph_bfs` desde el primero.
  3. Compara el tamaño del recorrido `bfsResult.traversal.size()` con `graph_vertex_count(graph)`.
  4. Devuelve `true` si se alcanzaron todos los vértices.

#### `std::vector<std::vector<int>> graph_connected_components(Graph* graph);`

- Devuelve un vector de componentes, donde cada componente es un `vector<int>` de vértices.
- Usa un mapa `visited` para marcar los vértices ya procesados.
- Para cada vértice del grafo:
  - Si no está visitado:
    - Llama a `graph_bfs(graph, vertex);` para obtener la componente conectada a él.
    - Añade el recorrido (`traversal`) como una nueva componente.
    - Marca todos los vértices de ese recorrido como visitados.

---

## 4. `view/graph_view.*` – Interacción por consola

La vista **no modifica el grafo**; solo lo muestra por pantalla y ofrece el menú.

### 4.1 `void graph_print(Graph* graph);`

- Muestra información general del grafo:
  - Tipo: "No Dirigido" o "Dirigido" según `graph->type`.
  - Número de vértices: `graph_vertex_count(graph)`.
  - Número de aristas: `graph_edge_count(graph)`.
- Luego imprime la **lista de adyacencia**:
  - Para cada vértice `v` en `adjacencyList`:
    - Imprime `v ->` seguido de cada vecino `destination(weight)`.

### 4.2 `void graph_print_traversal(AlgorithmResult result);`

- Imprime `Recorrido:` seguido de los vértices en `result.traversal`.
- Se usa para mostrar BFS, DFS y componentes (indirectamente).

### 4.3 `void graph_print_distances(AlgorithmResult result);`

- Imprime `Distancias desde el origen:`.
- Para cada par `(vertex, distance)` en `result.distances`, muestra `Hasta vertex: distance`.
- Se usa para Dijkstra (y sería útil para Bellman-Ford si se implementa).

### 4.4 `void graph_print_mst(AlgorithmResult result);`

- Imprime `Arbol de Expansion Minima:`.
- Recorre `result.mstEdges` y para cada arista imprime `source - destination (peso: weight)`.
- Acumula `totalWeight` y al final imprime el peso total del MST.

### 4.5 `void graph_print_components(std::vector<std::vector<int>> components);`

- Muestra el número de componentes conexas.
- Para cada componente `i` imprime `Componente i: v1 v2 v3 ...`.

### 4.6 `void graph_print_menu();`

- Muestra el menú principal con las opciones disponibles y pide al usuario que seleccione una.
- `main.cpp` lee la opción con `std::cin`.

---

## 5. Cómo compilar y ejecutar

Desde la carpeta raíz del proyecto (`grafos_EDD`), en PowerShell:

```powershell
g++ main.cpp model/graph_model.cpp controller/graph_controller.cpp view/graph_view.cpp -o bin/grafos.exe

./bin/grafos.exe
```

(En Windows puedes ejecutar el `.exe` navegando a la carpeta `bin` o haciendo doble clic.)

---

