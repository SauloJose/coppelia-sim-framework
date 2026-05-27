import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

def scipy_shortest_path(grid_map, start_world, goal_world):
    # 1. Converte coordenadas do mundo real para índices da matriz
    start_row, start_col = grid_map.world_to_grid(start_world[0], start_world[1])
    goal_row, goal_col = grid_map.world_to_grid(goal_world[0], goal_world[1])
    
    matrix = grid_map.matrix
    rows, cols = matrix.shape
    n_nodes = rows * cols
    
    # Mapeamento de matriz 2D para ID único de nó (Dijkstra precisa de ID único)
    def get_node_id(r, c):
        return r * cols + c

    # 2. Construindo as conexões do Grafo (Vizinhos livres)
    sources = []
    targets = []
    weights = []
    
    for r in range(rows):
        for c in range(cols):
            if matrix[r, c] == 1: # Obstáculo, pula
                continue
            
            current_id = get_node_id(r, c)
            
            # Olha os vizinhos (cima, baixo, esquerda, direita)
            for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and matrix[nr, nc] == 0:
                    sources.append(current_id)
                    targets.append(get_node_id(nr, nc))
                    weights.append(1.0) # Custo do passo
                    
    # Transforma em Matriz Esparsa compressa (padrão que o SciPy exige)
    graph = csr_matrix((weights, (sources, targets)), shape=(n_nodes, n_nodes))
    
    # 3. Executa o Dijkstra ultra-rápido do SciPy
    start_id = get_node_id(start_row, start_col)
    goal_id = get_node_id(goal_row, goal_col)
    
    distances, predecessors = dijkstra(
        csgraph=graph, 
        directed=False, 
        indices=start_id, 
        return_predecessors=True
    )
    
    # 4. Reconstrói o caminho se houver rota
    if predecessors[goal_id] == -9999: # Código do SciPy para inacessível
        return None
        
    path_ids = []
    curr = goal_id
    while curr != start_id:
        path_ids.append(curr)
        curr = predecessors[curr]
    path_ids.append(start_id)
    path_ids.reverse()
    
    # 5. Converte IDs de volta para metros reais (X, Y)
    world_path = []
    for node_id in path_ids:
        r = node_id // cols
        c = node_id % cols
        cx, cy = grid_map.grid_to_world(r, c)
        world_path.append([cx, cy])
        
    return np.array(world_path)